#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <Adafruit_NeoPixel.h>

// --- LEDの設定 ---
#define LED_PIN 16
#define LED_COUNT 1
Adafruit_NeoPixel strip =  Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- USB MIDI オブジェクトのセットアップ ---
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// --- ピンと設定 ---
const int PIN_PITCH_BEND = 28; // A0, GP28
const int PIN_MODULATION = 29; // A1, GP29

// --- スティックの安定化設定 ---
const int MOVING_AVG_SAMPLES = 8;  // 移動平均に使うサンプル数 (大きいほど滑らか)
const int STICK_DEAD_ZONE = 128;   // スティックのデッドゾーンの幅
const int PITCH_SEND_THRESHOLD = 10; // ピッチベンドを送信する値の変化量
const int MOD_SEND_THRESHOLD = 2;    // モジュレーションを送信する値の変化量 (0-127の範囲なので小さめ)
// --- スティックの物理的な可動範囲を設定 ---
const int STICK_MIN_ADC = 500;     // スティックを倒しきった時の最小値
const int STICK_MAX_ADC = 3600;    // スティックを倒しきった時の最大値


// --- キーマトリクスの設定 ---
const byte ROWS = 4;
const byte COLS = 5;
const int colPins[COLS] = {5, 6, 7, 8, 9};
const int rowPins[ROWS] = {10, 11, 12, 13};

// --- 機能キーの定義 ---
const int FUNC_OCTAVE_DOWN = -2;
const int FUNC_OCTAVE_UP = -3;
const int FUNC_HOLD = -4;
const int noteMap[ROWS][COLS] = {
  {76, 74, 75, 73, 72},
  {71, 70, 69, 68, 67},
  {66, 65, 64, 63, 62},
  {61, 60, FUNC_OCTAVE_DOWN, FUNC_OCTAVE_UP, FUNC_HOLD}
};

// --- 状態管理変数 ---
bool keyState[ROWS][COLS] = {false};
int octaveOffset = 0;
int lastPitchBend = 0;
int lastModulation = 0;
const byte MIDI_CHANNEL = 1;
int LED_flg=0;

// --- キャリブレーション結果を保存する変数 ---
int pitchCenter = 2048;
int modCenter = 2048;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  // LEDの初期化
  strip.begin();
  strip.show();
  colorWipe(16, 16, 16);

  // アナログスティックのキャリブレーション
  Serial.println("Starting joystick calibration... Do not touch the sticks.");
  long pitchTotal = 0;
  long modTotal = 0;
  const int calibrationSamples = 16;
  for (int i = 0; i < calibrationSamples; i++) {
    pitchTotal += analogRead(PIN_PITCH_BEND);
    modTotal += analogRead(PIN_MODULATION);
    delay(1);
  }
  pitchCenter = pitchTotal / calibrationSamples;
  modCenter = modTotal / calibrationSamples;
  Serial.print("Calibration complete. Pitch Center: ");
  Serial.print(pitchCenter);
  Serial.print(", Mod Center: ");
  Serial.println(modCenter);


  // キーマトリクスのピン初期化
  for (int i = 0; i < COLS; i++) {
    pinMode(colPins[i], OUTPUT);
    digitalWrite(colPins[i], HIGH);
  }
  for (int i = 0; i < ROWS; i++) {
    pinMode(rowPins[i], INPUT_PULLUP);
  }

  // MIDIの初期化
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  TinyUSB_Device_Init(0);
#endif
  usb_midi.setStringDescriptor("TinyKeyboard.1");
  MIDI.begin(MIDI_CHANNEL_OMNI);

  while (!TinyUSBDevice.mounted()) delay(1);
  
  Serial.println("Key matrix MIDI controller Pro ready!");
  Serial.print("Octave: ");
  Serial.println(octaveOffset);
  
  colorWipe(0, 0, 0);
}

void loop() {
  scanKeyMatrix();
  readAnalogControls();
  delay(5);
  colorWipe(0, 0, 0);
}

void scanKeyMatrix() {
  for (int col = 0; col < COLS; col++) {
    digitalWrite(colPins[col], LOW);
    for (int row = 0; row < ROWS; row++) {
      bool isPressed = (digitalRead(rowPins[row]) == LOW);
      int keyAction = noteMap[row][col];
      if (isPressed != keyState[row][col]) {
        keyState[row][col] = isPressed;
        if (keyAction >= 0) {
          if (isPressed) {
            MIDI.sendNoteOn(keyAction + 12 * octaveOffset, 127, MIDI_CHANNEL);
            colorWipe(16, 10, 1);
          } else {
            MIDI.sendNoteOff(keyAction + 12 * octaveOffset, 0, MIDI_CHANNEL);
          }
        } else {
          if (isPressed) {
            int oldOctaveOffset = octaveOffset;
            bool octaveChanged = false;
            switch (keyAction) {
              case FUNC_OCTAVE_DOWN:
                octaveOffset--; octaveChanged = true; 
                colorWipe(16, 0, 0);
                break;
              case FUNC_OCTAVE_UP:
                octaveOffset++; octaveChanged = true; 
                colorWipe(0, 0, 16);
                break;
            }
            if (octaveChanged) {
              Serial.print("Octave: ");
              Serial.println(octaveOffset);
              for (int r = 0; r < ROWS; r++) {
                for (int c = 0; c < COLS; c++) {
                  if (keyState[r][c] && noteMap[r][c] >= 0) {
                    MIDI.sendNoteOff(noteMap[r][c] + 12 * oldOctaveOffset, 0, MIDI_CHANNEL);
                    MIDI.sendNoteOn(noteMap[r][c] + 12 * octaveOffset, 127, MIDI_CHANNEL);
                  }
                }
              }
            }
          }
          if (keyAction == FUNC_HOLD) {
            MIDI.sendControlChange(64, isPressed ? 127 : 0, MIDI_CHANNEL);
            colorWipe(16, 16, 16);
          }
        }
      }
    }
    digitalWrite(colPins[col], HIGH);
  }
}

// --- アナログコントロールを読み取る関数 (修正箇所) ---
void readAnalogControls() {
  // --- 移動平均のための静的変数 (MIDIスケール後の値を保存) ---
  static long pitchHistory[MOVING_AVG_SAMPLES] = {0};
  static int modHistory[MOVING_AVG_SAMPLES] = {0};
  static int historyIndex = 0;
  static long pitchSum = 0;
  static long modSum = 0;

  // --- 1. ADC値の読み取り ---
  int rawPitchValue = analogRead(PIN_PITCH_BEND);
  int rawModValue = analogRead(PIN_MODULATION);

  // --- 2. ADC値を直接MIDI値に変換 (スケーリング) ---
  // ピッチベンド
  int currentPitchBend = 0; // デッドゾーン内なら中央値 0
  if (rawPitchValue < pitchCenter - STICK_DEAD_ZONE) {
    // 物理的な最小値(STICK_MIN_ADC)をMIDIの最小値にマッピング
    if (rawPitchValue < STICK_MIN_ADC) currentPitchBend = -8192;
    else currentPitchBend = map(rawPitchValue, pitchCenter - STICK_DEAD_ZONE, STICK_MIN_ADC, 0, -8192);
  } else if (rawPitchValue > pitchCenter + STICK_DEAD_ZONE) {
    // 物理的な最大値(STICK_MAX_ADC)をMIDIの最大値にマッピング
    if(rawPitchValue > STICK_MAX_ADC) currentPitchBend = 8191;
    else currentPitchBend = map(rawPitchValue, pitchCenter + STICK_DEAD_ZONE, STICK_MAX_ADC, 0, 8191);
  }

  // モジュレーション
  int currentModulation = 0; // デッドゾーン内なら0
  if (rawModValue < modCenter - STICK_DEAD_ZONE) {
    currentModulation = map(rawModValue, modCenter - STICK_DEAD_ZONE, STICK_MIN_ADC, 0, 127);
  }

  // --- 3. スケール後のMIDI値で移動平均を計算 ---
  pitchSum -= pitchHistory[historyIndex];
  pitchHistory[historyIndex] = currentPitchBend;
  pitchSum += currentPitchBend;
  int avgPitchBend = pitchSum / MOVING_AVG_SAMPLES;

  modSum -= modHistory[historyIndex];
  modHistory[historyIndex] = currentModulation;
  modSum += currentModulation;
  int avgModulation = modSum / MOVING_AVG_SAMPLES;

  // 履歴のインデックスを次に進める
  historyIndex = (historyIndex + 1) % MOVING_AVG_SAMPLES;
  
  // --- 4. 閾値を超えた場合のみ、平均化されたMIDI信号を送信 ---
  if (abs(avgPitchBend - lastPitchBend) > PITCH_SEND_THRESHOLD) { 
    MIDI.sendPitchBend(avgPitchBend, MIDI_CHANNEL);
    colorWipe(0, 14, 0);
    lastPitchBend = avgPitchBend;
  }
  
  avgModulation = constrain(avgModulation, 0, 127);
  if (abs(avgModulation - lastModulation) > MOD_SEND_THRESHOLD) {
    MIDI.sendControlChange(1, avgModulation, MIDI_CHANNEL);
    colorWipe(8, 2, 16);
    lastModulation = avgModulation;
  }
}

void colorWipe(int r, int g, int b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
    if(r == 0 && g == 0 && b == 0)LED_flg = 0;
    else LED_flg = 1;
}
