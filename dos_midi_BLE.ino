/*
 * 🎵 MIDIOPL_BLE (Transport 콜백 고정 + 지터개선, GM Reset 제거)
 *  - BLE-MIDI 콜백은 MidiInterface가 아니라 Transport에 등록 (setHandleConnected/Disconnected)
 *  - 빠른 곡에서도 안정적 전송을 위해 버스트/슬립 자동 조정
 *  - 연결 해제 시 잔향 방지를 위해 All Sound Off / All Notes Off 등 즉시 전송
 *  - GM Reset(SysEx) 초기화 제거 (요청 반영)
 */

#include <Arduino.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32.h>

// ===== 사용자 설정 =====
#define DEVICE_NAME     "MIDIOPL_BLE"
#define RS232_BAUD      38400
#define RS232_RX_GPIO   44
#define DEBUG_BAUD      115200
#define DEBUG_LOG       0   // 1=디버그 (성능 저하 가능)

BLEMIDI_CREATE_INSTANCE(DEVICE_NAME, BLE_MIDI);
HardwareSerial RS232(1);

// 통계
static uint32_t tx_count = 0;
static uint32_t drop_count = 0;
static bool ble_connected = false;

// ---------- 유틸 ----------
static void sendAllNotesOff() {
  for (uint8_t ch = 1; ch <= 16; ++ch) {
    // 잔향/웅~ 톤 방지를 위해 안전하게 리셋
    BLE_MIDI.sendControlChange(120, 0, ch); // All Sound Off
    BLE_MIDI.sendControlChange(123, 0, ch); // All Notes Off
    BLE_MIDI.sendControlChange(64,  0, ch); // Hold Pedal 해제
    BLE_MIDI.sendControlChange(121, 0, ch); // Reset All Controllers
    BLE_MIDI.sendPitchBend(8192, ch);       // Pitch Bend 중앙 (0..16383 스케일)
    BLE_MIDI.sendAfterTouch(0, ch);         // 채널 애프터터치 0
  }
}

static void onConnected() {
  ble_connected = true;
#if DEBUG_LOG
  Serial.println(F("🔵 BLE Connected"));
#endif
  // 이전 세션 잔음 정리
  sendAllNotesOff();
}

static void onDisconnected() {
  ble_connected = false;
#if DEBUG_LOG
  Serial.println(F("🔴 BLE Disconnected"));
#endif
  // 끊길 때도 잔향 방지
  sendAllNotesOff();
}

// ========== MIDI 파서 (성공했던 구조 기반) ==========
struct MidiParser {
  uint8_t running = 0;
  uint8_t buf[2];
  uint8_t need = 0;
  uint8_t have = 0;
  bool    inSysEx = false;
  uint8_t sx[256];
  uint16_t sxLen = 0;

  static uint8_t dataLen(uint8_t status) {
    if (status < 0x80) return 0;
    if (status < 0xF0) {
      switch (status & 0xF0) {
        case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
        case 0xC0: case 0xD0: return 1;
      }
      return 0;
    }
    switch (status) {
      case 0xF1: return 1;
      case 0xF2: return 2;
      case 0xF3: return 1;
      case 0xF6: return 0;
      default:   return 0;
    }
  }

  void resetShort() { have = 0; need = 0; }

  void sendRealtime(uint8_t rt) {
    BLE_MIDI.sendRealTime((midi::MidiType)rt);
    tx_count++;
#if DEBUG_LOG
    Serial.printf("RT: %02X\n", rt);
#endif
  }

  void flushShort(uint8_t status) {
    uint8_t ch = (status & 0x0F) + 1;
    uint8_t hi = (status & 0xF0);

    switch (status) {
      case 0xF1:  BLE_MIDI.sendTimeCodeQuarterFrame(buf[0]); break;
      case 0xF2:  { uint16_t pos = (uint16_t)buf[0] | ((uint16_t)buf[1] << 7);
                    BLE_MIDI.sendSongPosition(pos); } break;
      case 0xF3:  BLE_MIDI.sendSongSelect(buf[0]); break;
      case 0xF6:  BLE_MIDI.sendTuneRequest(); break;

      default:
        switch (hi) {
          case 0x80: BLE_MIDI.sendNoteOff       (buf[0], buf[1], ch); break;
          case 0x90: if (buf[1] == 0) BLE_MIDI.sendNoteOff(buf[0], 0, ch);
                     else             BLE_MIDI.sendNoteOn (buf[0], buf[1], ch);
                     break;
          case 0xA0: BLE_MIDI.sendPolyPressure  (buf[0], buf[1], ch); break; // 경고만; 동작 OK
          case 0xB0: BLE_MIDI.sendControlChange (buf[0], buf[1], ch); break;
          case 0xC0: BLE_MIDI.sendProgramChange (buf[0], ch);         break;
          case 0xD0: BLE_MIDI.sendAfterTouch    (buf[0], ch);         break; // 채널 애프터터치
          case 0xE0: { int bend = (int)buf[0] | ((int)buf[1] << 7);
                       BLE_MIDI.sendPitchBend(bend, ch); }            break;
          default: break;
        }
        break;
    }

#if DEBUG_LOG
    Serial.printf("EV: %02X", status);
    if (need > 0) Serial.printf(" %02X", buf[0]);
    if (need > 1) Serial.printf(" %02X", buf[1]);
    Serial.println();
#endif

    tx_count++;
    resetShort();
  }

  void flushSysEx() {
    if (sxLen == 0) return;
    BLE_MIDI.sendSysEx(sxLen, sx, true);
#if DEBUG_LOG
    Serial.print("SYSEX: ");
    for (uint16_t i=0; i<sxLen; i++) Serial.printf("%02X ", sx[i]);
    Serial.println();
#endif
    tx_count++;
    sxLen = 0;
    inSysEx = false;
  }

  void feed(uint8_t b) {
    if (b & 0x80) {
      if (b >= 0xF8) { sendRealtime(b); return; }

      if (b == 0xF0) {
        inSysEx = true; sxLen = 0;
        sx[sxLen++] = 0xF0;
        running = 0; resetShort();
        return;
      }

      if (b == 0xF7) {
        if (inSysEx) {
          if (sxLen < sizeof(sx)) sx[sxLen++] = 0xF7;
          flushSysEx();
        }
        return;
      }

      running = b;
      need = dataLen(b);
      have = 0;
      return;
    }

    if (inSysEx) {
      if (sxLen < sizeof(sx)) sx[sxLen++] = b;
      if (sxLen >= 220) {               // 너무 길면 중간 플러시
        if (sx[sxLen-1] != 0xF7) sx[sxLen++] = 0xF7;
        flushSysEx();
      }
      return;
    }

    if (running == 0) return;

    if (have < 2) buf[have++] = b;
    if (have >= need) {
      flushShort(running);
    }
  }
};

static MidiParser parser;

// ---------- SETUP ----------
void setup() {
  Serial.begin(DEBUG_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("🎵 MIDIOPL_BLE (Transport 콜백 고정 + 지터개선, GM Reset 제거)"));

  // UART 버퍼 크게
  RS232.setRxBufferSize(2048);
  RS232.begin(RS232_BAUD, SERIAL_8N1, RS232_RX_GPIO, -1);
  Serial.printf("📡 RS232: GPIO%d, %d baud, buf=2048\n", RS232_RX_GPIO, RS232_BAUD);

  // BLE-MIDI 시작
  BLE_MIDI.begin();

  // ★★★ 콜백은 Transport에 등록! (인자 없는 함수 포인터)
  BLE_MIDI.getTransport()->setHandleConnected(onConnected);
  BLE_MIDI.getTransport()->setHandleDisconnected(onDisconnected);

  Serial.println(F("🔵 BLE-MIDI Ready (Windows/스마트폰에서 페어링 후 입력 포트로 사용)"));

#if DEBUG_LOG
  Serial.println(F("⚠️ DEBUG_LOG=1"));
#else
  Serial.println(F("✅ DEBUG_LOG=0 (최적)"));
#endif

  tx_count = 0;
  drop_count = 0;
}

// ---------- LOOP ----------
void loop() {
  int available = RS232.available();
  int processed = 0;

  // 빠른 곡 대비: 버퍼 쌓임에 따라 버스트 자동 증가
  int burstSize = 10;
  if (available > 200)       burstSize = 40;
  else if (available > 120)  burstSize = 28;
  else if (available > 60)   burstSize = 18;

  uint32_t tStart = micros();

  while (RS232.available() > 0 && processed < burstSize) {
    parser.feed((uint8_t)RS232.read());
    processed++;

    // BLE 스택에 숨 고르기: 4바이트마다 아주 짧게 양보
    if ((processed & 0x03) == 0) {
      delayMicroseconds(35); // 30~40 사이에서 안정적
    }
  }

  // 처리 시간 기반 자동 보정 슬립: 너무 바쁘면 적게, 여유면 조금 더
  uint32_t dur = micros() - tStart;
  if (processed > 0) {
    if (dur < 2000) {
      // 가변 슬립(부하가 적을수록 살짝 더 쉬게)
      int adj = 55 - (int)(dur / 50);     // 55us 기준
      if (adj < 20) adj = 20;
      delayMicroseconds(adj);
    } else {
      // 오래 걸렸으면 BLE에 더 양보
      yield();
    }
  } else {
    // 입력 없을 때는 쉬기
    delayMicroseconds(250);
  }

  // 통계 (10초마다)
  static uint32_t last = 0;
  if (millis() - last > 10000) {
    last = millis();
    Serial.printf("📊 TX=%lu, DROP=%lu, Buf=%d Conn=%d\n",
                  (unsigned long)tx_count, (unsigned long)drop_count,
                  available, ble_connected);
  }
}
