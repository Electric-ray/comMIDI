// 🎵 DOS_MIDI 안정판 
// - 메모리: 큐 길이 128(실패 시 64 폴백), SysEx 512B
// - 로그: 3초 간격, 모드(GS/MT32) 표시, 백오프 재연결

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <AppleMIDI.h>
#include <atomic>

// ===== 하드웨어/환경 설정 =====
#define RS232_RX_GPIO 44      // RS232 RX 입력핀
#define RS232_BAUD    38400   // SoftMPU 기본속도
#define RESET_BTN     0       // 리셋버튼 (없으면 무시)

// Wi-Fi AP 모드 설정
static const char* kSSID   = "DOS_MIDI";
static const char* kPASS   = "bluewater";
static const int   kWiFiCH = 6;

// AppleMIDI 인스턴스
APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE();

// RS232 포트
HardwareSerial RS232(1);

// ===== 전역 상태 (atomic) =====
std::atomic<bool>     g_connected{false};
std::atomic<uint32_t> g_drop{0}, g_sent{0}, g_recv{0};
uint32_t g_lastReconnectMs = 0;
uint32_t g_backoffMs       = 1500;
uint32_t g_lastLogMs       = 0;

// 모드 감지용
enum MidiMode { MODE_UNKNOWN, MODE_GS, MODE_MT32 };
MidiMode g_mode = MODE_UNKNOWN;  // 모드 값 자체는 자주 바뀌지 않아 atomic 불필요

// ===== MIDI 메시지 =====
// SysEx를 0xF0~0xF7까지 “원형 그대로” 보존한다.
struct MidiMsg {
  uint8_t  st, d1, d2, len, chan; // len: 일반메시지 0/1/2, SysEx 시 실제 길이
  uint8_t  data[512];             // SysEx 버퍼 (원본 보존)
};

QueueHandle_t g_midiQueue = nullptr;

// ===== 유틸: 상태바이트별 데이터 길이 =====
static inline uint8_t dataLen(uint8_t s) {
  if (s >= 0xF8) return 0;  // 실시간 메시지
  switch (s & 0xF0) {
    case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
    case 0xC0: case 0xD0: return 1;
  }
  switch (s) { case 0xF1: case 0xF3: return 1; case 0xF2: return 2; default: return 0; }
}

// ===== AppleMIDI 콜백 =====
static void onAppleMidiConnected(const appleMidi::ssrc_t& ssrc, const char* name) {
  g_connected.store(true);
  g_backoffMs = 1500;
  Serial.printf("✅ DOS_MIDI 연결됨: %s (ssrc=%lu)\n", name, (unsigned long)ssrc);
}
static void onAppleMidiDisconnected(const appleMidi::ssrc_t& ssrc) {
  g_connected.store(false);
  Serial.printf("❌ DOS_MIDI 세션 종료 (ssrc=%lu)\n", (unsigned long)ssrc);
}
static void restartAppleMIDI() {
  Serial.println("🔄 DOS_MIDI 세션 재시작...");
  AppleMIDI.begin(); // 3.2.0에서는 인자 없음
  AppleMIDI.setHandleConnected(onAppleMidiConnected);
  AppleMIDI.setHandleDisconnected(onAppleMidiDisconnected);
}

// ===== 송신 태스크 (큐 → AppleMIDI) =====
// SysEx는 경계 포함(true), 일반메시지는 len에 맞춰 전송
static void midiTxTask(void*) {
  MidiMsg m;
  for (;;) {
    if (xQueueReceive(g_midiQueue, &m, portMAX_DELAY) == pdTRUE) {
      if (!g_connected.load()) { g_drop++; continue; }

      if (m.st == 0xF0) {
        // SysEx 메시지 전송 (0xF0/0xF7 포함)
        MIDI.sendSysEx(m.len, m.data, true);
      } else {
        // 일반 MIDI 메시지 전송 (채널 보존)
        uint8_t ch = (m.chan >= 1 && m.chan <= 16) ? m.chan : 1;
        if      (m.len == 0) MIDI.send((midi::MidiType)m.st, 0,    0,    ch);
        else if (m.len == 1) MIDI.send((midi::MidiType)m.st, m.d1, 0,    ch);
        else                 MIDI.send((midi::MidiType)m.st, m.d1, m.d2, ch);
      }
      g_sent++;
    }
    vTaskDelay(1);  // CPU 양보 (타이밍 안정화)
  }
}

// ===== RS232 수신 → 큐 =====
// - SysEx: 0xF0에서 시작, 0xF7에서 종료, 원형 보존 후 큐로 투입
// - 러닝 스테이터스: rs/need/d1/ch로 단순·정확 처리
static void feedByte(uint8_t b) {
  static uint8_t rs = 0, need = 0, d1 = 0, ch = 1;
  static bool     inSysEx = false;
  static uint8_t  syxBuf[512];
  static uint16_t syxLen = 0;

  g_recv++;

  // SysEx 시작
  if (b == 0xF0) {
    inSysEx = true;
    syxLen  = 0;
    syxBuf[syxLen++] = b;  // 0xF0 포함
    return;
  }

  // SysEx 수집 중
  if (inSysEx) {
    if (syxLen < sizeof(syxBuf)) syxBuf[syxLen++] = b;
    if (b == 0xF7) {  // 종료
      inSysEx = false;

      // ── 모드 감지 (Roland 0x41 계열) ──
      if (syxLen >= 6) {
        if (syxBuf[1] == 0x41 && syxBuf[3] == 0x42 && g_mode != MODE_GS) {
          g_mode = MODE_GS;  Serial.println("🎛️ Roland GS / SC-55~88 모드 감지됨");
        } else if (syxBuf[1] == 0x41 && syxBuf[3] == 0x16 && g_mode != MODE_MT32) {
          g_mode = MODE_MT32; Serial.println("🎹 Roland LA / MT-32 모드 감지됨");
        }
      }

      // SysEx 그대로 큐에 밀어넣기
      MidiMsg m{};
      m.st  = 0xF0;
      m.len = (uint8_t)((syxLen <= sizeof(m.data)) ? syxLen : sizeof(m.data));
      memcpy(m.data, syxBuf, m.len);
      if (xQueueSend(g_midiQueue, &m, 0) != pdTRUE) g_drop++;
    }
    return;
  }

  // 실시간 메시지 (Running Status와 독립)
  if (b >= 0xF8) {
    MidiMsg m{b, 0, 0, 0, ch, {0}};
    if (xQueueSend(g_midiQueue, &m, 0) != pdTRUE) g_drop++;
    return;
  }

  // 상태바이트
  if (b & 0x80) {
    if (b == 0xF7) { inSysEx = false; return; }
    rs = b; need = dataLen(b); ch = (b & 0x0F) + 1;
    if (need == 0) {
      MidiMsg m{rs, 0, 0, 0, ch, {0}};
      if (xQueueSend(g_midiQueue, &m, 0) != pdTRUE) g_drop++;
    }
    return;
  }

  // 데이터 바이트 (러닝 스테이터스)
  if (!rs) return;
  if (need == 1) {
    uint8_t hi = rs & 0xF0;
    MidiMsg m{rs, d1, b, 2, ch, {0}};
    if (hi == 0xC0 || hi == 0xD0 || rs == 0xF1 || rs == 0xF3) { // 실제 1바이트 메시지들
      m.d1 = b; m.d2 = 0; m.len = 1;
    }
    if (xQueueSend(g_midiQueue, &m, 0) != pdTRUE) g_drop++;
    need = dataLen(rs);  // 러닝 스테이터스 유지
  } else if (need >= 2) {
    d1 = b; need = 1;
  }
}

// ===== 큐 생성(폴백 포함) =====
static QueueHandle_t createQueueWithFallback(size_t itemSize) {
  UBaseType_t len1 = 128;
  Serial.printf("🧮 큐 생성 시도: item=%uB, len=%u, 예상≈%luB\n",
                (unsigned)itemSize, (unsigned)len1,
                (unsigned long)itemSize * len1);
  QueueHandle_t q = xQueueCreate(len1, itemSize);
  if (q) return q;

  UBaseType_t len2 = 64;
  Serial.printf("↩️ 1차 실패 → len=%u 재시도\n", (unsigned)len2);
  q = xQueueCreate(len2, itemSize);
  return q;
}

// ===== 초기화 =====
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("🎵 DOS_MIDI 안정판 ");

  // 큐 생성 (128 → 실패 시 64)
  size_t itemSize = sizeof(MidiMsg);
  Serial.printf("ℹ️ MidiMsg 크기:%uB, FreeHeap:%luB\n",
                (unsigned)itemSize, (unsigned long)ESP.getFreeHeap());
  g_midiQueue = createQueueWithFallback(itemSize);
  if (!g_midiQueue) {
    Serial.println("❌ 큐 생성 실패! 재부팅");
    delay(700);
    ESP.restart();
  }
  Serial.printf("✅ 큐 생성 완료 (FreeHeap:%luB)\n", (unsigned long)ESP.getFreeHeap());

  // 송신 태스크 (APP CPU로)
  xTaskCreatePinnedToCore(midiTxTask, "midiTx", 6144, nullptr, 2, nullptr, APP_CPU_NUM);

  // RS232 설정
  RS232.begin(RS232_BAUD, SERIAL_8N1, RS232_RX_GPIO, -1);
  RS232.setRxBufferSize(8192);
  Serial.println("✅ RS232 준비됨");

  // Wi-Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool apOk = WiFi.softAP(kSSID, kPASS, kWiFiCH, 0, 4);
  if (!apOk) Serial.println("⚠️ Wi-Fi AP 시작 실패");
  delay(800);
  Serial.printf("📡 SSID:%s  IP:%s\n", kSSID, WiFi.softAPIP().toString().c_str());

  // mDNS
  if (MDNS.begin("dosmidi")) {
    MDNS.addService("apple-midi", "udp", 5004);
    Serial.println("✅ mDNS 서비스 활성화됨");
  } else {
    Serial.println("⚠️ mDNS 시작 실패");
  }

  restartAppleMIDI();
  Serial.printf("🚀 준비 완료 (FreeHeap:%luB)\n", (unsigned long)ESP.getFreeHeap());
}

// ===== 메인 루프 =====
void loop() {
  MIDI.read();                       // AppleMIDI 내부 이벤트 처리
  while (RS232.available() > 0) {    // RS232 → 파서
    feedByte((uint8_t)RS232.read());
  }

  // AppleMIDI 재연결 백오프
  if (!g_connected.load()) {
    uint32_t now = millis();
    if (now - g_lastReconnectMs > g_backoffMs) {
      g_lastReconnectMs = now;
      restartAppleMIDI();
      uint32_t next = g_backoffMs * 2;
      g_backoffMs = (next > 8000u) ? 8000u : next;  // 8초 상한
    }
  }

  // 상태 로그 (3초 간격)
  uint32_t now = millis();
  if (now - g_lastLogMs > 3000) {
    g_lastLogMs = now;
    UBaseType_t qfree = uxQueueSpacesAvailable(g_midiQueue);
    const char* modeStr = (g_mode == MODE_GS) ? "GS" : (g_mode == MODE_MT32 ? "MT32" : "UNK");
    Serial.printf("ℹ️ conn:%d recv:%lu sent:%lu drop:%lu qfree:%u mode:%s heap:%lu\n",
      g_connected.load(), g_recv.load(), g_sent.load(), g_drop.load(),
      (unsigned)qfree, modeStr, (unsigned long)ESP.getFreeHeap());
  }

  vTaskDelay(1);
}
