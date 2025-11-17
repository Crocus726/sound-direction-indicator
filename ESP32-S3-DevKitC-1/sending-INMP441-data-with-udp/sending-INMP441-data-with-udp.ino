#include <Arduino.h>
#include "driver/i2s.h" // INMP441↔ESP32 통신에 I2S 프로토콜을 사용하기 위한 라이브러리
#include <Adafruit_NeoPixel.h> // 보드 내장 LED 제어 라이브러리 포함

/* Wi-Fi 통신에 필요한 라이브러리 포함 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define SR 16000 // 소리의 샘플링 레이트
#define I2S_PORT I2S_NUM_0
#define INMP441_SD_PIN 4 // INMP441의 SD가 연결된 핀 번호
#define INMP441_SCK_PIN 5 // INMP441의 SCK가 연결된 핀 번호
#define INMP441_WS_PIN 6 // INMP441의 WS가 연결된 핀 번호
#define CHUNK 256

#define LED_PIN 38
#define LED_COUNT 1
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

static const char PKT_MAGIC[] = "KPAY"; // 각 데이터 패킷의 시작 헤더
uint32_t seqL=0, seqR=0; // 패킷 순서 추적 카운터 설정

/* Wi-Fi 설정(ESP32가 접속할 Wi-Fi AP의 SSID, PASSWD) */
const char* ssid = "revegebox";
const char* password = "revegebox1";

/* Wi-Fi 설정(ESP32가 UDP 통신으로 데이터를 보낼 IP, PORT) */
const char* host = "10.136.164.41"; // 핫스팟으로부터 라즈베리 파이가 할당받은 IP
const uint16_t port = 12345;

/* Wi-Fi 설정(IP 수동 할당에 필요한 정보) */
IPAddress local_IP(10, 136, 164, 221);
IPAddress gateway(10, 136, 164, 85);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP udp; // UDP 통신 객체 설정
bool wasConnected = false; // Wi-Fi 연결 상태 저장 변수

void setup() {
  Serial.begin(9600); // 시리얼 통신을 시작함(디버깅용)

  pixels.begin(); // NeoPixel 라이브러리 초기화
  pixels.setBrightness(255); // 밝기 설정 (0-255)
  pixels.setPixelColor(0, pixels.Color(128, 0, 128)); pixels.show(); // 보라색 점등
  delay(500);

  /* Wi-Fi 통신 설정 */
  if (!WiFi.config(local_IP, gateway, subnet)) Serial.println("STA Failed to configure");
  WiFi.mode(WIFI_STA); // EPS32를 클라이언트로 설정

  WiFi.begin(ssid, password); // ESP32를 Wi-Fi AP에 접속

  /* 시리얼 통신으로 Wi-Fi 연결 상태 확인 */
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); pixels.show(); // 파란색 점등
    delay(250);
    pixels.clear(); pixels.show(); // LED 끄기
    delay(250);
    Serial.print(".");
  }

  Serial.println(" Connected!");
  Serial.print("IP address: "); Serial.println(WiFi.localIP()); // 할당받은 IP 주소 확인  
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); pixels.show(); // 초록색 점등
  wasConnected = true; // Wi-Fi 연결 상태 최신화

  /* I2S 통신 설정 */
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SR,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  #if ESP_IDF_VERSION_MAJOR >= 5
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
  #else
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  #endif
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = CHUNK,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  /* I2S 핀 설정 */
  i2s_pin_config_t pins = {
    .bck_io_num = INMP441_SCK_PIN,
    .ws_io_num = INMP441_WS_PIN,
    .data_out_num = -1,
    .data_in_num = INMP441_SD_PIN
  };

  /* I2S 드라이버를 설치하고 핀을 연결함 */
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

static inline int16_t conv32to16(int32_t v) {
  return (int16_t)(v >> 14);
}

void loop() {
  /* Wi-Fi 연결 상태 확인 후, Wi-Fi 연결이 끊어져 있으면 */
  if (WiFi.status() != WL_CONNECTED) {
    if (wasConnected) { // 이전에 Wi-Fi 연결이 되어 있었을 경우
      Serial.println("Wi-Fi connection lost!");
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.show(); // 빨간색 점등
      wasConnected = false; // Wi-Fi 연결 상태 최신화
    }
    delay(1000); // 1초 대기 후 재연결 시도 (자동 재연결은 WiFi 라이브러리가 처리)
    return; // 연결될 때까지 오디오 처리 중단
  }

  /* Wi-Fi 연결 끊어졌다가 다시 연결된 경우 */
  if (!wasConnected) { // 위 if문에서 wasConneted가 false로 저장됨
    Serial.println("Wi-Fi reconnected!");
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // 초록색 점등
    pixels.show();
    wasConnected = true;
  }

  /* I2S 버퍼에서 오디오 데이터를 읽음(INMP441) */
  static int32_t ibuf[CHUNK*2];
  size_t nread=0;
  if (i2s_read(I2S_PORT, (void*)ibuf, sizeof(ibuf), &nread, portMAX_DELAY) != ESP_OK) return;
  /* 읽어온 데이터의 바이트 수가 nread 변수에 저장됨 */

  /* 32비트 데이터를 16비트로 변환하고 왼쪽 오른쪽 채널을 분리함 */
  int nframes = nread / (int)sizeof(int32_t) / 2; // 프레임 수 계산
  if (nframes <= 0) return;
  static int16_t L[CHUNK], R[CHUNK];
  for (int i = 0; i < nframes; i++){
    int32_t l = ibuf[2 * i + 0]; // 짝수 인덱스 = 왼쪽 채널
    int32_t r = ibuf[2 * i + 1]; // 홀수 인덱스 = 오른쪽 채널
    L[i] = conv32to16(l); // 32비트를 16비트로 변환
    R[i] = conv32to16(r); // 32비트를 16비트로 변환
  }

  uint16_t plen = (uint16_t)(nframes * sizeof(int16_t));
  uint8_t h[11];

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 0; // 채널 ID 0 (라즈베리 파이에서 sid로 저장)
  memcpy(&h[5], &plen, 2);
  memcpy(&h[7], &seqL, 4);
  udp.beginPacket(host, port);
  udp.write(h, 11);
  udp.write((uint8_t*)L, plen);
  udp.endPacket();
  seqL++;

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 1; // 채널 ID 1 (라즈베리 파이에서 sid로 저장)
  memcpy(&h[5], &plen, 2);
  memcpy(&h[7], &seqR, 4);
  udp.beginPacket(host, port);
  udp.write(h, 11);
  udp.write((uint8_t*)R, plen);
  udp.endPacket();
  seqR++;
}

