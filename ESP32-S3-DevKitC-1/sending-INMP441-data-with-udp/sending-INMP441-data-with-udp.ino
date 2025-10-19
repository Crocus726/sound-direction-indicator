#include <Arduino.h>
#include "driver/i2s.h" // INMP441↔ESP32 통신에 I2S 프로토콜을 사용하기 위한 라이브러리

/* Wi-Fi 통신에 필요한 라이브러리 포함 */
#include <WiFi.h>
#include <WiFiUdp.h>

#define SR 16000 // 소리의 샘플링 레이트
#define I2S_PORT I2S_NUM_0
#define PIN_BCLK 5 // I2S Bit Clock 핀 번호 설정
#define PIN_LRCLK 6 // I2S Left/Right Clock 핀 번호 설정
#define PIN_DIN 4 // I2S Data In 핀 번호 설정
#define CHUNK 256

static const char PKT_MAGIC[] = "KPAY"; // 각 데이터 패킷의 시작 헤더
uint32_t seqL=0, seqR=0; // 패킷 순서 추적 카운터 설정

/* Wi-Fi 통신 설정(SSID, PASSWD) */
const char* ssid = "revegebox";
const char* password = "revegebox1";

/* Wi-Fi 통신 설정(IP, PORT) */
const char* host = "10.168.163.41"; // 핫스팟으로부터 라즈베리 파이가 할당받은 IP
const uint16_t port = 115;

WiFiUDP udp; // UDP 통신 객체 설

void setup() {
  Serial.begin(9600); // 시리얼 통신을 시작함(디버깅용)

  /* Wi-Fi 통신 설정 */
  WiFi.mode(WIFI_STA); // EPS32를 클라이언트로 설정
  WiFi.begin(ssid, password); // ESP32를 설정한 Wi-Fi AP에 접속

  /* 시리얼 통신으로 Wi-Fi 연결 상태 확인 */
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // DHCP로 할당받은 IP 주소 확인

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
    .bck_io_num = PIN_BCLK,
    .ws_io_num = PIN_LRCLK,
    .data_out_num = -1,
    .data_in_num = PIN_DIN
  };

  /* I2S 드라이버를 설치하고 핀을 연결함 */
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

static inline int16_t conv32to16(int32_t v){
  return (int16_t)(v >> 14);
}

void loop() {
  /* I2S 버퍼에서 오디오 데이터를 읽음(INMP441) */
  static int32_t ibuf[CHUNK*2];
  size_t nread=0;
  if (i2s_read(I2S_PORT, (void*)ibuf, sizeof(ibuf), &nread, portMAX_DELAY) != ESP_OK) return;
  /* 읽어온 데이터의 바이트 수가 nread 변수에 저장됨 */

  /* 32비트 데이터를 16비트로 변환하고 왼쪽 오른쪽 채널을 분리함 */
  int nframes = nread / (int)sizeof(int32_t) / 2; // 프레임 수 계산
  if (nframes <= 0) return;
  static int16_t L[CHUNK], R[CHUNK];
  for (int i=0;i<nframes;i++){
    int32_t l = ibuf[2*i+0]; // 짝수 인덱스 = 왼쪽 채널
    int32_t r = ibuf[2*i+1]; // 홀수 인덱스 = 오른쪽 채널
    L[i] = conv32to16(l); // 32비트를 16비트로 변환
    R[i] = conv32to16(r); // 32비트를 16비트로 변환
  }

  uint16_t plen = (uint16_t)(nframes * sizeof(int16_t));
  uint8_t h[11];

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 0; // 채널 ID 0
  memcpy(&h[5], &plen, 2);
  memcpy(&h[7], &seqL, 4);
  udp.beginPacket(host, port);
  udp.write(h, 11);
  udp.write((uint8_t*)L, plen);
  udp.endPacket();
  seqL++;

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 1; // 채널 ID 1
  memcpy(&h[5], &plen, 2);
  memcpy(&h[7], &seqR, 4);
  udp.beginPacket(host, port);
  udp.write(h, 11);
  udp.write((uint8_t*)R, plen);
  udp.endPacket();
  seqR++;
}
