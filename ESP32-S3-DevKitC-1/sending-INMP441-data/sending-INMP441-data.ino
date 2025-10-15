// ESP32-S3 firmware (Arduino core)
#include <Arduino.h>
#include "driver/i2s.h"

#define BAUD 921600
#define SR 16000
#define I2S_PORT I2S_NUM_0
#define PIN_BCLK 5
#define PIN_LRCLK 6
#define PIN_DIN 4
#define CHUNK 256

static const char HDR_MAGIC[] = "YAWAV1M";
static const char PKT_MAGIC[] = "KPAY";

uint32_t seqL=0, seqR=0;

void setup() {
  Serial.begin(BAUD);
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
  i2s_pin_config_t pins = {
    .bck_io_num = PIN_BCLK,
    .ws_io_num = PIN_LRCLK,
    .data_out_num = -1,
    .data_in_num = PIN_DIN
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);

  Serial.write((const uint8_t*)HDR_MAGIC, 7);
  uint32_t sr_le = SR;
  uint8_t hdr[5];
  memcpy(hdr, &sr_le, 4);
  hdr[4] = 2;
  Serial.write(hdr, 5);
}

static inline int16_t conv32to16(int32_t v){
  return (int16_t)(v >> 14);
}

void loop() {
  static int32_t ibuf[CHUNK*2];
  size_t nread=0;
  if (i2s_read(I2S_PORT, (void*)ibuf, sizeof(ibuf), &nread, portMAX_DELAY) != ESP_OK) return;
  int nframes = nread / (int)sizeof(int32_t) / 2;
  if (nframes <= 0) return;

  static int16_t L[CHUNK], R[CHUNK];
  for (int i=0;i<nframes;i++){
    int32_t l = ibuf[2*i+0];
    int32_t r = ibuf[2*i+1];
    L[i] = conv32to16(l);
    R[i] = conv32to16(r);
  }
  uint16_t plen = (uint16_t)(nframes * sizeof(int16_t));
  uint8_t h[11];

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 0;
  h[5] = (uint8_t)(plen & 0xFF);
  h[6] = (uint8_t)(plen >> 8);
  memcpy(&h[7], &seqL, 4);
  Serial.write(h, 11);
  Serial.write((uint8_t*)L, plen);
  seqL++;

  memcpy(h, PKT_MAGIC, 4);
  h[4] = 1;
  h[5] = (uint8_t)(plen & 0xFF);
  h[6] = (uint8_t)(plen >> 8);
  memcpy(&h[7], &seqR, 4);
  Serial.write(h, 11);
  Serial.write((uint8_t*)R, plen);
  seqR++;
}
