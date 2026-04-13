#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// MAC address of the UGV bridge - Replace with actual UGV MAC
// Default placeholder if unknown
static uint8_t UGV_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static const uint8_t SOF = 0xAA;

// Mailboxes
static QueueHandle_t qSerialToRadio = nullptr;
static SemaphoreHandle_t serialMutex = nullptr;

typedef struct {
  uint8_t type;
  uint8_t len;
  uint8_t payload[250];
} Packet;

// Checksum
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t c = type ^ len;
  for (uint8_t i = 0; i < len; i++) c ^= payload[i];
  return c;
}

// Write to Serial safely
static void serial_send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t chk = checksum_xor(type, len, payload);
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.write(SOF);
    Serial.write(type);
    Serial.write(len);
    if (len) Serial.write(payload, len);
    Serial.write(chk);
    xSemaphoreGive(serialMutex);
  }
}

// Receive from radio
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 3) return; // Need type, len, and at least chk (chk is implied handled Python side, but we forward exactly what we got)
  // Re-emit immediately to Serial as framed packet
  // The ESP-NOW packet will just contain what was sent: type + len + payload + chk
  // We expect data[0] = type, data[1] = len, data[2..2+len-1] = payload, data[2+len] = chk
  if (data[1] <= 250 && len == data[1] + 3) {
      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.write(SOF);
          for (int i=0; i<len; i++) {
              Serial.write(data[i]);
          }
          xSemaphoreGive(serialMutex);
      }
  }
}

enum ParseState { WAIT_SOF, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK };
typedef struct {
  ParseState st;
  uint8_t type;
  uint8_t len;
  uint8_t idx;
  uint8_t buf[250];
} SerialParser;

static void parser_init(SerialParser* p) { p->st = WAIT_SOF; }

static bool parser_step(SerialParser* p, uint8_t b, Packet* outPkt) {
  switch (p->st) {
    case WAIT_SOF: if (b == SOF) p->st = WAIT_TYPE; break;
    case WAIT_TYPE: p->type = b; p->st = WAIT_LEN; break;
    case WAIT_LEN:
      p->len = b;
      if (p->len > 250) { parser_init(p); break; }
      p->idx = 0;
      p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD;
      break;
    case WAIT_PAYLOAD:
      p->buf[p->idx++] = b;
      if (p->idx >= p->len) p->st = WAIT_CHK;
      break;
    case WAIT_CHK: {
      uint8_t expected = checksum_xor(p->type, p->len, p->buf);
      if (b == expected) {
        outPkt->type = p->type;
        outPkt->len = p->len;
        memcpy(outPkt->payload, p->buf, p->len);
        parser_init(p); return true;
      }
      parser_init(p); break;
    }
  }
  return false;
}

void serialRxTask(void* pv) {
  SerialParser parser;
  parser_init(&parser);
  Packet pkt;
  for (;;) {
    while (Serial.available() > 0) {
      uint8_t b = (uint8_t)Serial.read();
      if (parser_step(&parser, b, &pkt)) {
          xQueueSend(qSerialToRadio, &pkt, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void radioTxTask(void* pv) {
  Packet pkt;
  uint8_t espBuf[255]; 
  for (;;) {
    if (xQueueReceive(qSerialToRadio, &pkt, portMAX_DELAY) == pdTRUE) {
        espBuf[0] = pkt.type;
        espBuf[1] = pkt.len;
        memcpy(espBuf + 2, pkt.payload, pkt.len);
        espBuf[2 + pkt.len] = checksum_xor(pkt.type, pkt.len, pkt.payload);
        
        esp_now_send(UGV_MAC, espBuf, 3 + pkt.len);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while(true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, UGV_MAC, 6);
  peer.channel = 0; peer.encrypt = false;
  esp_now_add_peer(&peer);

  serialMutex = xSemaphoreCreateMutex();
  qSerialToRadio = xQueueCreate(20, sizeof(Packet));

  xTaskCreate(serialRxTask, "SerRx", 4096, nullptr, 2, nullptr);
  xTaskCreate(radioTxTask,  "RadTx", 4096, nullptr, 2, nullptr);

  Serial.println("====================================");
  Serial.print("UAV Bridge Ready! MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("====================================");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
