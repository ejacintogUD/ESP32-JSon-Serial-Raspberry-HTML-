#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// FreeRTOS example: read TMP102 in one task, send JSON over Serial to Raspberry in another

const uint8_t SDA_PIN = 21;
const uint8_t SCL_PIN = 22;
const uint8_t TMP102_ADDR = 0x48;

TwoWire I2C_MASTER = TwoWire(0);

struct TempMsg {
  uint32_t timestamp; // millis()
  float tempC;
};

static QueueHandle_t tempQueue = NULL;

bool readTmp102(TwoWire &i2c, uint8_t addr, float &outC) {
  uint8_t data[2] = {0, 0};
  i2c.beginTransmission(addr);
  i2c.write(0x00);
  if (i2c.endTransmission(false) != 0) return false;
  if (i2c.requestFrom((int)addr, 2) != 2) return false;
  data[0] = i2c.read();
  data[1] = i2c.read();
  int16_t raw = ((int16_t)data[0] << 8) | data[1];
  raw >>= 4;
  if (raw & 0x800) raw |= 0xF000;
  outC = raw * 0.0625f;
  return true;
}

// Task: read TMP102 every 1s and send to queue
void TMPTask(void *pv) {
  (void)pv;
  for (;;) {
    float t = 0.0f;
    bool ok = readTmp102(I2C_MASTER, TMP102_ADDR, t);
    TempMsg msg;
    msg.timestamp = millis();
    msg.tempC = ok ? t : NAN;
    // if queue full, overwrite the oldest (send with 0 timeout)
    if (xQueueSend(tempQueue, &msg, 0) != pdTRUE) {
      // try to remove one and resend
      TempMsg drop;
      xQueueReceive(tempQueue, &drop, 0);
      xQueueSend(tempQueue, &msg, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Task: take from queue and send JSON via Serial
void SerialTask(void *pv) {
  (void)pv;
  TempMsg msg;
  for (;;) {
    if (xQueueReceive(tempQueue, &msg, portMAX_DELAY) == pdTRUE) {
      // Build a small JSON manually: {"ts":123456,"temp":23.5625}
      char buf[64];
      if (isnan(msg.tempC)) {
        snprintf(buf, sizeof(buf), "{\"ts\":%lu,\"temp\":null}\n", (unsigned long)msg.timestamp);
      } else {
        // 4 decimal places
        snprintf(buf, sizeof(buf), "{\"ts\":%lu,\"temp\":%.4f}\n", (unsigned long)msg.timestamp, msg.tempC);
      }
      Serial.print(buf);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("TMP102 FreeRTOS JSON example starting...");
  I2C_MASTER.begin(SDA_PIN, SCL_PIN, 100000);

  tempQueue = xQueueCreate(4, sizeof(TempMsg));
  if (!tempQueue) {
    Serial.println("Failed to create queue");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Create tasks
  BaseType_t r1 = xTaskCreatePinnedToCore(TMPTask, "TMPTask", 2048, NULL, 1, NULL, 1);
  BaseType_t r2 = xTaskCreatePinnedToCore(SerialTask, "SerialTask", 4096, NULL, 1, NULL, 1);
  if (r1 != pdPASS || r2 != pdPASS) {
    Serial.println("Failed to create tasks");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop() {
  // Nothing here; tasks run
  vTaskDelay(pdMS_TO_TICKS(1000));
}
