/*
 *  RedArc TVMS Prime Tank Sensor via BLE using ESP32 + Mopeka Pro (per tank) + SN65HVD230
 *  (C) Justin M Dunlop 2026
 * ------------------------------------------------------------------
 * Listens for the real TVMS's own tank broadcast (0x1BFD0224) and
 * immediately re-transmits an overriding frame with our own values —
 * no separate device identity, no channel-table config, minimal
 * traffic.
 *
 * WIRING
 * ------
 * ESP32 to SN65HVD230
 *    GPIO19 -> CANTX (CTX)
 *    GPIO22 -> SCLCANRX (CRX)
 *    3.3V   -> 3V3
 *    GND.   -> GND  
 *
 * SN65HVD230 to R-BUS
 *    GND**  -> Pin 3    (** Can use ESP32 second GND pin -> R-BUS Pin 3)
 *    CAN L  -> Pin 4
 *    CAN H  -> Pin 5
 *
 * Configuration
 *
 * Each tank is independently "linked" to a Mopeka sensor by MAC
 * address in TANKS[] below. A tank with a MAC configured gets its
 * value from live BLE readings. A tank left with mac="" has no
 * sensor and just holds at its fallback_percent value.
 *
 * Run mopeka_mac_scanner.ino first to find your sensors' MAC
 * addresses.
 *
 * Architecture notes (only matters if you're modifying this):
 *   - CAN reaction runs in its own dedicated FreeRTOS task (canTask),
 *     created BEFORE BLE is touched, so it can never be starved or
 *     blocked by BLE's own background processing.
 *   - BLE scanning is periodic (not continuous) — tank levels don't
 *     change fast, so there's no need for the radio to be active
 *     between polls. Cuts power draw and CPU/radio contention.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// =================================================================
// EDIT YOUR TANKS HERE
// =================================================================
struct TankConfig {
  const char* mac;             // lowercase MAC from mopeka_mac_scanner.ino, or "" for no sensor
  float empty_mm;              // Mopeka-measured height at empty
  float full_mm;               // Mopeka-measured height at full
  uint8_t fallback_percent;    // used only while mac == "" (no sensor configured)
};

static TankConfig TANKS[6] = {
  /* Tank 1 */ { "", 0.0, 290.0, 0 },
  /* Tank 2 */ { "", 0.0, 290.0, 0 },
  /* Tank 3 */ { "", 0.0, 290.0, 0 },
  /* Tank 4 */ { "", 0.0, 290.0, 0 },
  /* Tank 5 */ { "", 0.0, 290.0, 0 },
  /* Tank 6 */ { "", 0.0, 290.0, 0 },
};

#define TANK_POLL_INTERVAL_MS 15000   // how often to wake BLE and poll
#define BLE_SCAN_DURATION_SEC 5       // how long each poll scans for

// Set true to restore full per-frame CAN logging and a 5s health
// heartbeat — useful if you're debugging, noisy for normal use.
#define DEBUG_VERBOSE false
// =================================================================

#define CAN_TX GPIO_NUM_22
#define CAN_RX GPIO_NUM_19
#define TANK_CAN_ID 0x1BFD0224
#define MOPEKA_MANUFACTURER_ID 0x0059

// Live/fallback tank values, 0-100. Populated from TANKS[].fallback_percent
// at boot; overwritten with real readings as they arrive for any tank
// with a MAC configured.
uint8_t tankPercent[6];

// ---------------------------------------------------------------
// R-BUS reactive override
// ---------------------------------------------------------------
void sendTankFrame(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,
                    uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  twai_message_t message = {};
  message.identifier = TANK_CAN_ID;
  message.extd = 1;
  message.rtr = 0;
  message.data_length_code = 8;
  message.data[0] = b0; message.data[1] = b1; message.data[2] = b2; message.data[3] = b3;
  message.data[4] = b4; message.data[5] = b5; message.data[6] = b6; message.data[7] = b7;

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(50));
  if (result != ESP_OK) {
    Serial.print("TX FAILED: ");
    Serial.println(esp_err_to_name(result));
  }
}

void sendTank14() { sendTankFrame(0x14, 0xFF, 0xFF, tankPercent[0], 0x00, tankPercent[1], 0x00, 0xFF); }
void sendTank17() { sendTankFrame(0x17, tankPercent[2], 0x00, tankPercent[3], 0x00, tankPercent[4], 0x00, 0xFF); }
void sendTank1A() { sendTankFrame(0x1A, tankPercent[5], 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF); }

// ---------------------------------------------------------------
// Mopeka BLE decoding
// ---------------------------------------------------------------
BLEScan* pBLEScan;

float mopekaRawToHeight(uint16_t rawLevel, uint8_t rawTemperature)
{
  double rawT = rawTemperature;
  double height = rawLevel * (0.573045 - (0.002822 * rawT) - (0.00000535 * rawT * rawT));
  return (float)height;
}

float heightToPercentage(float heightMM, float emptyMM, float fullMM)
{
  float percentage = ((heightMM - emptyMM) / (fullMM - emptyMM)) * 100.0;
  if (percentage < 0.0) percentage = 0.0;
  if (percentage > 100.0) percentage = 100.0;
  return percentage;
}

int findTankForMac(const String& address)
{
  for (int i = 0; i < 6; i++) {
    if (strlen(TANKS[i].mac) > 0 && address.equals(TANKS[i].mac)) {
      return i;
    }
  }
  return -1;
}

class MopekaCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    String address = advertisedDevice.getAddress().toString().c_str();
    address.toLowerCase();

    int tankIndex = findTankForMac(address);
    if (tankIndex < 0) return; // not one of our configured tanks

    String manufacturerData = advertisedDevice.getManufacturerData();
    if (manufacturerData.length() < 10) return;

    uint8_t data[12] = {0};
    size_t n = min((size_t)12, manufacturerData.length());
    for (size_t i = 0; i < n; i++) data[i] = (uint8_t)manufacturerData[i];

    uint16_t manufacturerID = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    if (manufacturerID != MOPEKA_MANUFACTURER_ID) return;

    uint8_t rawTemperature = data[4] & 0x7F;
    uint16_t rawLevel = ((uint16_t)data[5] | ((uint16_t)data[6] << 8)) & 0x3FFF;

    float heightMM = mopekaRawToHeight(rawLevel, rawTemperature);
    float percentage = heightToPercentage(heightMM, TANKS[tankIndex].empty_mm, TANKS[tankIndex].full_mm);

    tankPercent[tankIndex] = (uint8_t)(percentage + 0.5);

    Serial.printf("Tank %d: %.1f%% (height=%.1fmm, raw=%d)\n",
      tankIndex + 1, percentage, heightMM, rawLevel);
  }
};

// ---------------------------------------------------------------
// CAN reactive task — dedicated, high priority, created before BLE
// is ever touched.
// ---------------------------------------------------------------
void canTask(void* pvParameters)
{
#if DEBUG_VERBOSE
  Serial.println(">>> canTask started <<<");
  uint32_t lastHeartbeat = 0;
#endif

  for (;;) {
#if DEBUG_VERBOSE
    uint32_t now = millis();
    if (now - lastHeartbeat >= 5000) {
      lastHeartbeat = now;
      twai_status_info_t status;
      if (twai_get_status_info(&status) == ESP_OK) {
        Serial.printf("[canTask alive] tx_err=%d rx_err=%d\n",
          status.tx_error_counter, status.rx_error_counter);
      }
    }
#endif

    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) != ESP_OK)
      continue;
    if (message.identifier != TANK_CAN_ID)
      continue;

    switch (message.data[0])
    {
      case 0x14: sendTank14(); break;
      case 0x17: sendTank17(); break;
      case 0x1A: sendTank1A(); break;
      default: continue;
    }

#if DEBUG_VERBOSE
    Serial.printf("Overrode page 0x%02X with [%d,%d,%d,%d,%d,%d]\n",
      message.data[0], tankPercent[0], tankPercent[1], tankPercent[2],
      tankPercent[3], tankPercent[4], tankPercent[5]);
#endif
  }
}

// ---------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 6; i++) tankPercent[i] = TANKS[i].fallback_percent;

  Serial.println();
  Serial.println("======================================");
  Serial.println(" RedARC R-Bus Reactive Tank TX + Mopeka");
  Serial.println("======================================");

  // --- CAN ---
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("CAN driver install FAILED");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("CAN start FAILED");
    return;
  }
  Serial.println("CAN interface started.");

  // CAN task first — must exist and be running before BLE is touched.
  xTaskCreatePinnedToCore(canTask, "CANReact", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
  delay(100);

  // --- BLE ---
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MopekaCallbacks(), true);
  pBLEScan->setActiveScan(false); // passive — Mopeka's data is in the advertisement itself
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(80);

  Serial.println();
  for (int i = 0; i < 6; i++) {
    if (strlen(TANKS[i].mac) > 0) {
      Serial.printf("Tank %d: live from %s (empty=%.0fmm full=%.0fmm)\n",
        i + 1, TANKS[i].mac, TANKS[i].empty_mm, TANKS[i].full_mm);
    } else {
      Serial.printf("Tank %d: no sensor, fixed at %d%%\n", i + 1, TANKS[i].fallback_percent);
    }
  }
  Serial.println();
}

void loop()
{
  pBLEScan->start(BLE_SCAN_DURATION_SEC, false);
  pBLEScan->clearResults();
  vTaskDelay(pdMS_TO_TICKS(TANK_POLL_INTERVAL_MS));
}
