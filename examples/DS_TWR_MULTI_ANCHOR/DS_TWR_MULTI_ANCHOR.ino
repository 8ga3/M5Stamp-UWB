/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include <Arduino.h>
#include <M5Stamp_UWB.h>

// -----------------------------------------------------------------------------
// Hardware wiring.
// -----------------------------------------------------------------------------
// Default pin mapping for the Stamp C5 host. Update these pins when using
// different hosts or carrier boards.
static constexpr int UWB_PIN_GP7    = 23;
static constexpr int UWB_PIN_IRQ    = 0;
static constexpr int UWB_PIN_WAKEUP = 24;
static constexpr int UWB_PIN_RST    = 25;
static constexpr int UWB_PIN_MISO   = 26;  // DW_CDO -> host MISO.
static constexpr int UWB_PIN_MOSI   = 27;  // DW_CDI -> host MOSI.
static constexpr int UWB_PIN_CS     = 11;
static constexpr int UWB_PIN_SCK    = 12;

// -----------------------------------------------------------------------------
// UWB network and ranging parameters
// -----------------------------------------------------------------------------
// PAN_ID separates this example UWB network from other 802.15.4/UWB traffic.
static constexpr uint16_t PAN_ID = 0xDECA;

// ANCHOR_ID must be unique, so flash this sketch once per anchor with a different
// value. The tag's ANCHOR_SHORT_ADDR[] list must contain 0x0100 + ANCHOR_ID.
static constexpr uint8_t ANCHOR_ID = 1;

// Short addresses used inside the example frames. 0x01xx is reserved for anchors,
// so tag and anchor addresses never collide.
static constexpr uint16_t TAG_SHORT_ADDR    = 0x0001;
static constexpr uint16_t ANCHOR_SHORT_ADDR = 0x0100 + ANCHOR_ID;

// Common RX/host timeout. The UWB timeout is in UWB microseconds (uus), while
// hostTimeoutMs limits how long the MCU polls the driver status registers.
static constexpr uint32_t RX_TIMEOUT_UUS        = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS = 100;

// DS response timing. The TAG opens RX after transmitting Poll, and the
// ANCHOR schedules its delayed Response at RESPONSE_TX_DLY_UUS.
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS          = 3000;

// DS-only timing. After receiving Response, TAG schedules Final. The ANCHOR then
// waits for Final and sends a Result frame that carries the computed distance.
static constexpr uint32_t FINAL_TX_DLY_UUS                   = 1800;
static constexpr uint32_t FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS = 500;
static constexpr uint32_t RESULT_RX_AFTER_FINAL_TX_DLY_UUS   = 500;

// ANCHOR sends the Result frame once. Every repeat keeps this anchor transmitting
// while the tag is already polling the next one.
static constexpr uint8_t RESULT_REPEAT_COUNT   = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS = 3;

M5Stamp_UWB uwb;
static bool uwbReady = false;
// ANCHOR prints one statistics line every STATS_INTERVAL_MS instead of logging
// each event. Serial output blocks the RX hot path, and every anchor also hears
// the 4 * (N - 1) frames per cycle that belong to the other anchors' exchanges.
static constexpr uint32_t STATS_INTERVAL_MS = 5000;
// ok = exchanges served, ignored = frames for another anchor, fail = real errors.
static uint32_t okCount       = 0;
static uint32_t ignoredCount  = 0;
static uint32_t failCount     = 0;
static int32_t lastDistanceMm = 0;
static const char* lastFail   = "NONE";
static uint32_t lastStatsMs   = 0;

static M5Stamp_UWBDSRangeConfig makeDSRangeConfig()
{
    M5Stamp_UWBDSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus      = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus             = RESPONSE_TX_DLY_UUS;
    range.finalTxDelayUus                = FINAL_TX_DLY_UUS;
    range.finalRxAfterResponseTxDelayUus = FINAL_RX_AFTER_RESPONSE_TX_DLY_UUS;
    range.resultRxAfterFinalTxDelayUus   = RESULT_RX_AFTER_FINAL_TX_DLY_UUS;
    range.rxTimeoutUus                   = RX_TIMEOUT_UUS;
    range.hostTimeoutMs                  = RANGE_HOST_TIMEOUT_MS;
    range.resultRepeatCount              = RESULT_REPEAT_COUNT;
    range.resultRepeatGapMs              = RESULT_REPEAT_GAP_MS;
    return range;
}

static bool initUwb()
{
    // M5Stamp_UWBConfig is filled from the wiring table above.
    M5Stamp_UWBConfig config;
    config.pin_cs     = UWB_PIN_CS;
    config.pin_rst    = UWB_PIN_RST;
    config.pin_irq    = UWB_PIN_IRQ;
    config.pin_wakeup = UWB_PIN_WAKEUP;
    config.pin_gp7    = UWB_PIN_GP7;
    config.pin_sck    = UWB_PIN_SCK;
    config.pin_miso   = UWB_PIN_MISO;
    config.pin_mosi   = UWB_PIN_MOSI;
    // Default PHY uses channel 9 with the recommended settings from the library.
    M5Stamp_UWBPHYConfig phy;

    if (!uwb.begin(config, phy)) {
        Serial.printf("UWB_BEGIN,result=FAIL,error=%s\n", uwb.lastErrorName());
        Serial.printf("UWB_RAW_ID,dev_id=0x%08lX\n", static_cast<unsigned long>(uwb.readRawDeviceId()));
        return false;
    }

    const uint32_t devId = uwb.deviceId();
    Serial.printf("UWB_ID,dev_id=0x%08lX,chip=%s\n", static_cast<unsigned long>(devId), uwb.chipName());
    if (devId != M5STAMP_UWB_QM33120_DEVICE_ID) {
        Serial.printf("UWB_ID,result=FAIL,expected=0xDECA0314\n");
        return false;
    }

    Serial.printf("UWB_CONFIG,result=OK,ch=%u,plen=%u,rate=6M8,tx_power=0x%08lX\n", static_cast<unsigned>(phy.channel),
                  static_cast<unsigned>(phy.preambleLength), static_cast<unsigned long>(phy.txPower));
    return true;
}

static void runAnchorRole()
{
    const M5Stamp_UWBDSResponderResult result = uwb.respondDSRange(makeDSRangeConfig());

    if (result.success) {
        okCount++;
        lastDistanceMm = result.distanceMm;
    } else if (result.error == M5Stamp_UWBError::RangeFrameMismatch) {
        // Frame from another anchor's exchange. Normal here, so only count it.
        ignoredCount++;
    } else if (result.error != M5Stamp_UWBError::RxTimeout) {
        failCount++;
        lastFail = uwb.lastErrorName();
    }

    if ((millis() - lastStatsMs) < STATS_INTERVAL_MS) {
        return;
    }
    lastStatsMs = millis();

    Serial.printf("MULTI_RESP_STAT,anchor_id=%u,address=0x%04X,ok=%lu,ignored=%lu,fail=%lu,distance_mm=%ld,last=%s\n",
                  static_cast<unsigned>(ANCHOR_ID), ANCHOR_SHORT_ADDR, static_cast<unsigned long>(okCount),
                  static_cast<unsigned long>(ignoredCount), static_cast<unsigned long>(failCount),
                  static_cast<long>(lastDistanceMm), lastFail);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.printf("M5Stamp UWB DS-TWR MULTI-ANCHOR ANCHOR\n");
    Serial.printf("ROLE,mode=ANCHOR,anchor_id=%u,address=0x%04X\n", static_cast<unsigned>(ANCHOR_ID),
                  ANCHOR_SHORT_ADDR);
    Serial.printf("TWR_MODE,mode=DS-TWR\n");
    uwbReady    = initUwb();
    lastStatsMs = millis();
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
}

void loop()
{
    if (!uwbReady) {
        delay(1000);
        return;
    }
    runAnchorRole();
}
