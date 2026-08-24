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

// Short address of this tag. 0x01xx is reserved for anchors, so tag and anchor
// addresses never collide.
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;

// Anchors polled in this order, one exchange each per cycle. DS_TWR_MULTI_ANCHOR
// derives its address as 0x0100 + ANCHOR_ID, so keep this list in sync with the
// anchors that are actually powered on.
static constexpr uint16_t ANCHOR_SHORT_ADDR[] = {0x0101, 0x0102};
static constexpr uint8_t ANCHOR_COUNT         = sizeof(ANCHOR_SHORT_ADDR) / sizeof(ANCHOR_SHORT_ADDR[0]);

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

// Result repeat settings belong to the responder. They are kept here so both
// sketches share one configuration block, and match DS_TWR_MULTI_ANCHOR.
static constexpr uint8_t RESULT_REPEAT_COUNT   = 1;
static constexpr uint32_t RESULT_REPEAT_GAP_MS = 3;

M5Stamp_UWB uwb;
static bool uwbReady = false;
// TAG starts one polling cycle every POLL_INTERVAL_MS and waits ANCHOR_GAP_MS
// between two anchors, which gives the previous anchor time to leave TX and
// return to RX before the next Poll goes out.
static constexpr uint32_t POLL_INTERVAL_MS = 200;
static constexpr uint32_t ANCHOR_GAP_MS    = 20;
static uint32_t lastPollMs                 = 0;
static uint32_t pollSequence               = 0;

static M5Stamp_UWBDSRangeConfig makeDSRangeConfig(uint16_t anchorAddress)
{
    M5Stamp_UWBDSRangeConfig range;
    range.panId                          = PAN_ID;
    range.initiatorAddress               = TAG_SHORT_ADDR;
    range.responderAddress               = anchorAddress;
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

static void logRange(uint16_t anchorAddress, const M5Stamp_UWBDSRangeResult& result)
{
    // The anchor id is the low byte of the short address, so the log lines stay
    // readable when ANCHOR_SHORT_ADDR[] is reordered.
    const unsigned anchorId = static_cast<unsigned>(anchorAddress & 0xFF);

    if (result.success) {
        Serial.printf("MULTI_RANGE,cycle=%lu,anchor_id=%u,address=0x%04X,result=OK,distance_mm=%ld,distance_m=%.3f\n",
                      static_cast<unsigned long>(pollSequence), anchorId, anchorAddress,
                      static_cast<long>(result.distanceMm), result.distanceM);
        return;
    }

    Serial.printf("MULTI_RANGE,cycle=%lu,anchor_id=%u,address=0x%04X,result=FAIL,error=%s\n",
                  static_cast<unsigned long>(pollSequence), anchorId, anchorAddress, uwb.lastErrorName());
}

static void runTagRole()
{
    if ((millis() - lastPollMs) < POLL_INTERVAL_MS) {
        return;
    }
    lastPollMs = millis();

    // One cycle ranges every anchor in turn. A missing anchor only costs the RX
    // frame-wait timeout, so the remaining anchors are still polled.
    pollSequence++;
    for (uint8_t index = 0; index < ANCHOR_COUNT; ++index) {
        const uint16_t anchorAddress = ANCHOR_SHORT_ADDR[index];
        logRange(anchorAddress, uwb.requestDSRange(makeDSRangeConfig(anchorAddress)));
        delay(ANCHOR_GAP_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.printf("M5Stamp UWB DS-TWR MULTI-ANCHOR TAG\n");
    Serial.printf("ROLE,mode=TAG,address=0x%04X,anchors=%u\n", TAG_SHORT_ADDR, static_cast<unsigned>(ANCHOR_COUNT));
    Serial.printf("TWR_MODE,mode=DS-TWR\n");
    for (uint8_t index = 0; index < ANCHOR_COUNT; ++index) {
        const uint16_t anchorAddress = ANCHOR_SHORT_ADDR[index];
        Serial.printf("ANCHOR_LIST,anchor_id=%u,address=0x%04X\n", static_cast<unsigned>(anchorAddress & 0xFF),
                      anchorAddress);
    }
    uwbReady = initUwb();
    Serial.printf("TEST_START,result=%s\n", uwbReady ? "OK" : "FAIL");
}

void loop()
{
    if (!uwbReady) {
        delay(1000);
        return;
    }
    runTagRole();
}
