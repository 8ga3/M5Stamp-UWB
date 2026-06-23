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

// Short addresses used inside the example frames. Keep TAG and ANCHOR different.
static constexpr uint16_t TAG_SHORT_ADDR    = 0x0001;
static constexpr uint16_t ANCHOR_SHORT_ADDR = 0x0002;

// ANCHOR logs every ANCHOR_LOG_INTERVAL successful responses. This keeps the
// responder mostly silent so Serial output does not disturb the RX hot path.
static constexpr uint32_t ANCHOR_LOG_INTERVAL = 20;

// Common RX/host timeout. The UWB timeout is in UWB microseconds (uus), while
// hostTimeoutMs limits how long the MCU polls the driver status registers.
static constexpr uint32_t RX_TIMEOUT_UUS        = 3000;
static constexpr uint32_t RANGE_HOST_TIMEOUT_MS = 100;

// SS response timing. The TAG opens RX after transmitting Poll, and the
// ANCHOR schedules its delayed Response at RESPONSE_TX_DLY_UUS.
static constexpr uint32_t RESPONSE_RX_AFTER_TX_DLY_UUS = 1500;
static constexpr uint32_t RESPONSE_TX_DLY_UUS          = 3000;

M5Stamp_UWB uwb;
static bool uwbReady            = false;
static uint32_t lastRangeMs     = 0;
static uint32_t rangeCount      = 0;
static uint32_t rangeOkCount    = 0;
static uint32_t rangeFailCount  = 0;
static uint32_t anchorRespCount = 0;
static uint32_t anchorFailCount = 0;

static M5Stamp_UWBRangeConfig makeSSRangeConfig()
{
    M5Stamp_UWBRangeConfig range;
    range.panId                     = PAN_ID;
    range.initiatorAddress          = TAG_SHORT_ADDR;
    range.responderAddress          = ANCHOR_SHORT_ADDR;
    range.responseRxAfterTxDelayUus = RESPONSE_RX_AFTER_TX_DLY_UUS;
    range.responseTxDelayUus        = RESPONSE_TX_DLY_UUS;
    range.rxTimeoutUus              = RX_TIMEOUT_UUS;
    range.hostTimeoutMs             = RANGE_HOST_TIMEOUT_MS;
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
    // Default PHY uses channel 5 with the recommended settings from the library.
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
    const M5Stamp_UWBResponderResult result = uwb.respondRange(makeSSRangeConfig());
    const char* logPrefix                   = "SS_RESP_STAT";

    if (!result.success) {
        if (result.error == M5Stamp_UWBError::RxTimeout) {
            return;
        }

        anchorFailCount++;
        if ((anchorFailCount % ANCHOR_LOG_INTERVAL) == 0) {
            Serial.printf("%s,count=%lu,fail=%lu,last=FAIL,error=%s\n", logPrefix,
                          static_cast<unsigned long>(anchorRespCount), static_cast<unsigned long>(anchorFailCount),
                          uwb.lastErrorName());
        }
        return;
    }

    anchorRespCount++;
    if ((anchorRespCount % ANCHOR_LOG_INTERVAL) != 0) {
        return;
    }

    Serial.printf("%s,count=%lu,fail=%lu,last=OK,seq=%u,requester=0x%X,elapsed_ms=%lu\n", logPrefix,
                  static_cast<unsigned long>(anchorRespCount), static_cast<unsigned long>(anchorFailCount),
                  result.sequence, result.requester, static_cast<unsigned long>(result.elapsedMs));
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.printf("M5Stamp UWB SS-TWR ANCHOR\n");
    Serial.printf("ROLE,mode=ANCHOR\n");
    Serial.printf("TWR_MODE,mode=SS-TWR\n");
    uwbReady = initUwb();
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
