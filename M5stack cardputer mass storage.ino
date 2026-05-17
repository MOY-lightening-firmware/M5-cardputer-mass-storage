
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include "USB.h"
#include "USBMSC.h"
#include "tusb.h"

#define SD_CS_PIN   12
#define SD_SCK_PIN  40
#define SD_MOSI_PIN 14
#define SD_MISO_PIN 39
#define SECTOR_SIZE 512

USBMSC   MSC;
SPIClass sdSPI(HSPI);

static volatile bool mscWriteActive = false;
static bool     sdMounted     = false;
static bool     usbActive     = false;
static bool     needRedraw    = true;
static uint32_t sdSectorCount = 0;
static bool     sdIsHC        = false;

// ══════════════════════════════════════════════════════════════════════════
// SPI HELPERS
// ══════════════════════════════════════════════════════════════════════════

static uint8_t sdTransfer(uint8_t b) { return sdSPI.transfer(b); }
static void sdSelect()   { digitalWrite(SD_CS_PIN, LOW);  delayMicroseconds(1); }
static void sdDeselect() { digitalWrite(SD_CS_PIN, HIGH); sdSPI.transfer(0xFF); }

// ══════════════════════════════════════════════════════════════════════════
// SD RAW INIT
// ══════════════════════════════════════════════════════════════════════════

static uint8_t sdCmd(uint8_t cmd, uint32_t arg) {
    sdDeselect();
    sdTransfer(0xFF);
    sdSelect();
    sdTransfer(0x40 | cmd);
    sdTransfer((arg >> 24) & 0xFF);
    sdTransfer((arg >> 16) & 0xFF);
    sdTransfer((arg >>  8) & 0xFF);
    sdTransfer((arg      ) & 0xFF);
    uint8_t crc = 0xFF;
    if (cmd == 0) crc = 0x95;
    if (cmd == 8) crc = 0x87;
    sdTransfer(crc);
    uint8_t r = 0xFF;
    for (int i = 0; i < 8; i++) {
        r = sdTransfer(0xFF);
        if (!(r & 0x80)) break;
    }
    return r;
}

static bool sdRawInit() {
    sdSPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE0));
    sdDeselect();
    for (int i = 0; i < 10; i++) sdTransfer(0xFF);

    if (sdCmd(0, 0) != 0x01) { sdSPI.endTransaction(); return false; }

    bool v2 = false;
    if (sdCmd(8, 0x000001AA) == 0x01) {
        uint8_t r7[4];
        for (int i = 0; i < 4; i++) r7[i] = sdTransfer(0xFF);
        if (r7[2] == 0x01 && r7[3] == 0xAA) v2 = true;
    }

    uint32_t deadline = millis() + 2000;
    uint8_t r;
    do {
        sdCmd(55, 0);
        r = sdCmd(41, v2 ? 0x40000000 : 0);
        if (millis() > deadline) { sdSPI.endTransaction(); return false; }
    } while (r != 0x00);

    if (v2) {
        if (sdCmd(58, 0) == 0x00) {
            uint8_t ocr[4];
            for (int i = 0; i < 4; i++) ocr[i] = sdTransfer(0xFF);
            sdIsHC = (ocr[0] & 0x40) != 0;
        }
    }

    if (!sdIsHC) {
        if (sdCmd(16, 512) != 0x00) { sdSPI.endTransaction(); return false; }
    }

    sdDeselect();
    sdSPI.endTransaction();
    return true;
}

// ══════════════════════════════════════════════════════════════════════════
// SECTOR READ / WRITE
// ══════════════════════════════════════════════════════════════════════════

bool sdReadSectors(uint8_t* buf, uint32_t lba, uint32_t count) {
    sdSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    for (uint32_t i = 0; i < count; i++) {
        uint32_t addr = sdIsHC ? (lba + i) : ((lba + i) * SECTOR_SIZE);
        sdDeselect(); sdTransfer(0xFF); sdSelect();
        sdTransfer(0x40 | 17);
        sdTransfer((addr >> 24) & 0xFF); sdTransfer((addr >> 16) & 0xFF);
        sdTransfer((addr >>  8) & 0xFF); sdTransfer((addr      ) & 0xFF);
        sdTransfer(0xFF);
        uint8_t r1 = 0xFF;
        for (int t = 0; t < 10; t++) { r1 = sdTransfer(0xFF); if (!(r1 & 0x80)) break; }
        if (r1 != 0x00) { sdDeselect(); sdSPI.endTransaction(); return false; }
        uint8_t token = 0xFF;
        uint32_t dl = millis() + 500;
        while (millis() < dl) { token = sdTransfer(0xFF); if (token != 0xFF) break; }
        if (token != 0xFE) { sdDeselect(); sdSPI.endTransaction(); return false; }
        for (int b = 0; b < SECTOR_SIZE; b++) buf[i * SECTOR_SIZE + b] = sdTransfer(0xFF);
        sdTransfer(0xFF); sdTransfer(0xFF);
        sdDeselect();
    }
    sdSPI.endTransaction();
    return true;
}

bool sdWriteSectors(const uint8_t* buf, uint32_t lba, uint32_t count) {
    sdSPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
    for (uint32_t i = 0; i < count; i++) {
        uint32_t addr = sdIsHC ? (lba + i) : ((lba + i) * SECTOR_SIZE);
        sdDeselect(); sdTransfer(0xFF); sdSelect();
        sdTransfer(0x40 | 24);
        sdTransfer((addr >> 24) & 0xFF); sdTransfer((addr >> 16) & 0xFF);
        sdTransfer((addr >>  8) & 0xFF); sdTransfer((addr      ) & 0xFF);
        sdTransfer(0xFF);
        uint8_t r1 = 0xFF;
        for (int t = 0; t < 10; t++) { r1 = sdTransfer(0xFF); if (!(r1 & 0x80)) break; }
        if (r1 != 0x00) { sdDeselect(); sdSPI.endTransaction(); return false; }
        sdTransfer(0xFF); sdTransfer(0xFE);
        for (int b = 0; b < SECTOR_SIZE; b++) sdTransfer(buf[i * SECTOR_SIZE + b]);
        sdTransfer(0xFF); sdTransfer(0xFF);
        uint8_t dresp = sdTransfer(0xFF);
        if ((dresp & 0x1F) != 0x05) { sdDeselect(); sdSPI.endTransaction(); return false; }
        uint32_t dl = millis() + 2000;
        while (millis() < dl) { if (sdTransfer(0xFF) != 0x00) break; }
        sdDeselect();
    }
    sdSPI.endTransaction();
    return true;
}

// ══════════════════════════════════════════════════════════════════════════
// USB MSC CALLBACKS
// ══════════════════════════════════════════════════════════════════════════

static int32_t onRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
    (void)offset;
    uint32_t count = bufsize / SECTOR_SIZE;
    if (count == 0) return -1;
    return sdReadSectors((uint8_t*)buffer, lba, count) ? (int32_t)bufsize : -1;
}

static int32_t onWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
    (void)offset;
    uint32_t count = bufsize / SECTOR_SIZE;
    if (count == 0) return -1;
    mscWriteActive = true;
    bool ok = sdWriteSectors(buffer, lba, count);
    mscWriteActive = false;
    return ok ? (int32_t)bufsize : -1;
}

static bool onStartStop(uint8_t power_condition, bool start, bool load_eject) {
    (void)power_condition; (void)start; (void)load_eject;
    return true;
}

// ══════════════════════════════════════════════════════════════════════════
// COLORS
// ══════════════════════════════════════════════════════════════════════════

#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0
#define C_RED    0xF800
#define C_GRAY   0x7BEF
#define C_DGRAY  0x2104
#define C_ACCENT 0x07FF

// ══════════════════════════════════════════════════════════════════════════
// SD CARD SHAPE DRAW
// ══════════════════════════════════════════════════════════════════════════

void drawSDCardShape(int x, int y, int w, int h, uint16_t col) {
    M5Cardputer.Display.fillRoundRect(x, y, w, h, 4, col);

    int cutSize = 14;
    M5Cardputer.Display.fillTriangle(
        x + w - cutSize, y,
        x + w,           y,
        x + w,           y + cutSize,
        C_BLACK
    );
    M5Cardputer.Display.drawLine(
        x + w - cutSize, y,
        x + w,           y + cutSize,
        col
    );

    int pinY    = y + h - 10;
    int pinW    = 4;
    int pinGap  = 6;
    int pinStart= x + 6;
    for (int i = 0; i < 6; i++) {
        M5Cardputer.Display.fillRect(pinStart + i * pinGap, pinY, pinW, 10, C_BLACK);
    }
}

// ══════════════════════════════════════════════════════════════════════════
// MAIN UI
// ══════════════════════════════════════════════════════════════════════════

void drawMainScreen() {
    M5Cardputer.Display.fillScreen(C_BLACK);

    M5Cardputer.Display.drawFastHLine(0, 18, 240, C_DGRAY);

    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setTextColor(C_GRAY);
    M5Cardputer.Display.setCursor(4, 5);
    M5Cardputer.Display.print("USB Mass Storage");

    M5Cardputer.Display.setTextColor(C_DGRAY);
    M5Cardputer.Display.setCursor(178, 5);
    M5Cardputer.Display.print("by ");
    M5Cardputer.Display.setTextColor(C_ACCENT);
    M5Cardputer.Display.print("MOY");

    if (!sdMounted) {
        M5Cardputer.Display.setTextColor(C_RED);
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.setCursor(50, 55);
        M5Cardputer.Display.print("SD ERROR");
        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(C_GRAY);
        M5Cardputer.Display.setCursor(42, 85);
        M5Cardputer.Display.print("Check SD card");
        M5Cardputer.Display.setCursor(50, 98);
        M5Cardputer.Display.print("[R] to retry");
        return;
    }

    drawSDCardShape(168, 28, 60, 80, C_DGRAY);

    M5Cardputer.Display.setTextColor(C_GRAY);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(184, 48);
    M5Cardputer.Display.print("SD");

    if (usbActive) {
        M5Cardputer.Display.setTextColor(C_GREEN);
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.setCursor(8, 42);
        M5Cardputer.Display.print("CONNECTED");

        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(C_GRAY);
        M5Cardputer.Display.setCursor(8, 72);
        M5Cardputer.Display.print("Device ready");

        M5Cardputer.Display.setTextColor(C_DGRAY);
        M5Cardputer.Display.setCursor(8, 86);
        M5Cardputer.Display.printf("%s  %u sectors",
            sdIsHC ? "SDHC" : "SD", sdSectorCount);
    } else {
        M5Cardputer.Display.setTextColor(C_YELLOW);
        M5Cardputer.Display.setTextSize(2);
        M5Cardputer.Display.setCursor(8, 42);
        M5Cardputer.Display.print("WAITING");

        M5Cardputer.Display.setTextSize(1);
        M5Cardputer.Display.setTextColor(C_GRAY);
        M5Cardputer.Display.setCursor(8, 72);
        M5Cardputer.Display.print("Plug in USB cable");
    }

    M5Cardputer.Display.drawFastHLine(0, 118, 240, C_DGRAY);
    M5Cardputer.Display.setTextColor(C_DGRAY);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(4, 122);
    M5Cardputer.Display.print("[R] Refresh  [DEL] Safe Eject");
}

// ══════════════════════════════════════════════════════════════════════════
// ANIMATIONS
// ══════════════════════════════════════════════════════════════════════════

void animSDPulse() {
    static uint32_t lastT  = 0;
    static uint8_t  bright = 30;
    static bool     dir    = true;

    if (millis() - lastT < 30) return;
    lastT = millis();

    if (dir) bright += 4; else bright -= 4;
    if (bright >= 120) dir = false;
    if (bright <= 30)  dir = true;

    uint16_t col = usbActive
        ? M5Cardputer.Display.color565(0, bright, 0)
        : M5Cardputer.Display.color565(bright >> 1, bright >> 1, bright >> 1);

    M5Cardputer.Display.drawRoundRect(168, 28, 60, 80, 4, col);
}

void animDataFlow() {
    static uint32_t lastT = 0;
    static int      pos   = 0;

    if (!usbActive) return;
    if (millis() - lastT < 80) return;
    lastT = millis();

    M5Cardputer.Display.fillRect(6, 99, 155, 8, C_BLACK);
    for (int i = 0; i < 6; i++) {
        int x = 10 + (pos + i * 24) % 150;
        uint8_t br = (uint8_t)(200 - i * 30);
        uint16_t c = M5Cardputer.Display.color565(0, br / 5, br / 2);
        M5Cardputer.Display.fillCircle(x, 103, 3, c);
    }
    pos = (pos + 6) % 150;
}

void animWaitDots() {
    static uint32_t lastT = 0;
    static int      phase = 0;

    if (usbActive) return;
    if (millis() - lastT < 500) return;
    lastT = millis();

    M5Cardputer.Display.fillRect(6, 86, 155, 12, C_BLACK);
    M5Cardputer.Display.setTextColor(C_DGRAY);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(8, 88);
    M5Cardputer.Display.print("Waiting");
    for (int i = 0; i < phase; i++)
        M5Cardputer.Display.print(".");

    phase = (phase + 1) % 4;
}

// ══════════════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════════════

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(130);

    // Açılış ekranı kaldırıldı — doğrudan SD & USB başlatma
    M5Cardputer.Display.fillScreen(C_BLACK);

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (SD.begin(SD_CS_PIN, sdSPI, 4000000)) {
        sdSectorCount = (uint32_t)(SD.totalBytes() / SECTOR_SIZE);
        SD.end();
        delay(20);

        if (sdRawInit()) {
            sdMounted = true;

            MSC.vendorID("M5Stack");
            MSC.productID("Cardputer");
            MSC.productRevision("2.1");
            MSC.onRead(onRead);
            MSC.onWrite(onWrite);
            MSC.onStartStop(onStartStop);
            MSC.mediaPresent(true);
            MSC.begin(sdSectorCount, SECTOR_SIZE);

            USB.manufacturerName("M5Stack");
            USB.productName("Cardputer SD");
            USB.serialNumber("MSC00003");
            USB.begin();
        }
    }

    drawMainScreen();
    needRedraw = false;
}

// ══════════════════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════════════════

void loop() {
    M5Cardputer.update();

    if (sdMounted) {
        bool nowConn = tud_mounted();
        if (nowConn != usbActive) {
            usbActive  = nowConn;
            needRedraw = true;
        }
    }

    if (needRedraw) {
        drawMainScreen();
        needRedraw = false;
    }

    if (sdMounted) {
        animSDPulse();
        animDataFlow();
        animWaitDots();
    }

    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        auto st = M5Cardputer.Keyboard.keysState();

        for (char c : st.word) {
            if (c == 'r' || c == 'R') { needRedraw = true; break; }
        }

        if (st.del) {
            MSC.mediaPresent(false);
            delay(300);

            M5Cardputer.Display.fillRect(0, 119, 240, 16, C_BLACK);
            M5Cardputer.Display.setTextColor(C_GREEN);
            M5Cardputer.Display.setTextSize(1);
            M5Cardputer.Display.setCursor(8, 122);
            M5Cardputer.Display.print("Safely ejected!  Re-plug to use.");

            delay(3000);
            MSC.mediaPresent(true);
            needRedraw = true;
        }
    }

    delay(8);
}
