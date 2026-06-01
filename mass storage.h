#pragma once

/*
 * USB Mass Storage
 * M5Stack Cardputer
 * Header File
 */

#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include "USB.h"
#include "USBMSC.h"
#include "tusb.h"

// ─── Pin Tanımları ────────────────────────────────────────────────────────────
#define SD_CS_PIN   12
#define SD_SCK_PIN  40
#define SD_MOSI_PIN 14
#define SD_MISO_PIN 39
#define SECTOR_SIZE 512

// ─── Renk Tanımları ───────────────────────────────────────────────────────────
#define C_BLACK  0x0000
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0
#define C_RED    0xF800
#define C_GRAY   0x7BEF
#define C_DGRAY  0x2104
#define C_ACCENT 0x07FF

// ─── Global Nesneler (extern) ─────────────────────────────────────────────────
extern USBMSC   MSC;
extern SPIClass sdSPI;

// ─── Global Durum Değişkenleri (extern) ───────────────────────────────────────
extern volatile bool mscWriteActive;
extern bool          sdMounted;
extern bool          usbActive;
extern bool          needRedraw;
extern uint32_t      sdSectorCount;
extern bool          sdIsHC;

// ─── SPI Yardımcıları ────────────────────────────────────────────────────────
uint8_t sdTransfer(uint8_t b);
void    sdSelect();
void    sdDeselect();

// ─── SD Ham Başlatma ─────────────────────────────────────────────────────────
uint8_t sdCmd(uint8_t cmd, uint32_t arg);
bool    sdRawInit();

// ─── Sektör Okuma / Yazma ────────────────────────────────────────────────────
bool sdReadSectors(uint8_t* buf, uint32_t lba, uint32_t count);
bool sdWriteSectors(const uint8_t* buf, uint32_t lba, uint32_t count);

// ─── USB MSC Callback'leri ────────────────────────────────────────────────────
int32_t onRead(uint32_t lba, uint32_t offset,
               void* buffer, uint32_t bufsize);
int32_t onWrite(uint32_t lba, uint32_t offset,
                uint8_t* buffer, uint32_t bufsize);
bool    onStartStop(uint8_t power_condition,
                    bool start, bool load_eject);

// ─── SD Kart Şekli ───────────────────────────────────────────────────────────
void drawSDCardShape(int x, int y, int w, int h, uint16_t col);

// ─── Ana UI ──────────────────────────────────────────────────────────────────
void drawMainScreen();

// ─── Animasyonlar ────────────────────────────────────────────────────────────
void animSDPulse();
void animDataFlow();
void animWaitDots();

// ─── USB & SD Başlatma ───────────────────────────────────────────────────────
void initSDAndUSB();
