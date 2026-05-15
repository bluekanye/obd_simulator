/**
 *
 * Tento program simuluje jednu ECU jednotku, ktora odpoveda na OBD-II
 * diagnosticke poziadavky cez CAN zbernicu. Vytvoril som ho v ramci
 * diplomovej prace, aby som demonstroval ako OBD-II funguje na malom
 * mikrokontroleri.
 *
 * Podporovane sluzby:
 *   - Service 01 (Current Data):  PID 01, 02, 05, 0C, 0D
 *   - Service 02 (Freeze Frame):  PID 02, 05, 0C, 0D
 *   - Service 03 (citanie DTC)
 *   - Service 04 (mazanie DTC)
 *   - Service 09 (Vehicle Info):  VIN, CALID, CVN, ECU Name
 *   - Service 11 (ECU Reset)
 *   - ISO-TP (ISO 15765-2) viacramcovy prenos s flow control
 *
 * Pouzity hardver:
 *   - Arduino Mega 2560 s WIFI
 *   - MCP2515 CAN radic (SPI, CS na pine 9, 16 MHz kristal)
 *   - 20x4 I2C LCD displej (adresa 0x27)
 *   - Analogovy joystick na A0/A1
 *   - Tlacidlo na pine 7 (aktivne LOW, vnutorny pull-up)
 */

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

// forward deklaracie - Arduino IDE ich potrebuje skor, nez si sam vygeneruje prototypy
struct DTC;
struct PIDEntry;
struct Mode09TextEntry;
typedef uint8_t (*PidEncodeFn)(uint8_t service, uint8_t pid, uint8_t* out);
static void softResetSimulator();

// =====================================================================
// KONFIGURACIA CAN
// =====================================================================
#define CAN_CS_PIN   9
#define CAN_BAUD     CAN_500KBPS
#define MCP_CLOCK    MCP_16MHZ

MCP_CAN CAN(CAN_CS_PIN);

// standardne OBD-II CAN ID podla normy ISO 15765-4
static const uint32_t OBD_REQ_FUNCTIONAL = 0x7DF;  // broadcast poziadavka, mala by odpovedat lubovolna ECU
static const uint32_t OBD_REQ_ECU1       = 0x7E0;  // poziadavka adresovana priamo na ECU #1
static const uint32_t OBD_RESP_ECU1      = 0x7E8;  // nase odpovedne ID

// =====================================================================
// OBD-II / UDS KONSTANTY
// pouzivam pomenovane konstanty namiesto magickych cisel, aby bol kod citatelnejsi
// =====================================================================

// service ID
#define OBD_SID_CURRENT_DATA    0x01
#define OBD_SID_FREEZE_FRAME    0x02
#define OBD_SID_READ_DTC        0x03
#define OBD_SID_CLEAR_DTC       0x04
#define OBD_SID_VEHICLE_INFO    0x09
#define OBD_SID_ECU_RESET       0x11

// k service ID pripocitam 0x40 a tym dostanem bajt pozitivnej odpovede
#define OBD_RESP_OFFSET         0x40

// negativna odpoved
#define OBD_NRC_NEGATIVE_RESP           0x7F
#define OBD_NRC_SUB_FUNC_NOT_SUPPORTED  0x12

// odpovedne bajty pre service 03 a 04
#define OBD_RESP_READ_DTC       0x43
#define OBD_RESP_CLEAR_DTC      0x44

// odpovedny bajt pre service 09
#define OBD_RESP_VEHICLE_INFO   0x49

// odpovedny bajt pre service 11
#define OBD_RESP_ECU_RESET      0x51

// typy ISO-TP ramcov (horny nibble PCI bajtu)
#define ISOTP_PCI_SF   0x00   // Single Frame
#define ISOTP_PCI_FF   0x10   // First Frame
#define ISOTP_PCI_CF   0x20   // Consecutive Frame
#define ISOTP_PCI_FC   0x30   // Flow Control

// stavove kody Flow Control
#define ISOTP_FS_CONTINUE  0x00
#define ISOTP_FS_WAIT      0x01
#define ISOTP_FS_OVERFLOW  0x02

// casovanie ISO-TP
#define FC_TIMEOUT_MS    1000  // ak neprijmeme FC ramec do 1s, prenos zrusime
#define MIN_CF_DELAY_MS  5     // minimalna pauza medzi po sebe iducimi CF ramcami

// =====================================================================
// LCD
// =====================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);
// ak displej neodpoveda, treba skusit adresu 0x3F
//LiquidCrystal_I2C lcd(0x3F, 20, 4);

static uint32_t lastUiDrawMs = 0;
static bool     uiDirty      = true;

// senzorove hodnoty zobrazene na displeji a posielane cez CAN
// daju sa upravovat joystickom
static int     rpm    = 1200;
static int     speed  = 45;
static int     tempC  = 87;
static uint8_t selected = 0;   // 0 = RPM, 1 = rychlost, 2 = teplota

// =====================================================================
// JOYSTICK
// =====================================================================
#define JOY_X          A0
#define JOY_Y          A1
#define JOY_DEADZONE   90    // ako daleko od stredu musi byt joystick, aby sa pohyb registroval
#define JOY_REPEAT_MS  130   // pauza medzi opakovanymi krokmi pri drzani joysticku

static int      joyX0         = 512;  // stred X, kalibrovany pri starte
static int      joyY0         = 512;  // stred Y, kalibrovany pri starte
static uint32_t lastJoyStepMs = 0;

// =====================================================================
// OK TLACIDLO
// =====================================================================
#define BTN_OK  7  // aktivne LOW, pouzivam vnutorny pull-up

static uint32_t okDownMs  = 0;
static bool     okWasDown = false;

// kratke stlacenie pod 600ms - prepina vybrane policko alebo prepne DTC v menu
// dlhe stlacenie 600ms a viac - prepina medzi hlavnou obrazovkou a DTC menu
#define BTN_LONG_PRESS_MS  600

// =====================================================================
// UI REZIM
// =====================================================================
enum UiMode : uint8_t {
  UI_MAIN     = 0,
  UI_DTC_MENU = 1
};

static UiMode uiMode        = UI_MAIN;
static uint8_t dtcCursor    = 0;   // ktory riadok je v DTC menu zvyrazneny
static uint8_t dtcScrollTop = 0;   // horny viditelny riadok v zozname DTC

// =====================================================================
// DTC - 2-bajtove OBD-II kodovanie
// =====================================================================
struct DTC {
  uint8_t hi;
  uint8_t lo;
};

static inline bool dtcEquals(const DTC& a, const DTC& b) {
  return (a.hi == b.hi) && (a.lo == b.lo);
}

/**
 * Prevedie jeden hex znak na ciselnu hodnotu (0-15).
 * Vrati false, ak znak nie je platna hex cifra.
 */
static bool hexNibble(char c, uint8_t& out) {
  c = (char)toupper((unsigned char)c);
  if (c >= '0' && c <= '9') { out = (uint8_t)(c - '0');        return true; }
  if (c >= 'A' && c <= 'F') { out = (uint8_t)(10 + c - 'A');   return true; }
  return false;
}

/**
 * Sparsuje 5-znakovy DTC retazec ako "P0299" do 2-bajtoveho OBD-II formatu.
 *
 * Kodovanie podla SAE J2012 / ISO 15031-6:
 *   hi bajt = system(2 bity) | prva cifra(2 bity) | druha cifra(4 bity)
 *   lo bajt = tretia cifra(4 bity) | stvrta cifra(4 bity)
 *
 * Vrati true ak sa to podarilo.
 */
static bool dtcFromString(const char* s, DTC& out) {
  if (!s || strlen(s) != 5) return false;

  uint8_t sys = 0;
  switch ((char)toupper((unsigned char)s[0])) {
    case 'P': sys = 0; break;
    case 'C': sys = 1; break;
    case 'B': sys = 2; break;
    case 'U': sys = 3; break;
    default:  return false;
  }

  if (s[1] < '0' || s[1] > '3') return false;
  uint8_t d1 = (uint8_t)(s[1] - '0');

  uint8_t d2 = 0, d3 = 0, d4 = 0;
  if (!hexNibble(s[2], d2)) return false;
  if (!hexNibble(s[3], d3)) return false;
  if (!hexNibble(s[4], d4)) return false;

  out.hi = (uint8_t)((sys << 6) | ((d1 & 0x03) << 4) | (d2 & 0x0F));
  out.lo = (uint8_t)((d3 << 4) | d4);
  return true;
}

/**
 * Prevedie 2-bajtove DTC spat na citatelny retazec ako "P0299".
 * Vystupny buffer musi mat aspon 6 bajtov.
 */
static void dtcToString(const DTC& code, char out[6]) {
  static const char HEX_CHARS[] = "0123456789ABCDEF";
  uint8_t sys = (code.hi >> 6) & 0x03;
  out[0] = (sys == 0) ? 'P' : (sys == 1) ? 'C' : (sys == 2) ? 'B' : 'U';
  out[1] = (char)('0' + ((code.hi >> 4) & 0x03));
  out[2] = HEX_CHARS[code.hi & 0x0F];
  out[3] = HEX_CHARS[(code.lo >> 4) & 0x0F];
  out[4] = HEX_CHARS[code.lo & 0x0F];
  out[5] = '\0';
}

// =====================================================================
// KATALOG DTC - zoznam kodov, ktore sa daju prepinat z menu
// =====================================================================
struct DtcCatalogEntry {
  const char* name;  // 5-znakovy retazec, napr. "P0299"
};

static const DtcCatalogEntry DTC_CATALOG[] = {
  { "P0299" },   // turbodychadlo - nedostatocny tlak
  { "P0021" },   // casovanie sacieho vackoveho hriadela - predcasne (Bank 2)
  { "B0001" },   // ovladanie airbagu vodica (1. stupen)
  { "C0035" },   // snimac rychlosti laveho predneho kolesa
  { "U0113" },   // strata komunikacie s modulom emisneho systemu
  { "P0101" },   // MAF senzor - rozsah/vykon
  { "P0700" },   // porucha riadenia prevodovky
  { "P0100" },   // porucha obvodu MAF senzora
  { "U0100" },   // strata komunikacie s ECM/PCM
};
static const uint8_t DTC_CAT_N = (uint8_t)(sizeof(DTC_CATALOG) / sizeof(DTC_CATALOG[0]));

// =====================================================================
// ZOZNAM AKTIVNYCH DTC
// =====================================================================
#define MAX_DTC  10  // naraz drzime maximalne 10 aktivnych DTC

struct DtcState {
  DTC     active[MAX_DTC];
  uint8_t count;
} dtc = { {}, 0 };

// prida DTC do zoznamu aktivnych, duplicity a pretecenie potichu ignoruje
static bool dtcAdd(const DTC& code) {
  for (uint8_t i = 0; i < dtc.count; i++) {
    if (dtcEquals(dtc.active[i], code)) return true;  // uz je v zozname
  }
  if (dtc.count >= MAX_DTC) return false;             // zoznam je plny
  dtc.active[dtc.count++] = code;
  return true;
}

static bool dtcAddByName(const char* name) {
  DTC code;
  if (!dtcFromString(name, code)) return false;
  return dtcAdd(code);
}

static void dtcRemove(const DTC& code) {
  for (uint8_t i = 0; i < dtc.count; i++) {
    if (dtcEquals(dtc.active[i], code)) {
      // vsetko za touto polozkou posuniem o jeden krok dolava
      for (uint8_t j = i; j + 1 < dtc.count; j++) {
        dtc.active[j] = dtc.active[j + 1];
      }
      dtc.count--;
      return;
    }
  }
}

static bool dtcIsActive(const DTC& code) {
  for (uint8_t i = 0; i < dtc.count; i++) {
    if (dtcEquals(dtc.active[i], code)) return true;
  }
  return false;
}

static bool dtcCatalogCode(uint8_t idx, DTC& out) {
  if (idx >= DTC_CAT_N) return false;
  return dtcFromString(DTC_CATALOG[idx].name, out);
}

static void dtcClearAll() {
  dtc.count = 0;
}

// =====================================================================
// FREEZE FRAME
// Snapshot senzorovych hodnot v okamihu, ked sa DTC prvykrat nastavilo.
// Posielam ho cez Service 02. Drzim len jeden ramec (frame number 0x00).
// Bajt cisla ramca v prichadzajucich Mode 02 poziadavkach ignorujem.
// =====================================================================
struct FreezeFrame {
  bool    valid;
  uint8_t causeHi;   // hi bajt DTC, ktory tento freeze frame spustil
  uint8_t causeLo;   // lo bajt DTC, ktory tento freeze frame spustil
  int     rpm;
  int     speed;
  int     tempC;
} freezeFrame = { false, 0, 0, 0, 0, 0 };

// =====================================================================
// DATA PRE SERVICE 09 (informacie o vozidle)
// =====================================================================
struct Mode09TextEntry {
  uint8_t     pid;
  const char* text;
};

static const Mode09TextEntry MODE09_TEXT_TABLE[] = {
  { 0x02, "AdamT10437898013A" },  // VIN, 17 znakov podla ISO 3779
  { 0x04, "CALID-ARDUINO-001" },  // kalibracne ID
  { 0x0A, "ECU_SIM_MAIN"     },  // nazov ECU
};
static const uint8_t MODE09_TEXT_N = (uint8_t)(sizeof(MODE09_TEXT_TABLE) / sizeof(MODE09_TEXT_TABLE[0]));
static const uint32_t MODE09_CVN   = 0x1234ABCDUL;  // kalibracne overovacie cislo

// =====================================================================
// POMOCNA FUNKCIA NA ODOSIELANIE CAN RAMCOV
// =====================================================================

// posle pevny 8-bajtovy CAN ramec, volajuci musi nepouzite bajty vynulovat
static void sendCan8(uint32_t canId, const uint8_t data[8]) {
  CAN.sendMsgBuf(canId, 0, 8, (uint8_t*)data);
}

// =====================================================================
// ISO-TP TX STATE MACHINE (ISO 15765-2)
//
// Spracuva viacramcove odpovede s flow control.
// Implementoval som len vysielaciu stranu - skladanie prichadzajucich
// viacramcovych poziadaviek pre jednoduchy OBD simulator netreba,
// pretoze vsetky poziadavky sa zmestia do jedneho CAN ramca.
//
// Sekvencne cisla cykluju 0x0 -> 0xF -> 0x0 podla ISO 15765-2 sekcia 9.6.3.
// =====================================================================
struct IsoTpTx {
  bool     active;
  bool     waitingFc;      // true pokym cakame na Flow Control ramec

  uint8_t  payload[160];   // buffer drziaci celu odpoved, ktoru treba poslat
  uint16_t payloadLen;     // celkovy pocet bajtov na odoslanie
  uint16_t offset;         // index dalsieho bajtu na odoslanie

  uint8_t  seq;            // sekvencne cislo CF ramca, 0-15
  uint8_t  blockSize;      // block size z posledneho FC ramca (0 = bez limitu)
  uint8_t  blockSent;      // kolko CF sme uz poslali v aktualnom bloku
  uint8_t  stMin;          // STmin z FC ramca

  uint32_t nextSendMs;     // najskorsi cas, kedy smieme poslat dalsi CF
  uint32_t fcDeadlineMs;   // ak do tohto casu nepride FC, prenos zrusime
} iso = { false, false, {}, 0, 0, 1, 0, 0, 0, 0, 0 };

/**
 * Prevedie STmin bajt z Flow Control ramca na pauzu v milisekundach.
 *
 * Podla ISO 15765-2:
 *   0x00-0x7F  ->  0-127 ms
 *   0xF1-0xF9  ->  100-900 us (zaokruhlujem nahor na 1 ms)
 *   cokolvek ine -> 0 ms
 *
 * Navyse aplikujem minimalnu hranicu MIN_CF_DELAY_MS kvoli spolahlivosti na Arduine.
 */
static uint32_t stMinToDelayMs(uint8_t stMin) {
  uint32_t d = 0;
  if (stMin <= 0x7F)                        d = (uint32_t)stMin;
  else if (stMin >= 0xF1 && stMin <= 0xF9)  d = 1;
  if (d < MIN_CF_DELAY_MS) d = MIN_CF_DELAY_MS;
  return d;
}

/**
 * Spusti odosielanie ISO-TP odpovede.
 *
 * Ak sa payload zmesti do 7 bajtov, posle sa hned ako Single Frame.
 * Inak posleme First Frame a cakame na Flow Control ramec, kym budeme
 * pokracovat s Consecutive Frame.
 */
static void isotpStartResponse(uint32_t respId, const uint8_t* payload, uint16_t payloadLen) {
  // pred zacatim noveho prenosu vynulujem stav
  iso.active      = true;
  iso.waitingFc   = false;
  iso.offset      = 0;
  iso.seq         = 1;
  iso.blockSize   = 0;
  iso.blockSent   = 0;
  iso.stMin       = 0;
  iso.nextSendMs  = millis();
  iso.fcDeadlineMs = 0;

  if (payloadLen > (uint16_t)sizeof(iso.payload)) payloadLen = (uint16_t)sizeof(iso.payload);
  iso.payloadLen = payloadLen;
  memcpy(iso.payload, payload, payloadLen);

  if (payloadLen <= 7) {
    // zmesti sa do jedneho ramca, posleme a sme hotovi
    uint8_t sf[8] = { 0 };
    sf[0] = (uint8_t)(ISOTP_PCI_SF | (payloadLen & 0x0F));
    memcpy(&sf[1], iso.payload, payloadLen);
    sendCan8(respId, sf);
    iso.active = false;
    return;
  }

  // nezmesti sa - posleme First Frame a cakame na flow control
  uint8_t ff[8] = { 0 };
  ff[0] = (uint8_t)(ISOTP_PCI_FF | ((payloadLen >> 8) & 0x0F));
  ff[1] = (uint8_t)(payloadLen & 0xFF);
  memcpy(&ff[2], iso.payload, 6);
  sendCan8(respId, ff);

  iso.offset      = 6;
  iso.waitingFc   = true;
  iso.fcDeadlineMs = millis() + FC_TIMEOUT_MS;
}

/**
 * Spracuje prichadzajuci Flow Control ramec od testera.
 *
 * FS=0 (Continue): zaciname alebo pokracujeme s odosielanim Consecutive Frame
 * FS=1 (Wait):     ostaneme v necinnosti, resetujem FC timeout
 * FS=2 (Overflow): tester to nezvlada, prenos zrusime
 */
static void isotpOnFlowControl(const uint8_t* rx, uint8_t len) {
  if (!iso.active || !iso.waitingFc || len < 3) return;
  if (((rx[0] >> 4) & 0x0F) != (ISOTP_PCI_FC >> 4)) return;

  uint8_t fs = rx[0] & 0x0F;

  if (fs == ISOTP_FS_OVERFLOW) {
    iso.active    = false;
    iso.waitingFc = false;
    return;
  }
  if (fs == ISOTP_FS_WAIT) {
    iso.fcDeadlineMs = millis() + FC_TIMEOUT_MS;
    return;
  }
  // FS_CONTINUE - precitam block size a STmin a zacnem posielat
  iso.blockSize   = rx[1];
  iso.stMin       = rx[2];
  iso.blockSent   = 0;
  iso.waitingFc   = false;
  iso.nextSendMs  = millis() + stMinToDelayMs(iso.stMin);
}

/**
 * ISO-TP TX pump - volana v kazdej iteracii hlavnej slucky.
 *
 * Posiela dalsi Consecutive Frame, ked to casovanie a block size dovoluje.
 * Sekvencne cislo cykluje z 0xF spat na 0x0 ako vyzaduje standard.
 */
static void isotpProcess(uint32_t respId) {
  if (!iso.active) return;

  if (iso.waitingFc) {
    if (millis() > iso.fcDeadlineMs) {
      // tester neodpovedal vcas, koncime
      iso.active    = false;
      iso.waitingFc = false;
    }
    return;
  }

  if (millis() < iso.nextSendMs) return;
  if (iso.offset >= iso.payloadLen) { iso.active = false; return; }

  if (iso.blockSize != 0 && iso.blockSent >= iso.blockSize) {
    // dosiahli sme limit bloku, cakame na dalsi FC pred pokracovanim
    iso.waitingFc    = true;
    iso.fcDeadlineMs = millis() + FC_TIMEOUT_MS;
    return;
  }

  // postavim a odoslem dalsi Consecutive Frame
  uint8_t cf[8] = { 0 };
  cf[0] = (uint8_t)(ISOTP_PCI_CF | (iso.seq & 0x0F));
  iso.seq = (iso.seq >= 0x0F) ? 0x00 : iso.seq + 1;  // wrap 0xF -> 0x0

  uint8_t bytesSent = 0;
  while (bytesSent < 7 && iso.offset < iso.payloadLen) {
    cf[1 + bytesSent++] = iso.payload[iso.offset++];
  }

  sendCan8(respId, cf);
  iso.blockSent++;
  iso.nextSendMs = millis() + stMinToDelayMs(iso.stMin);
}

// =====================================================================
// PARSOVANIE OBD POZIADAVIEK
// =====================================================================

/**
 * Vytiahne service ID a PID zo Single Frame OBD poziadavky.
 * Vrati true ak ramec vyzera platne a obsahuje aspon tieto dva bajty.
 */
static bool parseServicePid(const uint8_t* rx, uint8_t len, uint8_t& service, uint8_t& pid) {
  if (len < 3) return false;
  uint8_t pciType = (rx[0] >> 4) & 0x0F;
  if (pciType == ISOTP_PCI_SF || rx[0] <= 7) {
    service = rx[1];
    pid     = rx[2];
    return true;
  }
  return false;
}

/**
 * Vytiahne len service ID zo Single Frame OBD poziadavky.
 * Pouziva sa pre sluzby, ktore nenesu PID (napr. 03, 04, 11).
 */
static bool parseServiceOnly(const uint8_t* rx, uint8_t len, uint8_t& service) {
  if (len < 2) return false;
  uint8_t pciType = (rx[0] >> 4) & 0x0F;
  if (pciType == ISOTP_PCI_SF || rx[0] <= 7) {
    service = rx[1];
    return true;
  }
  return false;
}

// =====================================================================
// POMOCNE FUNKCIE NA ODPOVEDE
// =====================================================================

// posle single-frame ISO-TP odpoved a vypise ju na seriovy port pre debug
static void sendSingleFramePayload(const uint8_t* payload, uint8_t payloadLen) {
  if (payloadLen > 7) payloadLen = 7;

  uint8_t sf[8] = { 0 };
  sf[0] = payloadLen;
  memcpy(&sf[1], payload, payloadLen);

  Serial.print(F("CAN TX 0x")); Serial.print(OBD_RESP_ECU1, HEX); Serial.print(F(": "));
  for (uint8_t i = 0; i < 8; i++) {
    if (sf[i] < 0x10) Serial.print('0');
    Serial.print(sf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  CAN.sendMsgBuf(OBD_RESP_ECU1, 0, 8, sf);
}

// posle negativnu odpoved (NRC) testeru
static void sendNegativeResponse(uint8_t service, uint8_t nrc) {
  uint8_t payload[3] = { OBD_NRC_NEGATIVE_RESP, service, nrc };
  sendSingleFramePayload(payload, 3);
}

/**
 * Postavi 4-bajtovu bitovu mapu podporovanych PID, pouzitu v odpovediach na PID 0x00.
 *
 * Rozlozenie bitov: bit 7 bajtu 0 = PID 0x01, bit 0 bajtu 3 = PID 0x20.
 * PID mimo rozsahu 0x01-0x20 zatial ignorujem.
 */
static void buildSupportedBitmap(const uint8_t* pids, uint8_t count, uint8_t* out4) {
  out4[0] = out4[1] = out4[2] = out4[3] = 0;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t pid = pids[i];
    if (pid < 1 || pid > 0x20) continue;
    uint8_t bitIndex  = (uint8_t)(pid - 1);
    uint8_t byteIdx   = bitIndex / 8;
    uint8_t bitInByte = bitIndex % 8;
    out4[byteIdx] |= (uint8_t)(0x80 >> bitInByte);
  }
}

// =====================================================================
// KODOVACIE FUNKCIE PID (Service 01 a 02)
// Kazda naplni vystupny buffer kompletnym payloadom OBD odpovede
// a vrati pocet zapisanych bajtov. Navratova hodnota 0 znamena ziadne data.
// =====================================================================

// Service 01 PID 01 - stav monitorovania od posledneho vymazania DTC
static uint8_t encode0101(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  out[0] = OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET;
  out[1] = pid;
  // bit 7 bajtu A = MIL kontrolka zap/vyp, dolnych 7 bitov = pocet DTC
  out[2] = (uint8_t)((dtc.count > 0 ? 0x80 : 0x00) | (dtc.count & 0x7F));
  // bajty B-D su pripravenost monitorov, drzim ich staticky
  out[3] = 0x07;
  out[4] = 0x65;
  out[5] = 0x00;
  return 6;
}

// Service 01 PID 02 - ktore DTC spustilo aktualny freeze frame
static uint8_t encode0102(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  out[0] = OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = freezeFrame.valid ? freezeFrame.causeHi : 0x00;
  out[3] = freezeFrame.valid ? freezeFrame.causeLo : 0x00;
  return 4;
}

// Service 01 PID 05 - teplota chladiacej kvapaliny
// OBD kodovanie: hodnota = teplota + 40, takze -40C = 0x00, 0C = 0x28, 87C = 0x7F
static uint8_t encode0105(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  int t = tempC;
  if (t < -40) t = -40;
  if (t > 215) t = 215;
  out[0] = OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = (uint8_t)(t + 40);
  return 3;
}

// Service 01 PID 0C - otacky motora
// OBD kodovanie: raw = RPM * 4, posielane ako 16-bit big-endian
static uint8_t encode010C(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  int r = rpm;
  if (r < 0) r = 0;
  if (r > 16383) r = 16383;   // max co sa zmesti do 16 bitov pri 0.25 rpm/LSB
  uint16_t raw = (uint16_t)(r * 4);
  out[0] = OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = (uint8_t)(raw >> 8);
  out[3] = (uint8_t)(raw & 0xFF);
  return 4;
}

// Service 01 PID 0D - rychlost vozidla v km/h, jeden bajt
static uint8_t encode010D(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  int s = speed;
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  out[0] = OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = (uint8_t)s;
  return 3;
}

// Service 02 PID 02 - DTC, ktory sposobil freeze frame (frame number 0x00)
static uint8_t encode0202(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  out[0] = OBD_SID_FREEZE_FRAME + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = 0x00;  // cislo ramca
  out[3] = freezeFrame.valid ? freezeFrame.causeHi : 0x00;
  out[4] = freezeFrame.valid ? freezeFrame.causeLo : 0x00;
  return 5;
}

// Service 02 PID 05 - teplota chladiacej kvapaliny z freeze frame
static uint8_t encode0205(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  if (!freezeFrame.valid) return 0;
  int t = freezeFrame.tempC;
  if (t < -40) t = -40;
  if (t > 215) t = 215;
  out[0] = OBD_SID_FREEZE_FRAME + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = 0x00;  // cislo ramca
  out[3] = (uint8_t)(t + 40);
  return 4;
}

// Service 02 PID 0C - RPM z freeze frame
static uint8_t encode020C(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  if (!freezeFrame.valid) return 0;
  int r = freezeFrame.rpm;
  if (r < 0) r = 0;
  if (r > 16383) r = 16383;
  uint16_t raw = (uint16_t)(r * 4);
  out[0] = OBD_SID_FREEZE_FRAME + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = 0x00;  // cislo ramca
  out[3] = (uint8_t)(raw >> 8);
  out[4] = (uint8_t)(raw & 0xFF);
  return 5;
}

// Service 02 PID 0D - rychlost vozidla z freeze frame
static uint8_t encode020D(uint8_t /*service*/, uint8_t pid, uint8_t* out) {
  if (!freezeFrame.valid) return 0;
  int s = freezeFrame.speed;
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  out[0] = OBD_SID_FREEZE_FRAME + OBD_RESP_OFFSET;
  out[1] = pid;
  out[2] = 0x00;  // cislo ramca
  out[3] = (uint8_t)s;
  return 4;
}

// =====================================================================
// DISPATCH TABULKA PID
// =====================================================================
struct PIDEntry {
  uint8_t      service;
  uint8_t      pid;
  PidEncodeFn  encode;
};

static const PIDEntry PID_TABLE[] = {
  { OBD_SID_CURRENT_DATA, 0x01, encode0101 },
  { OBD_SID_CURRENT_DATA, 0x02, encode0102 },
  { OBD_SID_CURRENT_DATA, 0x05, encode0105 },
  { OBD_SID_CURRENT_DATA, 0x0C, encode010C },
  { OBD_SID_CURRENT_DATA, 0x0D, encode010D },

  { OBD_SID_FREEZE_FRAME, 0x02, encode0202 },
  { OBD_SID_FREEZE_FRAME, 0x05, encode0205 },
  { OBD_SID_FREEZE_FRAME, 0x0C, encode020C },
  { OBD_SID_FREEZE_FRAME, 0x0D, encode020D },
};
static const uint8_t PID_TABLE_N = (uint8_t)(sizeof(PID_TABLE) / sizeof(PID_TABLE[0]));

static const PIDEntry* findPidEntry(uint8_t service, uint8_t pid) {
  for (uint8_t i = 0; i < PID_TABLE_N; i++) {
    if (PID_TABLE[i].service == service && PID_TABLE[i].pid == pid) {
      return &PID_TABLE[i];
    }
  }
  return nullptr;
}

/**
 * Posle odpoved s bitovou mapou podporovanych PID (PID 0x00) pre dany service.
 * Vrati true, ak je dany service taky, ktory spracuvame.
 */
static bool sendSupportedPidsResponse(uint8_t service) {
  uint8_t supported[16];
  uint8_t count = 0;

  for (uint8_t i = 0; i < PID_TABLE_N && count < (uint8_t)sizeof(supported); i++) {
    if (PID_TABLE[i].service == service) supported[count++] = PID_TABLE[i].pid;
  }

  uint8_t bm[4];
  buildSupportedBitmap(supported, count, bm);

  if (service == OBD_SID_CURRENT_DATA) {
    uint8_t payload[6] = { (uint8_t)(OBD_SID_CURRENT_DATA + OBD_RESP_OFFSET), 0x00,
                           bm[0], bm[1], bm[2], bm[3] };
    sendSingleFramePayload(payload, 6);
    return true;
  }
  if (service == OBD_SID_FREEZE_FRAME) {
    // odpoved na Mode 02 PID 00 obsahuje navyse bajt cisla ramca
    uint8_t payload[7] = { (uint8_t)(OBD_SID_FREEZE_FRAME + OBD_RESP_OFFSET), 0x00, 0x00,
                           bm[0], bm[1], bm[2], bm[3] };
    sendSingleFramePayload(payload, 7);
    return true;
  }
  return false;
}

static bool handlePidFromTable(uint8_t service, uint8_t pid) {
  const PIDEntry* entry = findPidEntry(service, pid);
  if (!entry) return false;

  uint8_t payload[8] = { 0 };
  uint8_t plen = entry->encode(service, pid, payload);
  if (plen == 0) return false;
  sendSingleFramePayload(payload, plen);
  return true;
}

// =====================================================================
// SERVICE 03 - Citanie ulozenych DTC
//
// Format odpovede (SAE J1979 sekcia 5.3):
//   bajt 0:   0x43
//   bajt 1:   pocet DTC
//   bajt 2+:  dvojice (hi, lo) bajtov pre kazdy DTC
// =====================================================================
static uint16_t buildMode03Payload(uint8_t* out, uint16_t maxLen) {
  uint8_t n = dtc.count;
  uint16_t needed = 2u + (uint16_t)n * 2u;
  if (needed > maxLen) n = (uint8_t)((maxLen - 2u) / 2u);

  out[0] = OBD_RESP_READ_DTC;
  out[1] = n;
  uint16_t idx = 2;
  for (uint8_t i = 0; i < n; i++) {
    out[idx++] = dtc.active[i].hi;
    out[idx++] = dtc.active[i].lo;
  }
  return idx;
}


static void replyMode03() {
  uint8_t  payload[128];
  uint16_t plen = buildMode03Payload(payload, sizeof(payload));
  isotpStartResponse(OBD_RESP_ECU1, payload, plen);
}

// =====================================================================
// SERVICE 04 - Mazanie DTC a freeze frame
// =====================================================================
static void replyMode04() {
  dtcClearAll();
  freezeFrame = { false, 0, 0, 0, 0, 0 };
  dtcCursor    = 0;
  dtcScrollTop = 0;
  uiDirty      = true;

  uint8_t payload[1] = { OBD_RESP_CLEAR_DTC };
  sendSingleFramePayload(payload, 1);
}

// =====================================================================
// SERVICE 09 - Informacie o vozidle
// =====================================================================
static const Mode09TextEntry* findMode09TextEntry(uint8_t pid) {
  for (uint8_t i = 0; i < MODE09_TEXT_N; i++) {
    if (MODE09_TEXT_TABLE[i].pid == pid) return &MODE09_TEXT_TABLE[i];
  }
  return nullptr;
}

static void reply0900_supportedInfoTypes() {
  const uint8_t supported[] = { 0x02, 0x04, 0x06, 0x0A };
  uint8_t bm[4];
  buildSupportedBitmap(supported, sizeof(supported), bm);
  uint8_t payload[6] = { OBD_RESP_VEHICLE_INFO, 0x00, bm[0], bm[1], bm[2], bm[3] };
  sendSingleFramePayload(payload, 6);
}
static void reply09TextPid(uint8_t pid, const char* text) {
  uint8_t  payload[96];
  uint16_t idx = 0;
  payload[idx++] = OBD_RESP_VEHICLE_INFO;
  payload[idx++] = pid;
  payload[idx++] = 0x01;  // pocet sprav, v tomto simulatore vzdy 1

  // VIN (PID 0x02) potrebuje 3 padding bajty pred retazcom podla ISO 15031-5
  // aby skenery ako Car Scanner spravne zobrazili cely retazec
  if (pid == 0x02) {
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
    payload[idx++] = 0x00;
  }

  uint16_t n = (uint16_t)strlen(text);
  for (uint16_t i = 0; i < n && idx < (uint16_t)sizeof(payload); i++) {
    payload[idx++] = (uint8_t)text[i];
  }
  isotpStartResponse(OBD_RESP_ECU1, payload, idx);
}

static void reply0906_cvn() {
  uint8_t payload[7] = {
    OBD_RESP_VEHICLE_INFO,
    0x06,
    0x01,   // pocet sprav
    (uint8_t)((MODE09_CVN >> 24) & 0xFF),
    (uint8_t)((MODE09_CVN >> 16) & 0xFF),
    (uint8_t)((MODE09_CVN >>  8) & 0xFF),
    (uint8_t)( MODE09_CVN        & 0xFF)
  };
  sendSingleFramePayload(payload, 7);
}

static bool handleMode09(uint8_t pid) {
  if (pid == 0x00) { reply0900_supportedInfoTypes(); return true; }
  if (pid == 0x06) { reply0906_cvn();                return true; }

  const Mode09TextEntry* item = findMode09TextEntry(pid);
  if (!item) return false;
  reply09TextPid(pid, item->text);
  return true;
}

// =====================================================================
// DISPATCHER OBD POZIADAVIEK
// =====================================================================
static void handleObdRequest(uint32_t canId, const uint8_t* rx, uint8_t len) {
  // kazdy prichadzajuci ramec si logujem na seriovy port, aby som mohol debugovat monitorom
  Serial.print(F("CAN RX 0x")); Serial.print(canId, HEX); Serial.print(F(": "));
  for (uint8_t i = 0; i < len; i++) {
    if (rx[i] < 0x10) Serial.print('0');
    Serial.print(rx[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  if (canId != OBD_REQ_FUNCTIONAL && canId != OBD_REQ_ECU1) return;

  uint8_t service = 0;
  uint8_t pid     = 0;

  // poziadavky obsahujuce PID bajt
  if (parseServicePid(rx, len, service, pid)) {

    if ((service == OBD_SID_CURRENT_DATA || service == OBD_SID_FREEZE_FRAME) && pid == 0x00) {
      if (!sendSupportedPidsResponse(service)) sendNegativeResponse(service, OBD_NRC_SUB_FUNC_NOT_SUPPORTED);
      return;
    }

    if (service == OBD_SID_VEHICLE_INFO) {
      if (!handleMode09(pid)) sendNegativeResponse(service, OBD_NRC_SUB_FUNC_NOT_SUPPORTED);
      return;
    }

    if (handlePidFromTable(service, pid)) return;

    if (service == OBD_SID_CURRENT_DATA || service == OBD_SID_FREEZE_FRAME) {
      sendNegativeResponse(service, OBD_NRC_SUB_FUNC_NOT_SUPPORTED);
      return;
    }
  }

  // poziadavky, ktore nesu len service bajt
  uint8_t s = 0;
  if (parseServiceOnly(rx, len, s)) {
    if (s == OBD_SID_READ_DTC)  { replyMode03(); return; }
    if (s == OBD_SID_CLEAR_DTC) { replyMode04(); return; }

    if (s == OBD_SID_ECU_RESET) {
      // sub-funkcia: 0x01 = hard reset, 0x02 = key off/on, 0x03 = soft reset
      // robim soft reset bez ohladu na to, ktoru sub-funkciu si tester vyziadal
      uint8_t sub = (len >= 3) ? rx[2] : 0x01;
      softResetSimulator();
      uint8_t payload[2] = { OBD_RESP_ECU_RESET, sub };
      sendSingleFramePayload(payload, 2);
      return;
    }
  }
}

// =====================================================================
// LCD ANTI-FLICKER
// Prepise riadok len ak sa jeho obsah skutocne zmenil.
// Tym sa vyhneme blikaniu, ktore vznika ked sa cely displej zmaze a prekresli.
// =====================================================================
static char lcdCache[4][21];

static void lcdPadLine(char* line) {
  uint8_t n = (uint8_t)strlen(line);
  while (n < 20) line[n++] = ' ';
  line[20] = '\0';
}

static void lcdWriteLineIfChanged(uint8_t row, char* line) {
  lcdPadLine(line);
  if (strncmp(lcdCache[row], line, 20) != 0) {
    lcd.setCursor(0, row);
    lcd.print(line);
    strncpy(lcdCache[row], line, 20);
    lcdCache[row][20] = '\0';
  }
}

static void lcdInvalidateCache() {
  for (uint8_t r = 0; r < 4; r++) {
    memset(lcdCache[r], '\x01', 20);
    lcdCache[r][20] = '\0';
  }
}

// =====================================================================
// FUNKCIE NA VYKRESLENIE UI
// =====================================================================
static void drawMainUI() {
  char line[21];

  snprintf(line, sizeof(line), "OBD ECU SIM DTC:%-3u", dtc.count);
  lcdWriteLineIfChanged(0, line);

  snprintf(line, sizeof(line), "%cRPM: %-14d", selected == 0 ? '>' : ' ', rpm);
  lcdWriteLineIfChanged(1, line);

  snprintf(line, sizeof(line), "%cSPD: %-4d km/h   ", selected == 1 ? '>' : ' ', speed);
  lcdWriteLineIfChanged(2, line);

  snprintf(line, sizeof(line), "%cTMP: %-4d C      ", selected == 2 ? '>' : ' ', tempC);
  lcdWriteLineIfChanged(3, line);
}

static void drawDtcMenu() {
  char line[21];
  char dtcStr[6];

  snprintf(line, sizeof(line), "DTC act:%u F:%c", dtc.count, freezeFrame.valid ? 'Y' : 'N');
  lcdWriteLineIfChanged(0, line);

  for (uint8_t row = 0; row < 3; row++) {
    uint8_t idx = dtcScrollTop + row;
    if (idx >= DTC_CAT_N) {
      line[0] = '\0';
      lcdWriteLineIfChanged(row + 1, line);
      continue;
    }

    DTC  code;
    bool ok  = dtcCatalogCode(idx, code);
    bool act = ok ? dtcIsActive(code) : false;

    // zobrazujem retazec odvodeny zo zakodovanych bajtov, nie priamo katalogovy
    // retazec - tym overujem, ze obojsmerne kodovanie funguje spravne
    if (ok) dtcToString(code, dtcStr);
    else    strncpy(dtcStr, "?????", 6);

    snprintf(line, sizeof(line), "%c%s %s",
             idx == dtcCursor ? '>' : ' ',
             act ? "[*]" : "[ ]",
             dtcStr);
    lcdWriteLineIfChanged(row + 1, line);
  }
}

static void drawUI() {
  if (!uiDirty) return;
  if (uiMode == UI_MAIN) drawMainUI();
  else                    drawDtcMenu();
  uiDirty = false;
}

// =====================================================================
// POMOCNE FUNKCIE PRE SENZOROVE HODNOTY
// =====================================================================
static void clampValues() {
  rpm   = constrain(rpm,   0,   8000);
  speed = constrain(speed, 0,   255);
  tempC = constrain(tempC, -40, 120);
}

static void incSelected(int dir) {
  if (selected == 0) rpm   += 50 * dir;
  if (selected == 1) speed +=  1 * dir;
  if (selected == 2) tempC +=  1 * dir;
  clampValues();
  uiDirty = true;
}

// =====================================================================
// POMOCNE FUNKCIE PRE DTC MENU
// =====================================================================
static void dtcMenuMove(int dir) {
  int16_t c = (int16_t)dtcCursor + dir;
  c = constrain(c, 0, (int16_t)(DTC_CAT_N - 1));
  dtcCursor = (uint8_t)c;

  if (dtcCursor < dtcScrollTop) dtcScrollTop = dtcCursor;
  if (dtcCursor >= (uint8_t)(dtcScrollTop + 3)) dtcScrollTop = dtcCursor - 2;
  uiDirty = true;
}

static void captureFreezeFrameFrom(const DTC& code) {
  freezeFrame.valid    = true;
  freezeFrame.causeHi  = code.hi;
  freezeFrame.causeLo  = code.lo;
  freezeFrame.rpm      = rpm;
  freezeFrame.speed    = speed;
  freezeFrame.tempC    = tempC;
}

static void dtcMenuToggle() {
  DTC code;
  if (!dtcCatalogCode(dtcCursor, code)) return;

  if (dtcIsActive(code)) {
    dtcRemove(code);
  } else {
    dtcAdd(code);
    captureFreezeFrameFrom(code);
  }
  uiDirty = true;
}

// =====================================================================
// SOFT RESET
// Vrati simulator na startovne hodnoty.
// Aktivne DTC zamerne ponecham, pretoze realna ECU ich uklada do
// energeticky nezavislej pamate. Na ich vymazanie sa pouziva Service 04.
// =====================================================================
static void softResetSimulator() {
  uiMode       = UI_MAIN;
  selected     = 0;
  dtcCursor    = 0;
  dtcScrollTop = 0;

  rpm   = 1200;
  speed = 45;
  tempC = 87;

  freezeFrame = { false, 0, 0, 0, 0, 0 };

  iso.active      = false;
  iso.waitingFc   = false;
  iso.offset      = 0;
  iso.payloadLen  = 0;
  iso.seq         = 1;
  iso.blockSize   = 0;
  iso.blockSent   = 0;
  iso.stMin       = 0;
  iso.nextSendMs  = 0;
  iso.fcDeadlineMs = 0;

  lcdInvalidateCache();
  uiDirty = true;
}

// =====================================================================
// OBSLUHA OK TLACIDLA
// Kratke stlacenie (pod 600ms): cyklicky meni vybrane policko, alebo prepne DTC v menu
// Dlhe stlacenie (600ms a viac): prepne medzi hlavnou obrazovkou a DTC menu
// =====================================================================
static void handleOkButton() {
  uint32_t now  = millis();
  bool     down = (digitalRead(BTN_OK) == LOW);

  if (down && !okWasDown) {
    okDownMs  = now;
    okWasDown = true;
  } else if (!down && okWasDown) {
    uint32_t held = now - okDownMs;
    okWasDown = false;

    if (held < BTN_LONG_PRESS_MS) {
      // kratke stlacenie
      if (uiMode == UI_MAIN) selected = (selected + 1) % 3;
      else                    dtcMenuToggle();
      uiDirty = true;
    } else {
      // dlhe stlacenie - prepnem medzi hlavnou obrazovkou a DTC menu
      uiMode  = (uiMode == UI_MAIN) ? UI_DTC_MENU : UI_MAIN;
      uiDirty = true;
      lcdInvalidateCache();
    }
  }
}

// =====================================================================
// OBSLUHA JOYSTICKU
// Hlavna obrazovka: Y os cyklicky prechadza policka, X os meni hodnotu
// DTC menu:         X os scrolluje hore/dole po zozname
// =====================================================================
static void handleJoystickNav() {
  int      x   = analogRead(JOY_X) - joyX0;
  int      y   = analogRead(JOY_Y) - joyY0;
  uint32_t now = millis();

  if (now - lastJoyStepMs <= JOY_REPEAT_MS) return;

  bool moved = true;

  if (uiMode == UI_MAIN) {
    if      (y >  JOY_DEADZONE)  incSelected(-1);
    else if (y < -JOY_DEADZONE)  incSelected(+1);
    else if (x >  JOY_DEADZONE)  { selected = (selected + 1) % 3; uiDirty = true; }
    else if (x < -JOY_DEADZONE)  { selected = (selected + 2) % 3; uiDirty = true; }
    else moved = false;
  } else {
    // v DTC menu pouzivam len X os na scrollovanie
    if      (x >  JOY_DEADZONE)  dtcMenuMove(+1);
    else if (x < -JOY_DEADZONE)  dtcMenuMove(-1);
    else moved = false;
  }

  if (moved) lastJoyStepMs = now;
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("OBD-II ECU Simulator starting..."));

  delay(1000);  // pockam, kym sa LCD spravne nabehne, predtym nez ho zacnem inicializovat

  wdt_enable(WDTO_2S);  // watchdog: ak by sa loop() niekde zasekla viac ako 2s, doska sa resetuje

  pinMode(BTN_OK, INPUT_PULLUP);

  // pri starte si precitam stredovu polohu joysticku, aby sme mali referencny bod
  delay(50);
  joyX0 = analogRead(JOY_X);
  joyY0 = analogRead(JOY_Y);

  // inicializacia LCD
  lcd.init();
  lcd.backlight();
  lcd.noBlink();
  lcd.noCursor();
  lcd.clear();
  lcdInvalidateCache();
  lcd.setCursor(0, 0);
  lcd.print(F("Starting..."));

  // inicializacia CAN - ak zlyha, zastavime sa, lebo bez CAN nic neurobime
  if (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    lcd.setCursor(0, 1);
    lcd.print(F("CAN OK"));
    Serial.println(F("CAN OK"));
    delay(500);
  } else {
    lcd.setCursor(0, 1);
    lcd.print(F("CAN FAIL"));
    Serial.println(F("CAN FAIL"));
    wdt_disable();
    while (1) {}
  }

  // nacitam pociatocnu sadu aktivnych DTC
  dtcAddByName("P0299");
  dtcAddByName("P0021");
  dtcAddByName("B0001");
  dtcAddByName("C0035");
  dtcAddByName("U0113");
  dtcAddByName("P0700");
  dtcAddByName("P0100");
  //dtcAddByName("U0100");
  //dtcAddByName("P0101");

  // odchytim freeze frame z prveho DTC v zozname
  if (dtc.count > 0) captureFreezeFrameFrom(dtc.active[0]);

  delay(250);
  uiDirty = true;
  drawUI();
}

// =====================================================================
// HLAVNA SLUCKA
// =====================================================================
void loop() {
  wdt_reset();  // resetnem watchdog, aby nas neresetoval

  unsigned long canId = 0;
  unsigned char len   = 0;
  unsigned char buf[8];

  // kontrolujem, ci neprisiel novy CAN ramec
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&canId, &len, buf);

    // Flow Control ramce chodia na nase fyzicke ID - tie spracovavam osobitne
    if ((uint32_t)canId == OBD_REQ_ECU1) {
      uint8_t pciType = (buf[0] >> 4) & 0x0F;
      if (pciType == (ISOTP_PCI_FC >> 4)) {
        isotpOnFlowControl((uint8_t*)buf, (uint8_t)len);
        return;  // FC ramce neposielame OBD dispatcheru
      }
    }

    handleObdRequest((uint32_t)canId, (uint8_t*)buf, (uint8_t)len);
  }

  // kontrola zdravia I2C - ak LCD prestane odpovedat, reinicializujem ho
  static uint32_t lastI2cCheckMs = 0;
  if (millis() - lastI2cCheckMs > 2000) {
    lastI2cCheckMs = millis();

    Wire.beginTransmission(0x27);  // pripadne 0x3F, ak je to tvoja adresa
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
      // LCD neodpoveda, reinicializujem ho
      Serial.print(F("I2C error: "));
      Serial.println(err);

      lcd.init();
      //lcd.init();
      lcd.backlight();
      lcd.noBlink();
      lcd.noCursor();
      lcd.clear();
      lcdInvalidateCache();
      uiDirty = true;
    }
  }

  // pumpa ISO-TP TX state machine
  isotpProcess(OBD_RESP_ECU1);

  // obsluha tlacidla a joysticku
  handleOkButton();
  handleJoystickNav();

  // obnova LCD, ale nie prilis casto, lebo by sme zahltili I2C zbernicu
  if (millis() - lastUiDrawMs > 120) {
    drawUI();
    lastUiDrawMs = millis();
  }
}