# obd_simulator
OBD II diagnostic ECU simulator for diploma thesis at STU FEI (2026)

# Popis
Systém simuluje správanie elektronickej riadiacej jednotky vozidla (ECU) 
prostredníctvom zbernice CAN. Podporuje diagnostickú komunikáciu OBD II 
podľa štandardov SAE J1979 a ISO 15765-4.

# Funkcie

- Simulácia prevádzkových parametrov (otáčky motora, rýchlosť vozidla, teplota)
- Generovanie, čítanie a mazanie diagnostických chybových kódov (DTC)
- Identifikácia vozidla (VIN) v režime Mode 09
- Podpora freeze frame údajov
- Viacrámcová komunikácia prostredníctvom ISO-TP
- Konfigurácia cez integrovaný joystick a LCD displej

# Hardvér

- Arduino Mega 2560 s WIFI (mikrokontrolér ATmega2560)
- Seeed Studio CAN-BUS Shield V2.0 (MCP2515 + MCP2551)
- 20x4 LCD displej s I2C rozhraním
- Joystick modul a ovládacie tlačidlo
- OBD-II konektor (female)

# Potrebné knižnice

Z Library Managera v Arduino IDE treba doinštalovať tieto knižnice:

| Knižnica | Autor | Verzia |
|----------|-------|--------|
| `mcp_can` | coryjfowler | 1.5.1 |
| `LiquidCrystal I2C` | Frank de Brabander | 1.1.2 |

# Podporované diagnostické režimy

| Režim | Popis |
|-------|-------|
| 01 | Aktuálne prevádzkové parametre (PID) |
| 02 | Freeze frame údaje |
| 03 | Čítanie uložených DTC kódov |
| 04 | Mazanie DTC kódov |
| 09 | Identifikačné údaje vozidla (VIN) |

Program bol vyvinutý vo Arduino IDE 2.3.8
