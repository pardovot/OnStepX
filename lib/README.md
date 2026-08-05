# Vendored libraries

Third party libraries the firmware links against, committed so a clean checkout builds identical firmware anywhere, CI included. PlatformIO picks them up from this directory automatically, there is no `lib_deps` entry for them.

Docs, examples and extras were stripped on import, sources are otherwise unmodified copies of the versions listed below.

| Directory | Library | Version | Source |
|---|---|---|---|
| `Rtc-2.3.5` | Rtc by Makuna | 2.3.5 | https://github.com/Makuna/Rtc |

Each library keeps its own license file.

FS, LittleFS, EEPROM, Wire, SPI, WiFi, ESPmDNS, WebServer and BluetoothSerial come from the ESP32 Arduino framework and are not vendored.

## Only one library?

PlatformIO's dependency finder runs in `chain` mode, which scans `#include` directives without evaluating the `#if` around them. Its dependency graph therefore lists every library any disabled feature mentions. Building with each candidate removed showed only `Rtc` is reachable, `TIME_LOCATION_SOURCE = DS3231` pulls it in via `src/lib/tls/ds3231/DS3231.cpp`. Dropping the rest left both firmware images byte-for-byte the same size.

Turning an option back on will fail the build with a `No such file` naming the header. Vendor the matching library from the list below and it will resolve.

| Library | Needed when | Source |
|---|---|---|
| Adafruit BME280 / BMP280 (+ BusIO, Unified Sensor) | `WEATHER != OFF` | https://github.com/adafruit |
| TinyGPSPlus 1.0.3 | `TIME_LOCATION_SOURCE = GPS` | https://github.com/mikalhart/TinyGPSPlus |
| OneWire 2.3.7 | `TEMPERATURE = DS1820` | https://github.com/hjd1964/OneWire (fork) |
| Adafruit MCP23017 2.3.2 | I2C GPIO expander | https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library |
| EthernetX 2.0.2 | Ethernet operational mode | https://github.com/hjd1964/Ethernet (fork) |
| QuickPID 3.1.9 | servo motors with PID feedback | https://github.com/Dlloydev/QuickPID |
| TMCStepper 0.7.3 | TMC SPI drivers (TMC2130, TMC5160) | https://github.com/teemuatlut/TMCStepper |
| TMC2209Stepper 1.0.0 + EspSoftwareSerial 8.1.0 | `AXIS*_DRIVER_MODEL = TMC2209` (UART) instead of `TMC2209S` | https://github.com/hjd1964/TMC2209 (fork), https://github.com/plerup/espsoftwareserial |

The OneWire, Ethernet and TMC2209 entries are hjd1964 forks rather than upstream and cannot be swapped for the original projects.
