# 1 "C:\\Users\\OFIR\\AppData\\Local\\Temp\\tmpskbcrua0"
#include <Arduino.h>
# 1 "C:/code/Astro/Fysetc E4/OnStepX-New-06-09-2025/OnStepX/OnStepX.ino"
# 44 "C:/code/Astro/Fysetc E4/OnStepX-New-06-09-2025/OnStepX/OnStepX.ino"
#define FirmwareName "On-Step"
#define FirmwareVersionMajor 10
#define FirmwareVersionMinor 24
#define FirmwareVersionPatch "c"
#define FirmwareVersionConfig 6

#include "src/Common.h"
NVS nv;
#include "src/Validate.h"
#include "src/lib/sense/Sense.h"
#include "src/lib/tasks/OnTask.h"

#include "src/telescope/Telescope.h"
extern Telescope telescope;

#include "src/plugins/Plugins.config.h"

#if DEBUG == PROFILER
  extern void profiler();
#endif
void systemServices();
void sensesPoll();
void setup();
void loop();
#line 65 "C:/code/Astro/Fysetc E4/OnStepX-New-06-09-2025/OnStepX/OnStepX.ino"
void systemServices() {
  if (!xBusy) nv.poll(false);
}

void sensesPoll() {
  sense.poll();
}

void setup() {
  #if ADDON_SELECT_PIN != OFF
    pinMode(ADDON_SELECT_PIN, OUTPUT);
    digitalWrite(ADDON_SELECT_PIN, HIGH);
  #endif

  #if DEBUG != OFF
    SERIAL_DEBUG.begin(SERIAL_DEBUG_BAUD);
    delay(2000);
  #endif


  #ifdef PIN_INIT
    PIN_INIT();
  #endif


  VLF("");
  VF("MSG: OnStepX, version "); V(FirmwareVersionMajor); V("."); V(FirmwareVersionMinor); VL(FirmwareVersionPatch);
  VF("MSG: OnStepX, MCU "); VLF(MCU_STR);
  VF("MSG: OnStepX, pinmap "); VLF(PINMAP_STR);


  VLF("MSG: Setup, HAL initialize");
  HAL_INIT();
  if (!HAL_NV_INIT()) {
    DLF("WRN: Setup, NV (EEPROM/FRAM/FlashMem/etc.) device not found!");
    nv.initError = true;
  }
  delay(2000);


  VF("MSG: Setup, start system service task (rate 10ms priority 7)... ");


  if (tasks.add(10, 0, true, 7, systemServices, "SysSvcs")) { VLF("success"); } else { VLF("FAILED!"); }


  int pollingRate = round((1000.0F/HAL_FRACTIONAL_SEC)/2.0F);
  if (pollingRate < 1) pollingRate = 1;
  VF("MSG: Setup, start input sense polling task (rate "); V(pollingRate); VF("ms priority 7)... ");
  if (tasks.add(pollingRate, 0, true, 7, sensesPoll, "SenPoll")) { VLF("success"); } else { VLF("FAILED!"); }


  telescope.init(FirmwareName, FirmwareVersionMajor, FirmwareVersionMinor, FirmwareVersionPatch, FirmwareVersionConfig);


  commandChannelInit();

  tasks.yield(2000);


  #if PLUGIN1 != OFF
    PLUGIN1.init();
  #endif
  #if PLUGIN2 != OFF
    PLUGIN2.init();
  #endif
  #if PLUGIN3 != OFF
    PLUGIN3.init();
  #endif
  #if PLUGIN4 != OFF
    PLUGIN4.init();
  #endif
  #if PLUGIN5 != OFF
    PLUGIN5.init();
  #endif
  #if PLUGIN6 != OFF
    PLUGIN6.init();
  #endif
  #if PLUGIN7 != OFF
    PLUGIN7.init();
  #endif
  #if PLUGIN8 != OFF
    PLUGIN8.init();
  #endif


  #if DEBUG == PROFILER
    tasks.add(142, 0, true, 7, profiler, "Profilr");
  #endif

  sense.poll();

  telescope.ready = true;
}

void loop() {
  tasks.yield();
}