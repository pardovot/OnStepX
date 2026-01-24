//--------------------------------------------------------------------------------------------------
// telescope mount control - supervised features
#pragma once

#include "../../../Common.h"

#ifdef MOUNT_PRESENT
#ifdef SUPERVISED_FEATURES

#include "../../../lib/nv/Nv.h"
#include "../coordinates/Transform.h"

// NV storage addresses defined in Constants.h (NV_SUPERVISED_*)
// NV enable/disable values: SUPERVISED_ENABLED (0xFF), SUPERVISED_DISABLED (0x00)

// Validation ranges
#define SUPERVISED_RA_LIMIT_MIN         1     // minimum RA limit in degrees
#define SUPERVISED_RA_LIMIT_MAX         180   // maximum RA limit in degrees
#define SUPERVISED_SYNC_THRESHOLD_MIN   5     // minimum sync threshold in degrees
#define SUPERVISED_SYNC_THRESHOLD_MAX   30    // maximum sync threshold in degrees

#pragma pack(1)
typedef struct SupervisedSettings {
  float axis1TruePosition;      // 4 bytes - Axis 1 true position (radians)
  float axis2TruePosition;      // 4 bytes - Axis 2 true position (radians)
  uint8_t raLimitEast;          // 1 byte  - East RA limit (degrees)
  uint8_t raLimitWest;          // 1 byte  - West RA limit (degrees)
  uint8_t homeEnable;           // 1 byte  - Supervised home enable
  uint8_t raLimitEnable;        // 1 byte  - Supervised RA limit enable
  uint8_t gotoEnable;           // 1 byte  - Supervised GOTO enable
  uint8_t syncThreshold;        // 1 byte  - GOTO sync threshold (degrees)
  uint8_t memoryEnable;         // 1 byte  - Power-off memory enable
  uint8_t mountTypeValidation;  // 1 byte  - Mount type for validation
} SupervisedSettings;
#pragma pack()

#define SupervisedSettingsSize 16

class Supervised {
  public:
    // Initialize supervised features from NV storage
    void init();

    // Poll for periodic tasks (position saving)
    void poll();

    // Position calculation
    // Get the angular distance between true and virtual positions in radians
    float getDistanceBetweenTruePosAndVirtualPos();

    // Synchronize true position to virtual position before GOTO
    void syncTruePosToVirtualPos(Coordinate& gotoTarget);

    // Feature enables - Supervised Home
    bool homeEnabled();
    void setHomeEnabled(bool state);

    // Feature enables - Supervised RA Limits
    bool raLimitEnabled();
    void setRaLimitEnabled(bool state);

    // Feature enables - Supervised GOTO
    bool gotoEnabled();
    void setGotoEnabled(bool state);

    // Feature enables - Power-off Memory
    bool memoryEnabled();
    void setMemoryEnabled(bool state);

    // RA Limits configuration
    uint8_t getRaLimitEast();
    uint8_t getRaLimitWest();
    void setRaLimitEast(uint8_t degrees);
    void setRaLimitWest(uint8_t degrees);

    // GOTO sync threshold configuration
    uint8_t getSyncThreshold();
    void setSyncThreshold(uint8_t degrees);

    // Command processing
    bool command(char* reply, char* command, char* parameter,
                 bool* supressFrame, bool* numericReply, CommandError* commandError);

    // Restore positions from NV storage (called from Mount::begin())
    void restorePositions();

    // Check supervised RA limits
    // Returns: 0 = no violation, 1 = east limit exceeded, 2 = west limit exceeded
    int checkRaLimits();

    // Settings structure (public for direct access if needed)
    SupervisedSettings supervisedSettings;

  private:
    // Save true positions to NV storage
    void savePositions();

    // Validate and apply defaults to settings
    void validateSettings();

    // Write settings to NV storage
    void writeSettings();

    // Last GOTO target for centering detection
    Coordinate lastGotoTarget;
    bool lastTargetValid = false;

    // Timing for position save
    unsigned long lastSaveTime = 0;
};

extern Supervised supervised;

#endif // SUPERVISED_FEATURES
#endif // MOUNT_PRESENT
