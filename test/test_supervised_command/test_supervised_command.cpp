/**
 * Property-Based Tests for Supervised Features Command Interface
 * 
 * Feature: supervised-features
 * Property 11: Command Round-Trip
 * 
 * For any supervised feature setting modified via a set command, 
 * the corresponding get command SHALL return the value that was set.
 * 
 * Validates: Requirements 2.4, 3.7, 4.8, 4.9, 5.8, 5.9, 7.1-7.14
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Validation ranges from design
#define SUPERVISED_RA_LIMIT_MIN         1
#define SUPERVISED_RA_LIMIT_MAX         180
#define SUPERVISED_RA_LIMIT_DEFAULT     95
#define SUPERVISED_SYNC_THRESHOLD_MIN   5
#define SUPERVISED_SYNC_THRESHOLD_MAX   30
#define SUPERVISED_SYNC_THRESHOLD_DEFAULT 15

// Command error codes (matching CommandErrors.h)
typedef enum CommandError {
  CE_NONE, CE_0, CE_CMD_UNKNOWN, CE_REPLY_UNKNOWN, CE_PARAM_RANGE, CE_PARAM_FORM
} CommandError;

// Simulated settings storage (mirrors SupervisedSettings structure)
static bool homeEnabled = true;
static bool raLimitEnabled = false;
static bool gotoEnabled = false;
static bool memoryEnabled = false;
static uint8_t raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
static uint8_t raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
static uint8_t syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;

// Getter/setter functions that mirror the Supervised class behavior
bool getHomeEnabled() { return homeEnabled; }
void setHomeEnabled(bool state) { homeEnabled = state; }

bool getRaLimitEnabled() { return raLimitEnabled; }
void setRaLimitEnabled(bool state) { raLimitEnabled = state; }

bool getGotoEnabled() { return gotoEnabled; }
void setGotoEnabled(bool state) { gotoEnabled = state; }

bool getMemoryEnabled() { return memoryEnabled; }
void setMemoryEnabled(bool state) { memoryEnabled = state; }

uint8_t getRaLimitEast() { return raLimitEast; }
uint8_t getRaLimitWest() { return raLimitWest; }

void setRaLimitEast(uint8_t degrees) {
    if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
        raLimitEast = degrees;
    }
}

void setRaLimitWest(uint8_t degrees) {
    if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
        raLimitWest = degrees;
    }
}

uint8_t getSyncThreshold() { return syncThreshold; }

void setSyncThreshold(uint8_t degrees) {
    if (degrees >= SUPERVISED_SYNC_THRESHOLD_MIN && degrees <= SUPERVISED_SYNC_THRESHOLD_MAX) {
        syncThreshold = degrees;
    }
}

/**
 * Simulated command processor that mirrors Supervised::command()
 * 
 * @param reply Output buffer for command response
 * @param command The command string (e.g., "SH", "SR", "SG", "SM", "SD")
 * @param parameter The parameter string
 * @param numericReply Output flag for numeric reply
 * @param commandError Output error code
 * @return true if command was handled, false otherwise
 */
bool processCommand(char* reply, const char* command, const char* parameter,
                    bool* numericReply, CommandError* commandError) {
    *commandError = CE_NONE;
    *numericReply = true;
    
    // All supervised commands start with 'S'
    if (command[0] != 'S') return false;
    
    // :SH - Supervised Home commands
    if (command[1] == 'H') {
        // :SHg# - Get supervised home status
        if (parameter[0] == 'g' && parameter[1] == 0) {
            sprintf(reply, "%d", getHomeEnabled() ? 1 : 0);
            *numericReply = false;
            return true;
        }
        
        // :SHs[n]# - Set supervised home status
        if (parameter[0] == 's' && parameter[2] == 0) {
            if (parameter[1] == '0') {
                setHomeEnabled(false);
                return true;
            } else if (parameter[1] == '1') {
                setHomeEnabled(true);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        *commandError = CE_CMD_UNKNOWN;
        return true;
    }

    // :SR - Supervised RA Limit commands
    if (command[1] == 'R') {
        // :SRg# - Get supervised RA limit status
        if (parameter[0] == 'g' && parameter[1] == 0) {
            sprintf(reply, "%d", getRaLimitEnabled() ? 1 : 0);
            *numericReply = false;
            return true;
        }
        
        // :SRs[n]# - Set supervised RA limit status
        if (parameter[0] == 's' && parameter[2] == 0) {
            if (parameter[1] == '0') {
                setRaLimitEnabled(false);
                return true;
            } else if (parameter[1] == '1') {
                setRaLimitEnabled(true);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        // :SRl# - Get RA limits
        if (parameter[0] == 'l' && parameter[1] == 0) {
            sprintf(reply, "%d|%d", getRaLimitEast(), getRaLimitWest());
            *numericReply = false;
            return true;
        }
        
        // :SRlE[n]# - Set east RA limit
        if (parameter[0] == 'l' && parameter[1] == 'E') {
            int value = atoi(&parameter[2]);
            if (value >= SUPERVISED_RA_LIMIT_MIN && value <= SUPERVISED_RA_LIMIT_MAX) {
                setRaLimitEast((uint8_t)value);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        // :SRlW[n]# - Set west RA limit
        if (parameter[0] == 'l' && parameter[1] == 'W') {
            int value = atoi(&parameter[2]);
            if (value >= SUPERVISED_RA_LIMIT_MIN && value <= SUPERVISED_RA_LIMIT_MAX) {
                setRaLimitWest((uint8_t)value);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        *commandError = CE_CMD_UNKNOWN;
        return true;
    }

    // :SG - Supervised GOTO commands
    if (command[1] == 'G') {
        // :SGg# - Get supervised GOTO status
        if (parameter[0] == 'g' && parameter[1] == 0) {
            sprintf(reply, "%d", getGotoEnabled() ? 1 : 0);
            *numericReply = false;
            return true;
        }
        
        // :SGs[n]# - Set supervised GOTO status
        if (parameter[0] == 's' && parameter[2] == 0) {
            if (parameter[1] == '0') {
                setGotoEnabled(false);
                return true;
            } else if (parameter[1] == '1') {
                setGotoEnabled(true);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        // :SGt# - Get GOTO sync threshold
        if (parameter[0] == 't' && parameter[1] == 0) {
            sprintf(reply, "%d", getSyncThreshold());
            *numericReply = false;
            return true;
        }
        
        // :SGt[n]# - Set GOTO sync threshold
        if (parameter[0] == 't' && parameter[1] != 0) {
            int value = atoi(&parameter[1]);
            if (value >= SUPERVISED_SYNC_THRESHOLD_MIN && value <= SUPERVISED_SYNC_THRESHOLD_MAX) {
                setSyncThreshold((uint8_t)value);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        *commandError = CE_CMD_UNKNOWN;
        return true;
    }

    // :SM - Supervised Memory commands
    if (command[1] == 'M') {
        // :SMg# - Get power-off memory status
        if (parameter[0] == 'g' && parameter[1] == 0) {
            sprintf(reply, "%d", getMemoryEnabled() ? 1 : 0);
            *numericReply = false;
            return true;
        }
        
        // :SMs[n]# - Set power-off memory status
        if (parameter[0] == 's' && parameter[2] == 0) {
            if (parameter[1] == '0') {
                setMemoryEnabled(false);
                return true;
            } else if (parameter[1] == '1') {
                setMemoryEnabled(true);
                return true;
            } else {
                *commandError = CE_PARAM_RANGE;
                return true;
            }
        }
        
        *commandError = CE_CMD_UNKNOWN;
        return true;
    }
    
    return false;
}

// Reset all settings to defaults
void resetSettings() {
    homeEnabled = true;
    raLimitEnabled = false;
    gotoEnabled = false;
    memoryEnabled = false;
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
}

/**
 * Property 11: Command Round-Trip for Boolean Settings
 * 
 * For any boolean supervised feature setting modified via a set command,
 * the corresponding get command SHALL return the value that was set.
 * 
 * Feature: supervised-features, Property 11: Command Round-Trip
 * Validates: Requirements 2.4, 3.7, 4.8, 4.9, 5.8, 5.9, 7.1-7.14
 */
void test_property_command_roundtrip_boolean() {
    srand((unsigned int)time(NULL));
    
    char reply[64];
    bool numericReply;
    CommandError commandError;
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetSettings();
        
        // Generate random boolean value
        bool setValue = (rand() % 2) == 1;
        char setParam[4];
        sprintf(setParam, "s%d", setValue ? 1 : 0);
        
        // Test supervised home (SH)
        processCommand(reply, "SH", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        processCommand(reply, "SH", "g", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        int homeResult = atoi(reply);
        TEST_ASSERT_EQUAL_MESSAGE(setValue ? 1 : 0, homeResult, 
            "Home status get should return the value that was set");
        
        // Test supervised RA limit (SR)
        processCommand(reply, "SR", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        processCommand(reply, "SR", "g", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        int raLimitResult = atoi(reply);
        TEST_ASSERT_EQUAL_MESSAGE(setValue ? 1 : 0, raLimitResult,
            "RA limit status get should return the value that was set");
        
        // Test supervised GOTO (SG)
        processCommand(reply, "SG", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        processCommand(reply, "SG", "g", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        int gotoResult = atoi(reply);
        TEST_ASSERT_EQUAL_MESSAGE(setValue ? 1 : 0, gotoResult,
            "GOTO status get should return the value that was set");
        
        // Test supervised memory (SM)
        processCommand(reply, "SM", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        processCommand(reply, "SM", "g", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        int memoryResult = atoi(reply);
        TEST_ASSERT_EQUAL_MESSAGE(setValue ? 1 : 0, memoryResult,
            "Memory status get should return the value that was set");
    }
}

/**
 * Property 11: Command Round-Trip for RA Limits
 * 
 * For any valid RA limit value (1-180) modified via a set command,
 * the corresponding get command SHALL return the value that was set.
 * 
 * Feature: supervised-features, Property 11: Command Round-Trip
 * Validates: Requirements 4.8, 7.5-7.7
 */
void test_property_command_roundtrip_ra_limits() {
    srand((unsigned int)time(NULL) + 1);
    
    char reply[64];
    char setParam[16];
    bool numericReply;
    CommandError commandError;
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetSettings();
        
        // Generate random valid RA limit values (1-180)
        uint8_t eastValue = (uint8_t)(SUPERVISED_RA_LIMIT_MIN + 
            rand() % (SUPERVISED_RA_LIMIT_MAX - SUPERVISED_RA_LIMIT_MIN + 1));
        uint8_t westValue = (uint8_t)(SUPERVISED_RA_LIMIT_MIN + 
            rand() % (SUPERVISED_RA_LIMIT_MAX - SUPERVISED_RA_LIMIT_MIN + 1));
        
        // Set east limit
        sprintf(setParam, "lE%d", eastValue);
        processCommand(reply, "SR", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        // Set west limit
        sprintf(setParam, "lW%d", westValue);
        processCommand(reply, "SR", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        // Get limits and verify
        processCommand(reply, "SR", "l", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        // Parse "east|west" format
        int parsedEast, parsedWest;
        sscanf(reply, "%d|%d", &parsedEast, &parsedWest);
        
        TEST_ASSERT_EQUAL_MESSAGE(eastValue, parsedEast,
            "East RA limit get should return the value that was set");
        TEST_ASSERT_EQUAL_MESSAGE(westValue, parsedWest,
            "West RA limit get should return the value that was set");
    }
}

/**
 * Property 11: Command Round-Trip for Sync Threshold
 * 
 * For any valid sync threshold value (5-30) modified via a set command,
 * the corresponding get command SHALL return the value that was set.
 * 
 * Feature: supervised-features, Property 11: Command Round-Trip
 * Validates: Requirements 5.8, 7.10-7.11
 */
void test_property_command_roundtrip_sync_threshold() {
    srand((unsigned int)time(NULL) + 2);
    
    char reply[64];
    char setParam[16];
    bool numericReply;
    CommandError commandError;
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetSettings();
        
        // Generate random valid sync threshold value (5-30)
        uint8_t thresholdValue = (uint8_t)(SUPERVISED_SYNC_THRESHOLD_MIN + 
            rand() % (SUPERVISED_SYNC_THRESHOLD_MAX - SUPERVISED_SYNC_THRESHOLD_MIN + 1));
        
        // Set sync threshold
        sprintf(setParam, "t%d", thresholdValue);
        processCommand(reply, "SG", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        // Get sync threshold and verify
        processCommand(reply, "SG", "t", &numericReply, &commandError);
        TEST_ASSERT_EQUAL(CE_NONE, commandError);
        
        int parsedThreshold = atoi(reply);
        TEST_ASSERT_EQUAL_MESSAGE(thresholdValue, parsedThreshold,
            "Sync threshold get should return the value that was set");
    }
}

/**
 * Property 11: Command Round-Trip for Invalid Values
 * 
 * For any invalid parameter value, the set command SHALL return an error
 * and the setting SHALL remain unchanged.
 * 
 * Feature: supervised-features, Property 11: Command Round-Trip
 * Validates: Requirements 7.1-7.14
 */
void test_property_command_roundtrip_invalid_values() {
    srand((unsigned int)time(NULL) + 3);
    
    char reply[64];
    char setParam[16];
    bool numericReply;
    CommandError commandError;
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetSettings();
        
        // Store original values
        uint8_t originalEast = getRaLimitEast();
        uint8_t originalWest = getRaLimitWest();
        uint8_t originalThreshold = getSyncThreshold();
        
        // Generate invalid RA limit value (0 or > 180)
        uint8_t invalidRaLimit;
        if (rand() % 2 == 0) {
            invalidRaLimit = 0;  // Below minimum
        } else {
            invalidRaLimit = (uint8_t)(181 + rand() % 75);  // Above maximum
        }
        
        // Try to set invalid east limit
        sprintf(setParam, "lE%d", invalidRaLimit);
        processCommand(reply, "SR", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL_MESSAGE(CE_PARAM_RANGE, commandError,
            "Invalid RA limit should return CE_PARAM_RANGE");
        TEST_ASSERT_EQUAL_MESSAGE(originalEast, getRaLimitEast(),
            "RA limit east should remain unchanged after invalid set");
        
        // Try to set invalid west limit
        sprintf(setParam, "lW%d", invalidRaLimit);
        processCommand(reply, "SR", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL_MESSAGE(CE_PARAM_RANGE, commandError,
            "Invalid RA limit should return CE_PARAM_RANGE");
        TEST_ASSERT_EQUAL_MESSAGE(originalWest, getRaLimitWest(),
            "RA limit west should remain unchanged after invalid set");
        
        // Generate invalid sync threshold value (< 5 or > 30)
        uint8_t invalidThreshold;
        if (rand() % 2 == 0) {
            invalidThreshold = (uint8_t)(rand() % SUPERVISED_SYNC_THRESHOLD_MIN);  // Below minimum
        } else {
            invalidThreshold = (uint8_t)(SUPERVISED_SYNC_THRESHOLD_MAX + 1 + rand() % 50);  // Above maximum
        }
        
        // Try to set invalid sync threshold
        sprintf(setParam, "t%d", invalidThreshold);
        processCommand(reply, "SG", setParam, &numericReply, &commandError);
        TEST_ASSERT_EQUAL_MESSAGE(CE_PARAM_RANGE, commandError,
            "Invalid sync threshold should return CE_PARAM_RANGE");
        TEST_ASSERT_EQUAL_MESSAGE(originalThreshold, getSyncThreshold(),
            "Sync threshold should remain unchanged after invalid set");
    }
}

/**
 * Unit test: Command boundary values
 */
void test_command_boundary_values() {
    char reply[64];
    char setParam[16];
    bool numericReply;
    CommandError commandError;
    
    resetSettings();
    
    // Test RA limit minimum (1)
    sprintf(setParam, "lE%d", SUPERVISED_RA_LIMIT_MIN);
    processCommand(reply, "SR", setParam, &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_NONE, commandError);
    TEST_ASSERT_EQUAL(SUPERVISED_RA_LIMIT_MIN, getRaLimitEast());
    
    // Test RA limit maximum (180)
    sprintf(setParam, "lE%d", SUPERVISED_RA_LIMIT_MAX);
    processCommand(reply, "SR", setParam, &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_NONE, commandError);
    TEST_ASSERT_EQUAL(SUPERVISED_RA_LIMIT_MAX, getRaLimitEast());
    
    // Test sync threshold minimum (5)
    sprintf(setParam, "t%d", SUPERVISED_SYNC_THRESHOLD_MIN);
    processCommand(reply, "SG", setParam, &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_NONE, commandError);
    TEST_ASSERT_EQUAL(SUPERVISED_SYNC_THRESHOLD_MIN, getSyncThreshold());
    
    // Test sync threshold maximum (30)
    sprintf(setParam, "t%d", SUPERVISED_SYNC_THRESHOLD_MAX);
    processCommand(reply, "SG", setParam, &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_NONE, commandError);
    TEST_ASSERT_EQUAL(SUPERVISED_SYNC_THRESHOLD_MAX, getSyncThreshold());
}

/**
 * Unit test: Unknown command handling
 */
void test_unknown_command_handling() {
    char reply[64];
    bool numericReply;
    CommandError commandError;
    
    resetSettings();
    
    // Test unknown subcommand for SH
    processCommand(reply, "SH", "x", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_CMD_UNKNOWN, commandError);
    
    // Test unknown subcommand for SR
    processCommand(reply, "SR", "x", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_CMD_UNKNOWN, commandError);
    
    // Test unknown subcommand for SG
    processCommand(reply, "SG", "x", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_CMD_UNKNOWN, commandError);
    
    // Test unknown subcommand for SM
    processCommand(reply, "SM", "x", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_CMD_UNKNOWN, commandError);
    
    // Test completely unknown command
    bool handled = processCommand(reply, "SX", "g", &numericReply, &commandError);
    TEST_ASSERT_FALSE(handled);
}

/**
 * Unit test: Invalid boolean parameter handling
 */
void test_invalid_boolean_parameter() {
    char reply[64];
    bool numericReply;
    CommandError commandError;
    
    resetSettings();
    bool originalHome = getHomeEnabled();
    
    // Test invalid boolean value (2)
    processCommand(reply, "SH", "s2", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_PARAM_RANGE, commandError);
    TEST_ASSERT_EQUAL(originalHome, getHomeEnabled());
    
    // Test invalid boolean value (9)
    processCommand(reply, "SR", "s9", &numericReply, &commandError);
    TEST_ASSERT_EQUAL(CE_PARAM_RANGE, commandError);
}

void setUp(void) {
    resetSettings();
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property-based tests for command round-trip (Property 11)
    RUN_TEST(test_property_command_roundtrip_boolean);
    RUN_TEST(test_property_command_roundtrip_ra_limits);
    RUN_TEST(test_property_command_roundtrip_sync_threshold);
    RUN_TEST(test_property_command_roundtrip_invalid_values);
    
    // Unit tests for boundary values and error handling
    RUN_TEST(test_command_boundary_values);
    RUN_TEST(test_unknown_command_handling);
    RUN_TEST(test_invalid_boolean_parameter);
    
    return UNITY_END();
}
