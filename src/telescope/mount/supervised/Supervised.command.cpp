//--------------------------------------------------------------------------------------------------
// telescope mount control - supervised features command interface
//
// Feature: supervised-features
// Implements: Requirements 7.1-7.14
//
// Command Interface:
// :SHg#      Get supervised home status (returns 0=off, 1=on)
// :SHs[n]#   Set supervised home status (0=off, 1=on)
// :SRg#      Get supervised RA limit status
// :SRs[n]#   Set supervised RA limit status
// :SRl#      Get RA limits (returns "east|west" in degrees)
// :SRlE[n]#  Set east RA limit (degrees)
// :SRlW[n]#  Set west RA limit (degrees)
// :SGg#      Get supervised GOTO status
// :SGs[n]#   Set supervised GOTO status
// :SGt#      Get GOTO sync threshold (degrees)
// :SGt[n]#   Set GOTO sync threshold (5-30 degrees)
// :SMg#      Get power-off memory status
// :SMs[n]#   Set power-off memory status
// :SDp#      Get diagnostic info (true vs virtual position difference)

#include "Supervised.h"

#ifdef MOUNT_PRESENT
#ifdef SUPERVISED_FEATURES

#include "../../../lib/tasks/OnTask.h"
#include "../Mount.h"

bool Supervised::command(char* reply, char* command, char* parameter,
                         bool* supressFrame, bool* numericReply, CommandError* commandError) {
  UNUSED(supressFrame);
  
  // All supervised commands start with 'S'
  if (command[0] != 'S') return false;
  
  // :SH - Supervised Home commands
  if (command[1] == 'H') {
    // :SHg# - Get supervised home status
    // Returns: 0=off, 1=on
    if (parameter[0] == 'g' && parameter[1] == 0) {
      sprintf(reply, "%d", homeEnabled() ? 1 : 0);
      *numericReply = false;
      return true;
    }
    
    // :SHs[n]# - Set supervised home status (0=off, 1=on)
    // Returns: 0 on failure, 1 on success
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
    // Returns: 0=off, 1=on
    if (parameter[0] == 'g' && parameter[1] == 0) {
      sprintf(reply, "%d", raLimitEnabled() ? 1 : 0);
      *numericReply = false;
      return true;
    }
    
    // :SRs[n]# - Set supervised RA limit status (0=off, 1=on)
    // Returns: 0 on failure, 1 on success
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
    
    // :SRl# - Get RA limits (returns "east|west" in degrees)
    if (parameter[0] == 'l' && parameter[1] == 0) {
      sprintf(reply, "%d|%d", getRaLimitEast(), getRaLimitWest());
      *numericReply = false;
      return true;
    }
    
    // :SRlE[n]# - Set east RA limit (degrees, 1-180)
    // Returns: 0 on failure, 1 on success
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
    
    // :SRlW[n]# - Set west RA limit (degrees, 1-180)
    // Returns: 0 on failure, 1 on success
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
    // Returns: 0=off, 1=on
    if (parameter[0] == 'g' && parameter[1] == 0) {
      sprintf(reply, "%d", gotoEnabled() ? 1 : 0);
      *numericReply = false;
      return true;
    }
    
    // :SGs[n]# - Set supervised GOTO status (0=off, 1=on)
    // Returns: 0 on failure, 1 on success
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
    
    // :SGt# - Get GOTO sync threshold (degrees)
    // Returns: n (threshold in degrees)
    if (parameter[0] == 't' && parameter[1] == 0) {
      sprintf(reply, "%d", getSyncThreshold());
      *numericReply = false;
      return true;
    }
    
    // :SGt[n]# - Set GOTO sync threshold (5-30 degrees)
    // Returns: 0 on failure, 1 on success
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
    // Returns: 0=off, 1=on
    if (parameter[0] == 'g' && parameter[1] == 0) {
      sprintf(reply, "%d", memoryEnabled() ? 1 : 0);
      *numericReply = false;
      return true;
    }
    
    // :SMs[n]# - Set power-off memory status (0=off, 1=on)
    // Returns: 0 on failure, 1 on success
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
  
  // :SD - Supervised Diagnostic commands
  if (command[1] == 'D') {
    // :SDp# - Get diagnostic info (true vs virtual position difference)
    // Returns: difference in degrees as floating point
    if (parameter[0] == 'p' && parameter[1] == 0) {
      float distance = getDistanceBetweenTruePosAndVirtualPos();
      // Convert from radians to degrees for display
      float distanceDeg = radToDeg(distance);
      sprintf(reply, "%0.4f", distanceDeg);
      *numericReply = false;
      return true;
    }
    
    *commandError = CE_CMD_UNKNOWN;
    return true;
  }
  
  // Unknown supervised command
  return false;
}

#endif // SUPERVISED_FEATURES
#endif // MOUNT_PRESENT
