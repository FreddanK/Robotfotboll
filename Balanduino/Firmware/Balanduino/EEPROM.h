#ifndef _eeprom_h_
#define _eeprom_h_

/* Firmware Version Information */
constexpr char *version = "1.1.0";
constexpr uint8_t eepromVersion = 3; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

bool checkInitializationFlags();
void readEEPROMValues();
void updateConfig();
void restoreEEPROMValues();

#endif

