// Save profile to EEPROM and update CRC
void eeprom_save(profile_t *profile);
// Load profile from EEPROM - warning: not checking CRC
void eeprom_load(profile_t *profile);
// Clear CRC field in EEPROM, invalidating the profile
void eeprom_clear(void);
// Check whether stored CRC is valid
bool eeprom_check_crc(void);
