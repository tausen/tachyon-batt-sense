#include <EEPROM.h>
#include "common.h"
#include "eeprom_profile.h"

// Persist profile locations (assumed all int16_t's)
#define EEPROM_ADDR_CRC 0
#define EEPROM_ADDR_RT (EEPROM_ADDR_CRC+sizeof(unsigned long))
#define EEPROM_ADDR_FT (EEPROM_ADDR_RT+sizeof(int16_t))
#define EEPROM_ADDR_LR (EEPROM_ADDR_FT+sizeof(int16_t))
#define EEPROM_ADDR_LF (EEPROM_ADDR_LR+sizeof(int16_t))
#define EEPROM_ADDR_END (EEPROM_ADDR_LF+sizeof(int16_t))

static unsigned long eeprom_crc(void);

void eeprom_save(profile_t *profile) {
    // Save profile to EEPROM
    EEPROM.put(EEPROM_ADDR_RT, profile->det_rising_threshold);
    EEPROM.put(EEPROM_ADDR_FT, profile->det_falling_threshold);
    EEPROM.put(EEPROM_ADDR_LR, profile->lookback_rising);
    EEPROM.put(EEPROM_ADDR_LF, profile->lookback_falling);
    // Update CRC
    unsigned long crc = eeprom_crc();
    EEPROM.put(EEPROM_ADDR_CRC, crc);
}

void eeprom_load(profile_t *profile) {
    // Load profile from EEPROM
    EEPROM.get(EEPROM_ADDR_RT, profile->det_rising_threshold);
    EEPROM.get(EEPROM_ADDR_FT, profile->det_falling_threshold);
    EEPROM.get(EEPROM_ADDR_LR, profile->lookback_rising);
    EEPROM.get(EEPROM_ADDR_LF, profile->lookback_falling);
}

void eeprom_clear() {
    // Clear CRC field in EEPROM, invalidating the profile
    for (unsigned int i = 0; i < sizeof(unsigned long); i++)
        EEPROM.write(EEPROM_ADDR_CRC+i, 0);
}

bool eeprom_check_crc() {
    // Check whether stored CRC is valid
    unsigned long crc = eeprom_crc();
    unsigned long crc_rd;
    EEPROM.get(EEPROM_ADDR_CRC, crc_rd);

    return crc_rd == crc;
}

// https://www.arduino.cc/en/Tutorial/EEPROMCrc
static unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (unsigned int index = (EEPROM_ADDR_CRC+sizeof(unsigned long)); index < EEPROM_ADDR_END; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}
