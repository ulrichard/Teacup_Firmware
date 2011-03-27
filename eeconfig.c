#include	"eeconfig.h"

#include	<avr/eeprom.h>

#include	"crc.h"
#include "config.h"
#include	"clock.h"

/// in-memory configuration data structure
eeconfig_struct eeconfig;

/// in-eeprom configuration data structure
eeconfig_struct EEMEM EE_config;

void eeconfig_init() {
	uint16_t mycrc;
	eeprom_read_block(&eeconfig, &EE_config, sizeof(eeconfig_struct));
	mycrc = crc_block(&eeconfig, sizeof(eeconfig_struct) - sizeof(uint16_t));
	if (mycrc != eeconfig.crc) {
		// set sane defaults
		eeconfig.steps_per_mm_x = STEPS_PER_MM_X;
		eeconfig.steps_per_mm_y = STEPS_PER_MM_Y;
		eeconfig.steps_per_mm_z = STEPS_PER_MM_Z;
		eeconfig.steps_per_mm_e = STEPS_PER_MM_E;

		eeconfig.size_x = X_MAX;
		eeconfig.size_y = Y_MAX;
		eeconfig.size_z = Z_MAX;

		eeconfig.max_speed_x = MAXIMUM_FEEDRATE_X;
		eeconfig.max_speed_y = MAXIMUM_FEEDRATE_Y;
		eeconfig.max_speed_z = MAXIMUM_FEEDRATE_Z;
		eeconfig.max_speed_e = MAXIMUM_FEEDRATE_E;

		eeconfig.max_temp_e = 1000;
		eeconfig.max_temp_b = 480;
		eeconfig.max_temp_r = 240;

		eeconfig.min_endstop_pos_z = Z_MIN;

		eeconfig.temp_hysteresis = TEMP_HYSTERESIS;
		eeconfig.temp_residency = TEMP_RESIDENCY_TIME;

		eeconfig.baud = BAUD;
	}
}

void eeconfig_save() {
	eeconfig.crc = crc_block(&eeconfig, sizeof(eeconfig_struct) - sizeof(uint16_t));
	eeprom_write_block(&eeconfig, &EE_config, sizeof(eeconfig_struct));
	do {
		clock_poll();
	} while (eeprom_is_ready() == 0);
}
