#ifndef	_EECONFIG_H
#define	_EECONFIG_H

#include	<stdint.h>

/// this doubles as both an in-eeprom and in-memory storage struct for configuration settings
typedef struct {
	uint32_t	steps_per_mm_x; ///< steps per mm. critically important for accurate prints
	uint32_t	steps_per_mm_y; ///< steps per mm. critically important for accurate prints
	uint32_t	steps_per_mm_z; ///< steps per mm. critically important for accurate prints
	uint32_t	steps_per_mm_e; ///< steps per mm. critically important for accurate prints

	uint32_t	size_x; ///< build volume. don't allow axes to move outside an area of this size
	uint32_t	size_y; ///< build volume. don't allow axes to move outside an area of this size
	uint32_t	size_z; ///< build volume. don't allow axes to move outside an area of this size

	uint32_t	max_speed_x; ///< axis speed limit. Any move which requires this axis to go above this speed will have its specified speed reduced, preserving geometry.
	uint32_t	max_speed_y; ///< axis speed limit. Any move which requires this axis to go above this speed will have its specified speed reduced, preserving geometry.
	uint32_t	max_speed_z; ///< axis speed limit. Any move which requires this axis to go above this speed will have its specified speed reduced, preserving geometry.
	uint32_t	max_speed_e; ///< axis speed limit. Any move which requires this axis to go above this speed will have its specified speed reduced, preserving geometry.

	uint16_t	max_temp_e;	///< do not allow temperature to go above this amount even if host software requests it, assume host software has gone crazy.
	uint16_t	max_temp_b;	///< do not allow temperature to go above this amount even if host software requests it, assume host software has gone crazy.
	uint16_t	max_temp_r;	///< do not allow temperature to go above this amount even if host software requests it, assume host software has gone crazy.

	uint32_t	min_endstop_pos_z; ///< this is the Z position where the endstop is encountered. This helps prevent the head crashing into the bed while homing. To use, set this value to 3mm or so, then adjust your Z endstop flag so that it trips when your extruder nozzle is 3mm from your bed.

	uint16_t	temp_hysteresis; ///< temperature has to be within target +/- this amount for M109 and friends to continue
	uint16_t	temp_residency; ///< temperature has to be near target for this long for M109 and friends to continue

	uint32_t	baud; ///< serial baud rate to communicate at. If changed, does not take effect until reset. \warning if a bad value is progammed, the only way to recover is to re-flash your eeprom from the host.

	struct {
		uint16_t	adc_value;
		uint16_t	temperature;
	} temptable[20]; ///< the temperature lookup table for linear interpolation of ADC readings -> temperatures

	uint16_t crc; ///< data integrity check. If crc doesn't match data, use defaults instead.
} eeconfig_struct;

extern eeconfig_struct eeconfig;

/// read settings from eeprom
void eeconfig_init(void);

/// save current settings to eeprom
void eeconfig_save(void);

#endif	/* _EECONFIG_H */
