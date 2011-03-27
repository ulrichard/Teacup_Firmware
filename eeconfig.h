#ifndef	_EECONFIG_H
#define	_EECONFIG_H

#include	<stdint.h>

typedef struct {
	uint32_t	steps_per_mm_x;
	uint32_t	steps_per_mm_y;
	uint32_t	steps_per_mm_z;
	uint32_t	steps_per_mm_e;

	uint32_t	size_x;
	uint32_t	size_y;
	uint32_t	size_z;

	uint32_t	max_speed_x;
	uint32_t	max_speed_y;
	uint32_t	max_speed_z;
	uint32_t	max_speed_e;

	uint16_t	max_temp_e;
	uint16_t	max_temp_b;
	uint16_t	max_temp_r;

	uint32_t	min_endstop_pos_z;

	uint16_t	temp_hysteresis;
	uint16_t	temp_residency;

	uint32_t	baud;

	struct {
		uint16_t	adc_value;
		uint16_t	temperature;
	} temptable[20];

	uint16_t crc;
} eeconfig_struct;

extern eeconfig_struct eeconfig;

void eeconfig_init(void);
void eeconfig_save(void);

#endif	/* _EECONFIG_H */
