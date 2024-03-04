#ifndef _ACT_CONFS_H_
#define _ACT_CONFS_H_

#include <stdint.h>

const uint8_t act_hiz_conf[] = {
	0x42, // 0
	0x00, // 1
	0x00, // 2
	0x00, // 3
	0x00, // 4
	0x00, // 5
	0x00, // 6
	0x00, // 7
	0x00, // 8
	0xA0, // 9
	0x12, // 10 A
	0x90, // 11 B
	0x40, // 12 C
	0x4C, // 13 D
	0x00, // 14 E
	0x00, // 15 F
	0x00, // 16 10
	0xF2, // 11
	0xF8, // 12
	0xA0, // Must be set in the driver based on USB requirements
	0x00, // Must be set in the driver based on USB requirements
	0x7F, // 21 15
	0x80, // 22 16
	0x7F, // Here output current is set to 100%, must be changed in application
	0x00, // Set to 100%
	0x00, // 25 19
	0x0A, // VBAT_LOW = 6V
	0x04, // Default
	0x09, // 28 1C
	0x81, // 29 1D
	0x00, // 30 1E
	0x00, // 31 1F
	0x00  // 32 20
};

const uint8_t act_otg_conf[] = {
	0x02, // 0
	0x10, // 1
	0x00, // 2
	0x00, // 3
	0x00, // 4
	0x00, // 5
	0x00, // 6
	0x00, // 7
	0x00, // 8
	0xA0, // 9
	0x12, // 10 A
	0x90, // 11 B
	0x40, // 12 C
	0x00, // 13 D
	0x60, // 14 E
	0x00, // 15 F
	0xC0, // 16 10
	0x01, // 17 11
	0x54, // 18 12
	0xA0, // Must be set in the driver based on USB requirements
	0x00, // Must be set in the driver based on USB requirements
	0x7F, // 21 15
	0x80, // 22 16
	0x7F, // Here output current is set to 100%, must be changed in application
	0x7F, // Set to 100%
	0x00, // 25 19
	0x0A, // VBAT_LOW = 6V
	0x04, // Default
	0x09, // 28 1C
	0x81, // 29 1D
	0x00, // 30 1E
	0x00, // 31 1F
	0x00  // 32 20
};

const uint8_t act_chg_conf[] = {
	0x42, // 0
	0x10, // 1
	0x00, // 2
	0x00, // 3
	0x00, // 4
	0x00, // 5
	0x00, // 6
	0x00, // 7
	0x00, // 8
	0xA0, // 9
	0x12, // A
	0xB0, // B
	0x40, // C
	0xE8, // D
	0x00, // E
	0x00, // F
	0x00, // 10
	0xF2, // 11
	0xF8, // 12
	0xA0, // Must be set in the driver based on USB requirements
	0x00, // Must be set in the driver based on USB requirements
	0x7F, // 15
	0x80, // 16
	0x7F, // Here output current is set to 100%, must be changed in application
	0x0A, // Set to 100%
	0x00, // 19
	0x28, // VBAT_LOW = 9V
	0x04, // Default
	0x09, // 1C
	0x81, // 1D
	0x00, // 1E
	0x00, // 1F
	0x00 // 20
};




#endif