#ifndef DRV_NMEA_CONFIG_H
#define DRV_NMEA_CONFIG_H

/* Configuration strings needed to put GPS into right mode */
char nmea_config_version[] =
	{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
char nmea_config_rate[] =
	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
char nmea_config_port[] =
	{0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00,
	 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB0, 0x7E};

#endif // DRV_NMEA_CONFIG_H
