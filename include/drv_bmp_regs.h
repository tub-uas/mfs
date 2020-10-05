#ifndef DRV_BMP280_REGS_H
#define DRV_BMP280_REGS_H

#define BMP280_TEMP_HB     0xFA
#define BMP280_TEMP_LB     0xFB
#define BMP280_TEMP_XLB    0xFC

#define BMP280_PRESS_HB    0xF7
#define BMP280_PRESS_LB    0xF8
#define BMP280_PRESS_XLB   0xF9

#define BMP280_CONFIG      0xF5
#define BMP280_CTRL_MEAS   0xF4
#define BMP280_STAT        0xF3
#define BMP280_RESET       0xE0
#define BMP280_WHO_AM_I    0xD0

#define BMP280_CALIBBASE   0x88
#define BMP280_CALIB00     (BMP280_CALIBBASE+0)
#define BMP280_CALIB01     (BMP280_CALIBBASE+1)
#define BMP280_CALIB02     (BMP280_CALIBBASE+2)
#define BMP280_CALIB03     (BMP280_CALIBBASE+3)
#define BMP280_CALIB04     (BMP280_CALIBBASE+4)
#define BMP280_CALIB05     (BMP280_CALIBBASE+5)
#define BMP280_CALIB06     (BMP280_CALIBBASE+6)
#define BMP280_CALIB07     (BMP280_CALIBBASE+7)
#define BMP280_CALIB08     (BMP280_CALIBBASE+8)
#define BMP280_CALIB09     (BMP280_CALIBBASE+9)
#define BMP280_CALIB10     (BMP280_CALIBBASE+10)
#define BMP280_CALIB11     (BMP280_CALIBBASE+11)
#define BMP280_CALIB12     (BMP280_CALIBBASE+12)
#define BMP280_CALIB13     (BMP280_CALIBBASE+13)
#define BMP280_CALIB14     (BMP280_CALIBBASE+14)
#define BMP280_CALIB15     (BMP280_CALIBBASE+15)
#define BMP280_CALIB16     (BMP280_CALIBBASE+16)
#define BMP280_CALIB17     (BMP280_CALIBBASE+17)
#define BMP280_CALIB18     (BMP280_CALIBBASE+18)
#define BMP280_CALIB19     (BMP280_CALIBBASE+19)
#define BMP280_CALIB20     (BMP280_CALIBBASE+20)
#define BMP280_CALIB21     (BMP280_CALIBBASE+21)
#define BMP280_CALIB22     (BMP280_CALIBBASE+22)
#define BMP280_CALIB23     (BMP280_CALIBBASE+23)
#define BMP280_CALIB24     (BMP280_CALIBBASE+24)
#define BMP280_CALIB25     (BMP280_CALIBBASE+25)

#define BMP280_REGT1       (BMP280_CALIB00)
#define BMP280_REGT2       (BMP280_CALIB02)
#define BMP280_REGT3       (BMP280_CALIB04)
#define BMP280_REGP1       (BMP280_CALIB06)
#define BMP280_REGP2       (BMP280_CALIB08)
#define BMP280_REGP3       (BMP280_CALIB10)
#define BMP280_REGP4       (BMP280_CALIB12)
#define BMP280_REGP5       (BMP280_CALIB14)
#define BMP280_REGP6       (BMP280_CALIB16)
#define BMP280_REGP7       (BMP280_CALIB18)
#define BMP280_REGP8       (BMP280_CALIB20)
#define BMP280_REGP9       (BMP280_CALIB22)

#define SIZECOEFFREG    2
#define SIZETEMPREG     3
#define SIZEPRESSREG    3

#endif // DRV_BMP280_REGS_H
