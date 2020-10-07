#ifndef DRV_AK_REGS_H
#define DRV_AK_REGS_H

#define WIA                 0x00 // aka WHO_AM_I
#define INFO                0x01
#define ST1                 0x02
#define HXL                 0x03
#define HXH                 0x04
#define HYL                 0x05
#define HYH                 0x06
#define HZL                 0x07
#define HZH                 0x08
#define ST2                 0x09

#define ASAX                0x10
#define ASAY                0x11
#define ASAZ                0x12

#define CNTL1               0x0A
#define CNTL2               0x0B
#define ASTC                0x0C
#define TS1                 0x0D
#define TS2                 0x0E
#define I2CDIS              0x0F

#define AK8963_SENSITIVITY  0.15
#define AK8963_BITMASK_DATA_RDY  0x01
#define AK8963_BITMASK_HOFL 0x08

#endif // DRV_AK_REGS_H
