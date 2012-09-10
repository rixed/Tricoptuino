#ifndef ADXL345_H
#define ADXL345_H

//#define ADXL_ADDR 0x1DU
#define ADXL_ADDR 0x53U

// ADXL registers
#define ADXL_DEVID         0x00U // (RO, should return 0345)
#define ADXL_TAP_THRESH    0x1DU
#define ADXL_OFSX          0x1EU
#define ADXL_OFSY          0x1FU
#define ADXL_OFSZ          0x20U
#define ADXL_TAP_DUR       0x21U
#define ADXL_TAP_LATENT    0x22U
#define ADXL_TAP_WINDOW    0x23U
#define ADXL_ACT_THRESH    0x24U
#define ADXL_INACT_THRESH  0x25U
#define ADXL_INACT_TIME    0x26U
#define ADXL_ACT_INACT_CTL 0x27U
#define ADXL_FF_THRESH     0x28U
#define ADXL_FF_TIME       0x29U
#define ADXL_TAP_AXES      0x2AU
#define ADXL_TAP_STATUS    0x2BU
#define ADXL_BW_RATE       0x2CU
#define ADXL_POWER_CTL     0x2DU
#define ADXL_INT_ENABLE    0x2EU
#define ADXL_INT_MAP       0x2FU
#define ADXL_INT_SOURCE    0x30U
#define ADXL_DATA_FORMAT   0x31U
#define ADXL_DATA_X0       0x32U
#define ADXL_DATA_X1       0x33U
#define ADXL_DATA_Y0       0x34U
#define ADXL_DATA_Y1       0x35U
#define ADXL_DATA_Z0       0x36U
#define ADXL_DATA_Z1       0x37U
#define ADXL_FIFO_CTL      0x38U
#define ADXL_FIFO_STATUS   0x39U

#define ADXL_MEASURE_BIT BIT(3)

uint8_t adxl_read(uint8_t reg);
void adxl_write(uint8_t reg, uint8_t v, uint8_t wait);
void adxl_xyz(uint16_t xyz[3]);
void adxl_init(void);

#endif
