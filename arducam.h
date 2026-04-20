/**
 * @file    arducam.h
 * @brief   ArduCAM OV2640 — JPEG capture + TJpgDec decode + ST7796 display
 *
 *  Wiring on L4R5ZI-P:
 *    ArduCAM CS  → GPIOG PIN_12  (init HIGH)
 *    ArduCAM SPI → hspi2  (NSSPMode = PULSE_DISABLE, prescaler 32)
 *    OV2640 I2C  → hi2c2
 *    Display SPI → hspi1  (unchanged)
 */

#ifndef ARDUCAM_H
#define ARDUCAM_H

#include "main.h"
#include <stdint.h>

/* ─── Hardware config ────────────────────────────────────────────────────── */
#define ARDUCAM_CS_PORT         GPIOG
#define ARDUCAM_CS_PIN          GPIO_PIN_12

extern SPI_HandleTypeDef  hspi3;
extern I2C_HandleTypeDef  hi2c2;

#define OV2640_I2C_ADDR         (0x30 << 1)

/* ─── ArduCAM chip registers ─────────────────────────────────────────────── */
#define ARDUCHIP_TEST1          0x00
#define ARDUCHIP_MODE           0x03
#define ARDUCHIP_FIFO           0x04
#define ARDUCHIP_TRIG           0x41
#define BURST_FIFO_READ         0x3C

#define FIFO_FLUSH              0x01
#define FIFO_START              0x02
#define CAP_DONE_MASK           0x08

/* ─── JPEG buffer ────────────────────────────────────────────────────────── */
#define JPEG_BUF_MAX            (80U * 1024U)

/* ─── Return codes ───────────────────────────────────────────────────────── */
#define ARDUCAM_OK              0U
#define ARDUCAM_ERR_SPI         1U   /* SPI register test failed */
#define ARDUCAM_ERR_MODE        2U   /* ArduChip mode not confirmed */
#define ARDUCAM_ERR_TIMEOUT     3U   /* Capture done flag never set */
#define ARDUCAM_ERR_SIZE        4U   /* FIFO length invalid */
#define ARDUCAM_ERR_DECODE      5U   /* TJpgDec error */

/* ─── Sensor register struct ─────────────────────────────────────────────── */
struct sensor_reg {
    uint8_t reg;
    uint8_t val;
};

/* ─── Public API ─────────────────────────────────────────────────────────── */
void    arducam_spi_write(uint8_t address, uint8_t value);
uint8_t arducam_spi_read(uint8_t address);
void    wrSensorReg8_8(uint8_t reg, uint8_t data);
void    wrSensorRegs8_8(const struct sensor_reg reglist[]);

/**
 * @brief  Init OV2640 for JPEG 320x240. Returns ARDUCAM_OK or error code.
 *         Call once after SPI/I2C are ready.
 */
uint8_t OV2640_JPEG_Init(void);

/**
 * @brief  Capture JPEG, decode with TJpgDec, stream RGB565 to ST7796.
 * @return ARDUCAM_OK on success, error code on failure.
 */
uint8_t Arducam_CaptureDecodeDisplay(void);

#endif /* ARDUCAM_H */
