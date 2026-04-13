/**
 * =============================================================================
 *  Arducam 2MP Plus (OV2640) Driver for NUCLEO-L4R5ZI-P
 *  -------------------------------------------------------------------------
 *  Wiring:
 *    CAM CS   → PG12  (GPIO Output)
 *    CAM MOSI → PG11  (SPI3)
 *    CAM MISO → PG10  (SPI3)
 *    CAM SCK  → PG9   (SPI3)
 *    CAM SDA  → PF0   (I2C2)
 *    CAM SCL  → PF1   (I2C2)
 *    CAM VCC  → 3.3V
 *    CAM GND  → GND
 *
 *  SPI3: Mode 0, max 8 MHz
 *  I2C2: Standard or Fast mode, OV2640 slave addr 0x60 (write) / 0x61 (read)
 * =============================================================================
 */

#ifndef ARDUCAM_H
#define ARDUCAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── OV2640 I2C address (7-bit for HAL) ───────────────────────────────────── */
#define OV2640_I2C_ADDR      0x60   /* 7-bit: 0x30, but HAL uses 8-bit */

/* ── OV2640 chip ID registers ─────────────────────────────────────────────── */
#define OV2640_CHIPID_HIGH   0x0A   /* Should read 0x26 */
#define OV2640_CHIPID_LOW    0x0B   /* Should read 0x41 or 0x42 */

/* ── ArduChip SPI register map ────────────────────────────────────────────── */
/* SPI protocol: bit[7]=1 for write, bit[7]=0 for read.  bit[6:0] = address.
 * So to write reg 0x04, send 0x84 then data byte.
 * To read  reg 0x04, send 0x04 then dummy byte, read MISO. */

#define ARDUCHIP_TEST        0x00   /* R/W test register                     */
#define ARDUCHIP_MODE        0x02   /* R/W mode register                     */
#define ARDUCHIP_TIM         0x03   /* R/W timing control                    */
#define ARDUCHIP_FIFO_CR     0x04   /* W   FIFO control register             */
#define ARDUCHIP_GPIO        0x06   /* R/W GPIO / power control              */
#define ARDUCHIP_REV         0x40   /* R   ArduChip version (0x40 = 2MP)     */
#define ARDUCHIP_TRIG        0x41   /* R   Trigger / status register         */
#define ARDUCHIP_FIFO_SIZE1  0x42   /* R   FIFO size [7:0]                   */
#define ARDUCHIP_FIFO_SIZE2  0x43   /* R   FIFO size [15:8]                  */
#define ARDUCHIP_FIFO_SIZE3  0x44   /* R   FIFO size [18:16]                 */

/* Single FIFO read (one byte at a time). */
#define ARDUCHIP_FIFO_SINGLE 0x3D
/* Burst FIFO read (continuous clocking). */
#define ARDUCHIP_FIFO_BURST  0x3C

/* ── FIFO control bits (write to ARDUCHIP_FIFO_CR = 0x84) ────────────────── */
#define FIFO_CLEAR_MASK      0x01   /* Clear FIFO write done flag            */
#define FIFO_START_MASK      0x02   /* Start capture                         */
#define FIFO_RDPTR_RST_MASK  0x10   /* Reset FIFO read pointer               */
#define FIFO_WRPTR_RST_MASK  0x20   /* Reset FIFO write pointer              */

/* ── Trigger / status bits (read from ARDUCHIP_TRIG = 0x41) ──────────────── */
#define CAP_DONE_MASK        0x08   /* Capture done flag (bit 3)             */
#define VSYNC_MASK           0x01   /* VSYNC status (bit 0)                  */

/* ── Mode register values ─────────────────────────────────────────────────── */
#define MCU2LCD_MODE         0x00
#define CAM2LCD_MODE         0x01
#define LCD2MCU_MODE         0x02

/* ── Image format enum ────────────────────────────────────────────────────── */
typedef enum {
    CAM_FMT_BMP  = 0,
    CAM_FMT_JPEG = 1
} cam_format_t;

/* ── JPEG resolution enum ─────────────────────────────────────────────────── */
typedef enum {
    CAM_160x120   = 0,
    CAM_176x144   = 1,
    CAM_320x240   = 2,
    CAM_352x288   = 3,
    CAM_640x480   = 4,
    CAM_800x600   = 5,
    CAM_1024x768  = 6,
    CAM_1280x1024 = 7,
    CAM_1600x1200 = 8
} cam_jpeg_size_t;

/* ── Status codes ─────────────────────────────────────────────────────────── */
typedef enum {
    CAM_OK             = 0,
    CAM_ERR_SPI        = 1,   /* SPI test register read-back failed       */
    CAM_ERR_I2C        = 2,   /* I2C communication failed                 */
    CAM_ERR_SENSOR_ID  = 3,   /* OV2640 chip ID mismatch                  */
    CAM_ERR_TIMEOUT    = 4    /* Capture timeout                          */
} cam_status_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  PUBLIC API
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Full init: verifies SPI link, checks OV2640 chip ID, loads JPEG defaults.
 * Call after HAL_Init, MX_SPI3_Init, MX_I2C2_Init, MX_GPIO_Init. */
cam_status_t cam_init(void);

/* Set output format (JPEG or BMP). */
void cam_set_format(cam_format_t fmt);

/* Set JPEG output resolution. */
void cam_set_jpeg_size(cam_jpeg_size_t size);

/* Capture a single frame into the ArduChip FIFO.
 * Returns CAM_OK on success, CAM_ERR_TIMEOUT if capture doesn't complete. */
cam_status_t cam_capture(void);

/* Get the number of bytes in the FIFO after a capture. */
uint32_t cam_fifo_length(void);

/* Read `len` bytes from the FIFO into `buf` using burst read.
 * Call after cam_capture() succeeds.  You can call this multiple
 * times if you want to read in chunks. */
void cam_fifo_read(uint8_t *buf, uint32_t len);

/* Read the ArduChip version register (should return 0x40 for 2MP). */
uint8_t cam_read_chip_version(void);

/* Low-level: write one byte to an ArduChip register. */
void cam_write_reg(uint8_t addr, uint8_t data);

/* Low-level: read one byte from an ArduChip register. */
uint8_t cam_read_reg(uint8_t addr);

/* Low-level: write one OV2640 sensor register via I2C. */
void cam_sensor_write(uint8_t reg, uint8_t val);

/* Low-level: read one OV2640 sensor register via I2C. */
uint8_t cam_sensor_read(uint8_t reg);

#ifdef __cplusplus
}
#endif
#endif /* ARDUCAM_H */
