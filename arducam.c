/**
 * @file    arducam.c
 * @brief   ArduCAM OV2640 — JPEG capture + TJpgDec decode + ST7796 stream
 *          Directly based on teammate's working STM32 capture code.
 *          Adapted: hspi1→hspi3, hi2c1→hi2c2, GPIOD_14→GPIOG_12
 */

#include "arducam.h"
#include "st7796.h"
#include "ov2640_regs.h"
#include "tjpgd.h"
#include <string.h>

/* ─── Buffers ────────────────────────────────────────────────────────────── */
static uint8_t  s_jpeg_buf[JPEG_BUF_MAX];
static uint32_t s_jpeg_len  = 0;
static uint32_t s_jpeg_start = 0;   /* offset of 0xFF 0xD8 SOI in buffer */

#define TJPGD_WORK_SZ  3100U
static uint8_t  s_tjpgd_work[TJPGD_WORK_SZ];

/* ─── TJpgDec context ────────────────────────────────────────────────────── */
typedef struct {
    uint32_t read_pos;
    uint16_t disp_x;
    uint16_t disp_y;
} JpegCtx;

/* ─────────────────────────────────────────────────────────────────────────────
 * SPI / I2C LOW-LEVEL  (mirrors teammate's exact functions)
 * ───────────────────────────────────────────────────────────────────────────── */
void arducam_spi_write(uint8_t address, uint8_t value)
{
    uint8_t data[2] = { address | 0x80, value };
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, data, 2, 100);
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_SET);
}

uint8_t arducam_spi_read(uint8_t address)
{
    uint8_t addr  = address & 0x7F;
    uint8_t value = 0;
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &addr,  1, 100);
    HAL_SPI_Receive (&hspi3, &value, 1, 100);
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_SET);
    return value;
}

void wrSensorReg8_8(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c2, OV2640_I2C_ADDR,
                      reg, I2C_MEMADD_SIZE_8BIT,
                      &data, 1, 100);
    HAL_Delay(1);
}

void wrSensorRegs8_8(const struct sensor_reg reglist[])
{
    for (int i = 0; ; i++) {
        if (reglist[i].reg == 0xFF && reglist[i].val == 0xFF) break;
        wrSensorReg8_8(reglist[i].reg, reglist[i].val);
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
 * JPEG SENSOR INIT
 * Exact mirror of teammate's OV2640_JPEG_Init — same register sequence,
 * same SPI test, just using hspi3/hi2c2 instead of hspi1/hi2c1
 * ───────────────────────────────────────────────────────────────────────────── */
uint8_t OV2640_JPEG_Init(void)
{
    /* 1. Soft-reset ArduCAM chip (matches teammate's write_reg(0x07, 0x80)) */
    arducam_spi_write(0x07, 0x80);
    HAL_Delay(100);
    arducam_spi_write(0x07, 0x00);
    HAL_Delay(100);

    /* 2. SPI communication test — write 0x55 to TEST1, read it back.
          If this fails, SPI is not working and we return early. */
    arducam_spi_write(ARDUCHIP_TEST1, 0x55);
    uint8_t test_val = arducam_spi_read(ARDUCHIP_TEST1);
    if (test_val != 0x55) {
        return ARDUCAM_ERR_SPI;   /* SPI not working */
    }

    /* 3. Reset OV2640 sensor */
    wrSensorReg8_8(0xFF, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    HAL_Delay(100);

    /* 4. Same three-table sequence as teammate's OV2640_JPEG_Init */
    wrSensorRegs8_8(OV2640_JPEG_INIT);
    wrSensorRegs8_8(OV2640_YUV422);
    wrSensorRegs8_8(OV2640_JPEG);

    /* 5. VSYNC polarity fix */
    wrSensorReg8_8(0xFF, 0x01);
    wrSensorReg8_8(0x15, 0x00);

    /* 6. 320x240 resolution */
    wrSensorRegs8_8(OV2640_320x240_JPEG);

    /* 7. Set ArduChip to JPEG mode (reg 0x03 = 0x00) */
    arducam_spi_write(ARDUCHIP_MODE, 0x00);

    /* 8. Verify mode switch */
    uint8_t mode = arducam_spi_read(ARDUCHIP_MODE);
    if (mode != 0x00) {
        return ARDUCAM_ERR_MODE;
    }

    return ARDUCAM_OK;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * TJPGDEC CALLBACKS
 * ───────────────────────────────────────────────────────────────────────────── */
static unsigned int _jpeg_in(JDEC *jd, uint8_t *buf, unsigned int nbytes)
{
    JpegCtx *ctx = (JpegCtx *)jd->device;
    /* Read from s_jpeg_buf starting at s_jpeg_start */
    uint32_t offset = s_jpeg_start + ctx->read_pos;
    uint32_t avail  = (s_jpeg_len > offset) ? (s_jpeg_len - offset) : 0U;
    if ((uint32_t)nbytes > avail) nbytes = (unsigned int)avail;
    if (buf) memcpy(buf, s_jpeg_buf + offset, nbytes);
    ctx->read_pos += nbytes;
    return nbytes;
}

static int _jpeg_out(JDEC *jd, void *bitmap, JRECT *rect)
{
    JpegCtx  *ctx = (JpegCtx *)jd->device;
    uint16_t  w   = (uint16_t)(rect->right  - rect->left + 1U);
    uint16_t  h   = (uint16_t)(rect->bottom - rect->top  + 1U);
    uint16_t  x0  = ctx->disp_x + rect->left;
    uint16_t  y0  = ctx->disp_y + rect->top;
    uint16_t  x1  = ctx->disp_x + rect->right;
    uint16_t  y1  = ctx->disp_y + rect->bottom;
    uint32_t  npx = (uint32_t)w * h;

    /* TJpgDec RGB565 output is native-endian (little-endian on ARM).
       ST7796 expects big-endian (high byte first).
       Swap bytes in-place before transmitting. */
    uint8_t *px = (uint8_t *)bitmap;
    for (uint32_t i = 0; i < npx * 2U; i += 2U) {
        uint8_t tmp = px[i];
        px[i]       = px[i + 1U];
        px[i + 1U]  = tmp;
    }

    /* Write this MCU block to the display */
    ST7796_SetAddrWindow(x0, y0, x1, y1);
    HAL_GPIO_WritePin(ST7796_DC_PORT, ST7796_DC_PIN, GPIO_PIN_SET);
    ST7796_CS_Low();
    HAL_SPI_Transmit(&hspi1, (uint8_t *)bitmap, (uint16_t)(npx * 2U), 500);
    ST7796_CS_High();

    return 1;  /* continue */
}

/* ─────────────────────────────────────────────────────────────────────────────
 * CAPTURE → DECODE → DISPLAY
 * Mirrors teammate's Capture_JPEG flow exactly, then decodes instead of printing
 * ───────────────────────────────────────────────────────────────────────────── */
uint8_t Arducam_CaptureDecodeDisplay(void)
{
    /* ── 1. Trigger capture — same as teammate's flush/clear/start ── */
    arducam_spi_write(ARDUCHIP_FIFO, FIFO_FLUSH);
    arducam_spi_write(ARDUCHIP_FIFO, FIFO_START);

    /* ── 2. Wait for capture done — 3-second HAL_GetTick timeout ── */
    uint32_t deadline = HAL_GetTick() + 3000U;
    while (!(arducam_spi_read(ARDUCHIP_TRIG) & CAP_DONE_MASK))
    {
        if (HAL_GetTick() > deadline) return ARDUCAM_ERR_TIMEOUT;
        HAL_Delay(1);
    }

    /* ── 3. Read FIFO length (same as teammate's read_fifo_length) ── */
    uint32_t len1   = arducam_spi_read(0x42);
    uint32_t len2   = arducam_spi_read(0x43);
    uint32_t len3   = arducam_spi_read(0x44) & 0x7FU;
    uint32_t length = (len3 << 16) | (len2 << 8) | len1;

    if (length == 0U || length > JPEG_BUF_MAX) return ARDUCAM_ERR_SIZE;

    /* ── 4. Set RXFIFO threshold for 8-bit SPI on STM32L4 ─────────────
          (teammate does this right before the burst read)             ── */
    SET_BIT(hspi3.Instance->CR2, SPI_RXFIFO_THRESHOLD_QF);

    /* ── 5. Burst read entire FIFO into buffer ── */
    uint8_t burst_cmd = BURST_FIFO_READ;
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, &burst_cmd, 1, 10);

    for (uint32_t i = 0; i < length && i < JPEG_BUF_MAX; i++)
    {
        HAL_SPI_Receive(&hspi3, &s_jpeg_buf[i], 1, 10);
    }
    HAL_GPIO_WritePin(ARDUCAM_CS_PORT, ARDUCAM_CS_PIN, GPIO_PIN_SET);
    s_jpeg_len = length;

    /* ── 6. Find JPEG SOI marker 0xFF 0xD8 ─────────────────────────────
          Same as teammate's alignment block, but we scan the buffer    ── */
    s_jpeg_start = 0;
    for (uint32_t i = 0; i + 1U < s_jpeg_len; i++)
    {
        if (s_jpeg_buf[i] == 0xFFU && s_jpeg_buf[i + 1U] == 0xD8U)
        {
            s_jpeg_start = i;
            break;
        }
    }

    /* ── 7. Decode with TJpgDec and stream MCU blocks to display ── */
    JDEC    jdec;
    JpegCtx ctx = {
        .read_pos = 0,
        .disp_x   = (uint16_t)((ST7796_WIDTH  - 320U) / 2U),
        .disp_y   = 28U,
    };

    JRESULT res = jd_prepare(&jdec, _jpeg_in, s_tjpgd_work, TJPGD_WORK_SZ, &ctx);
    if (res != JDR_OK) return ARDUCAM_ERR_DECODE;

    res = jd_decomp(&jdec, _jpeg_out, 0);
    if (res != JDR_OK) return ARDUCAM_ERR_DECODE;

    return ARDUCAM_OK;
}
