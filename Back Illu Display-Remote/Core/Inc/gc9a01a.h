#ifndef __GC9A01_H
#define __GC9A01_H

#include <stdint.h>
#include <stddef.h>
#include "main.h"

#define ORIENTATION 2   // Set the display orientation 0,1,2,3

// Command codes:
#define COL_ADDR_SET		0x2A
#define ROW_ADDR_SET		0x2B
#define MEM_WR				0x2C
#define COLOR_MODE			0x3A
#define COLOR_MODE__12_BIT	0x03
#define COLOR_MODE__16_BIT	0x05
#define COLOR_MODE__18_BIT	0x06
#define MEM_WR_CONT			0x3C
#define ON					1
#define OFF					0

#define RESET_ON()			RES_GPIO_Port->BSRR = (uint32_t) RES_Pin;
#define RESET_OFF()			RES_GPIO_Port->BRR	= (uint32_t) RES_Pin;
#define DC_ON()				DC_GPIO_Port->BSRR	= (uint32_t) DC_Pin;
#define DC_OFF()			DC_GPIO_Port->BRR	= (uint32_t) DC_Pin;
#define CS_ON()				CS_GPIO_Port->BSRR	= (uint32_t) CS_Pin;
#define CS_OFF()			CS_GPIO_Port->BRR	= (uint32_t) CS_Pin;

#define GC9A01_cs_on(gc9a01)	gc9a01->cs_port->BSRR	= (uint32_t) gc9a01->cs_pin
#define GC9A01_cs_off(gc9a01)	gc9a01->cs_port->BRR	= (uint32_t) gc9a01->cs_pin
#define GC9A01_dc_on(gc9a01)	gc9a01->dc_port->BSRR	= (uint32_t) gc9a01->dc_pin
#define GC9A01_dc_off(gc9a01)	gc9a01->dc_port->BRR	= (uint32_t) gc9a01->dc_pin
#define GC9A01_res_on(gc9a01)	gc9a01->res_port->BSRR	= (uint32_t) gc9a01->res_pin
#define GC9A01_res_off(gc9a01)	gc9a01->res_port->BRR	= (uint32_t) gc9a01->res_pin

void GC9A01_set_reset(uint8_t val);
void GC9A01_set_data_command(uint8_t val);
void GC9A01_set_chip_select(uint8_t val);

struct GC9A01_point {
    uint16_t X, Y;
};

struct GC9A01_frame {
    struct GC9A01_point start, end;
};

typedef struct {
	SPI_HandleTypeDef *spi;
	uint16_t *frame;
	uint16_t frame_size;

	GPIO_TypeDef *dc_port;
	GPIO_TypeDef *res_port;
	GPIO_TypeDef *cs_port;

	uint16_t res_pin;
	uint16_t dc_pin;
	uint16_t cs_pin;

} gc9a01_t;

void GC9A01_init(gc9a01_t *gc9a01);

void GC9A01_write(gc9a01_t *gc9a01, uint8_t *data, size_t len);
void GC9A01_write_continue(gc9a01_t *gc9a01, uint8_t *data, size_t len);

void GC9A01_write_data(gc9a01_t *gc9a01, uint8_t *data, size_t len);
void GC9A01_write_command(gc9a01_t *gc9a01, uint8_t cmd);

void GC9A01_spi_frame(gc9a01_t *gc9a01);
void GC9A01_spi_tx(gc9a01_t *gc9a01, uint8_t *data, size_t len);

#endif
