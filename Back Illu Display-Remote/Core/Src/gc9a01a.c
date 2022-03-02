#include "gc9a01a.h"

void GC9A01_spi_frame(gc9a01_t *gc9a01) {
	  GC9A01_write_command(gc9a01, MEM_WR);
	  GC9A01_dc_on(gc9a01);
	  GC9A01_cs_off(gc9a01);
	  HAL_SPI_Transmit(gc9a01->spi, gc9a01->frame, gc9a01->frame_size, -1);
	  HAL_SPI_Transmit(gc9a01->spi, gc9a01->frame, gc9a01->frame_size, -1);
}

void GC9A01_spi_tx(gc9a01_t *gc9a01, uint8_t *data, size_t len) {
    HAL_SPI_Transmit(gc9a01->spi, data, len, -1);
}

void GC9A01_write_command(gc9a01_t *gc9a01, uint8_t cmd) {
    GC9A01_dc_off(gc9a01);
    GC9A01_cs_off(gc9a01);
    GC9A01_spi_tx(gc9a01, &cmd, sizeof(cmd));
    GC9A01_cs_on(gc9a01);
}

void GC9A01_write_data(gc9a01_t *gc9a01, uint8_t *data, size_t len) {
    GC9A01_dc_on(gc9a01);
    GC9A01_cs_off(gc9a01);
    GC9A01_spi_tx(gc9a01, data, len);
    GC9A01_cs_on(gc9a01);
}

static inline void GC9A01_write_byte(gc9a01_t *gc9a01, uint8_t val) {
    GC9A01_write_data(gc9a01, &val, sizeof(val));
}

void GC9A01_write(gc9a01_t *gc9a01, uint8_t *data, size_t len) {
    GC9A01_write_command(gc9a01, MEM_WR);
    GC9A01_write_data(gc9a01, data, len);
}

void GC9A01_write_continue(gc9a01_t *gc9a01, uint8_t *data, size_t len) {
    GC9A01_write_command(gc9a01, MEM_WR_CONT);
    GC9A01_write_data(gc9a01, data, len);
}

void GC9A01_init(gc9a01_t *gc9a01) {

    GC9A01_cs_on(gc9a01);
    HAL_Delay(5);
    GC9A01_res_off(gc9a01);
    HAL_Delay(10);
    GC9A01_res_on(gc9a01);
    HAL_Delay(120);

    /* Initial Sequence */

    GC9A01_write_command(gc9a01, 0xEF);

    GC9A01_write_command(gc9a01, 0xEB);
    GC9A01_write_byte(gc9a01, 0x14);

    GC9A01_write_command(gc9a01, 0xFE);
    GC9A01_write_command(gc9a01, 0xEF);

    GC9A01_write_command(gc9a01, 0xEB);
    GC9A01_write_byte(gc9a01, 0x14);

    GC9A01_write_command(gc9a01, 0x84);
    GC9A01_write_byte(gc9a01, 0x40);

    GC9A01_write_command(gc9a01, 0x85);
    GC9A01_write_byte(gc9a01, 0xFF);

    GC9A01_write_command(gc9a01, 0x86);
    GC9A01_write_byte(gc9a01, 0xFF);

    GC9A01_write_command(gc9a01, 0x87);
    GC9A01_write_byte(gc9a01, 0xFF);

    GC9A01_write_command(gc9a01, 0x88);
    GC9A01_write_byte(gc9a01, 0x0A);

    GC9A01_write_command(gc9a01, 0x89);
    GC9A01_write_byte(gc9a01, 0x21);

    GC9A01_write_command(gc9a01, 0x8A);
    GC9A01_write_byte(gc9a01, 0x00);

    GC9A01_write_command(gc9a01, 0x8B);
    GC9A01_write_byte(gc9a01, 0x80);

    GC9A01_write_command(gc9a01, 0x8C);
    GC9A01_write_byte(gc9a01, 0x01);

    GC9A01_write_command(gc9a01, 0x8D);
    GC9A01_write_byte(gc9a01, 0x01);

    GC9A01_write_command(gc9a01, 0x8E);
    GC9A01_write_byte(gc9a01, 0xFF);

    GC9A01_write_command(gc9a01, 0x8F);
    GC9A01_write_byte(gc9a01, 0xFF);


    GC9A01_write_command(gc9a01, 0xB6);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);

    GC9A01_write_command(gc9a01, 0x36);

#if ORIENTATION == 0
    GC9A01_write_byte(gc9a01, 0x18);
#elif ORIENTATION == 1
    GC9A01_write_byte(gc9a01, 0x28);
#elif ORIENTATION == 2
    GC9A01_write_byte(gc9a01, 0x48);
#else
    GC9A01_write_byte(gc9a01, 0x88);
#endif

    GC9A01_write_command(gc9a01, COLOR_MODE);
    GC9A01_write_byte(gc9a01, COLOR_MODE__16_BIT);

    GC9A01_write_command(gc9a01, 0x90);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x08);

    GC9A01_write_command(gc9a01, 0xBD);
    GC9A01_write_byte(gc9a01, 0x06);

    GC9A01_write_command(gc9a01, 0xBC);
    GC9A01_write_byte(gc9a01, 0x00);

    GC9A01_write_command(gc9a01, 0xFF);
    GC9A01_write_byte(gc9a01, 0x60);
    GC9A01_write_byte(gc9a01, 0x01);
    GC9A01_write_byte(gc9a01, 0x04);

    GC9A01_write_command(gc9a01, 0xC3);
    GC9A01_write_byte(gc9a01, 0x13);
    GC9A01_write_command(gc9a01, 0xC4);
    GC9A01_write_byte(gc9a01, 0x13);

    GC9A01_write_command(gc9a01, 0xC9);
    GC9A01_write_byte(gc9a01, 0x22);

    GC9A01_write_command(gc9a01, 0xBE);
    GC9A01_write_byte(gc9a01, 0x11);

    GC9A01_write_command(gc9a01, 0xE1);
    GC9A01_write_byte(gc9a01, 0x10);
    GC9A01_write_byte(gc9a01, 0x0E);

    GC9A01_write_command(gc9a01, 0xDF);
    GC9A01_write_byte(gc9a01, 0x21);
    GC9A01_write_byte(gc9a01, 0x0c);
    GC9A01_write_byte(gc9a01, 0x02);

    GC9A01_write_command(gc9a01, 0xF0);
    GC9A01_write_byte(gc9a01, 0x45);
    GC9A01_write_byte(gc9a01, 0x09);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x26);
    GC9A01_write_byte(gc9a01, 0x2A);

    GC9A01_write_command(gc9a01, 0xF1);
    GC9A01_write_byte(gc9a01, 0x43);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x72);
    GC9A01_write_byte(gc9a01, 0x36);
    GC9A01_write_byte(gc9a01, 0x37);
    GC9A01_write_byte(gc9a01, 0x6F);

    GC9A01_write_command(gc9a01, 0xF2);
    GC9A01_write_byte(gc9a01, 0x45);
    GC9A01_write_byte(gc9a01, 0x09);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x26);
    GC9A01_write_byte(gc9a01, 0x2A);

    GC9A01_write_command(gc9a01, 0xF3);
    GC9A01_write_byte(gc9a01, 0x43);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x72);
    GC9A01_write_byte(gc9a01, 0x36);
    GC9A01_write_byte(gc9a01, 0x37);
    GC9A01_write_byte(gc9a01, 0x6F);

    GC9A01_write_command(gc9a01, 0xED);
    GC9A01_write_byte(gc9a01, 0x1B);
    GC9A01_write_byte(gc9a01, 0x0B);

    GC9A01_write_command(gc9a01, 0xAE);
    GC9A01_write_byte(gc9a01, 0x77);

    GC9A01_write_command(gc9a01, 0xCD);
    GC9A01_write_byte(gc9a01, 0x63);

    GC9A01_write_command(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x07);
    GC9A01_write_byte(gc9a01, 0x07);
    GC9A01_write_byte(gc9a01, 0x04);
    GC9A01_write_byte(gc9a01, 0x0E);
    GC9A01_write_byte(gc9a01, 0x0F);
    GC9A01_write_byte(gc9a01, 0x09);
    GC9A01_write_byte(gc9a01, 0x07);
    GC9A01_write_byte(gc9a01, 0x08);
    GC9A01_write_byte(gc9a01, 0x03);

    GC9A01_write_command(gc9a01, 0xE8);
    GC9A01_write_byte(gc9a01, 0x34);

    GC9A01_write_command(gc9a01, 0x62);
    GC9A01_write_byte(gc9a01, 0x18);
    GC9A01_write_byte(gc9a01, 0x0D);
    GC9A01_write_byte(gc9a01, 0x71);
    GC9A01_write_byte(gc9a01, 0xED);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x18);
    GC9A01_write_byte(gc9a01, 0x0F);
    GC9A01_write_byte(gc9a01, 0x71);
    GC9A01_write_byte(gc9a01, 0xEF);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x70);

    GC9A01_write_command(gc9a01, 0x63);
    GC9A01_write_byte(gc9a01, 0x18);
    GC9A01_write_byte(gc9a01, 0x11);
    GC9A01_write_byte(gc9a01, 0x71);
    GC9A01_write_byte(gc9a01, 0xF1);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x18);
    GC9A01_write_byte(gc9a01, 0x13);
    GC9A01_write_byte(gc9a01, 0x71);
    GC9A01_write_byte(gc9a01, 0xF3);
    GC9A01_write_byte(gc9a01, 0x70);
    GC9A01_write_byte(gc9a01, 0x70);

    GC9A01_write_command(gc9a01, 0x64);
    GC9A01_write_byte(gc9a01, 0x28);
    GC9A01_write_byte(gc9a01, 0x29);
    GC9A01_write_byte(gc9a01, 0xF1);
    GC9A01_write_byte(gc9a01, 0x01);
    GC9A01_write_byte(gc9a01, 0xF1);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x07);

    GC9A01_write_command(gc9a01, 0x66);
    GC9A01_write_byte(gc9a01, 0x3C);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0xCD);
    GC9A01_write_byte(gc9a01, 0x67);
    GC9A01_write_byte(gc9a01, 0x45);
    GC9A01_write_byte(gc9a01, 0x45);
    GC9A01_write_byte(gc9a01, 0x10);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);

    GC9A01_write_command(gc9a01, 0x67);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x3C);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x01);
    GC9A01_write_byte(gc9a01, 0x54);
    GC9A01_write_byte(gc9a01, 0x10);
    GC9A01_write_byte(gc9a01, 0x32);
    GC9A01_write_byte(gc9a01, 0x98);

    GC9A01_write_command(gc9a01, 0x74);
    GC9A01_write_byte(gc9a01, 0x10);
    GC9A01_write_byte(gc9a01, 0x85);
    GC9A01_write_byte(gc9a01, 0x80);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x00);
    GC9A01_write_byte(gc9a01, 0x4E);
    GC9A01_write_byte(gc9a01, 0x00);

    GC9A01_write_command(gc9a01, 0x98);
    GC9A01_write_byte(gc9a01, 0x3e);
    GC9A01_write_byte(gc9a01, 0x07);

    GC9A01_write_command(gc9a01, 0x35);
    GC9A01_write_command(gc9a01, 0x21);

    GC9A01_write_command(gc9a01, 0x11);
    HAL_Delay(120);
    GC9A01_write_command(gc9a01, 0x29);
    HAL_Delay(20);
}
