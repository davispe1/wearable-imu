/**
 * w25q64.c — driver for the W25Q64 8 MB SPI NOR flash.
 *
 * Used as session-insurance log: if all live links fail, samples are
 * recovered over SWD after the session.
 *
 * TODO:
 *  - Implement w25q64_init() — SPI config, chip ID check.
 *  - Implement w25q64_write_page() / w25q64_read_page().
 *  - Implement w25q64_erase_sector().
 *  - Implement circular-log append with wear levelling.
 */

#include "w25q64.h"

void w25q64_init(void) {}
int  w25q64_write(uint32_t addr, const uint8_t *buf, uint32_t len) { (void)addr; (void)buf; (void)len; return 0; }
int  w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len)        { (void)addr; (void)buf; (void)len; return 0; }
