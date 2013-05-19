
/**
 * various functions to deal with flaws and portability issues
 *
 * @author Matthias Ringwald
 */
 
// mspgcc LTS doesn't support 20-bit pointer yet -> put const data into .fartext

#pragma once

#include <stdint.h>

// single byte read
uint8_t FlashReadByte (uint32_t addr);

// argument order matches memcpy
void FlashReadBlock(uint8_t *buffer, uint32_t addr,  uint16_t len);

// Copy init data
void MoveInitBytes(unsigned int offset, unsigned char *dest, unsigned int length);

