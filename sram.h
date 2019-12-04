/*
 * These functions allow for 16 bit 256k addressable
 */
#include <AIC23.h>

void cs1_high (void);
void cs0_high (void);
void cs1_low (void);
void cs0_low (void);

void init_spi(void);
void sram_write(Uint32 address, Uint16 data);
Uint16 sram_read(Uint32 address);
Uint16 spi_byte(Uint16 data);
void write_buffer(Uint32 addr, Uint16 data);
Uint16 read_buffer(Uint32 addr);



//240 +
