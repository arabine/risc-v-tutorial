
#ifndef SPI_DEFINED
#define SPI_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
SD Cards operate at two speed modes. The default mode clock speed is 0 -25MHz. A high speed mode is available at clock speed of 0 -50MHz.
*/
void spi_set_fclk_slow();
void spi_set_fclk_fast();

void spi_cs_high();
void spi_cs_low();


uint8_t xchg_spi (uint8_t dat);

/**
 * Pointer to data buffer
 * Number of bytes to receive (even number)
 */
void rcvr_spi_multi (uint8_t *buff, uint32_t btr);

/**
 * Pointer to the data
 * Number of bytes to send (even number)
 */
void xmit_spi_multi (const uint8_t *buff, uint32_t btx);

/**
 * Setup SPI block ID
 */
void spi_initialize(uint8_t id);


#endif // SPI_DEFINED
