/*
 * SPI DAC interface.
 */
#ifndef _DAC_H_
#define _DAC_H_

int spi_dac_set(ulong output);
ulong spi_dac_get(void);

#endif	/* _DAC_H_ */
