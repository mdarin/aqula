// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : at25df161.h
* - Compiler          : avr-gcc 3... >
*
*
* $Revision: 1.0 $
* $Date: 20:09 20.08.2014 GMT+4
*****************************************************************************/

#ifndef SPILIGHT_H
#define SPILIGHT_H

#include <avr/io.h>
#include <avr/interrupt.h>

#define SPILIGHT


/////////////////////////////////////////
// MACROs 
/////////////////////////////////////////
//! Marco :Pull down the chip select line of the serial memory
#define CS PB2
//! Enable the SPI Interrupt Macro
#define Enable_SPI_Interrupt SPCR |= (1 << SPIE)
//! Disable the SPI Interrupt Macro
#define Disable_SPI_Interrupt SPCR &= ~(1 << SPIE)



/////////////////////////////////////////////////////////////
// COMMON LOW LEVEL FUNCTIONS 
/////////////////////////////////////////////////////////////
void init_spi_master();
char spi_transfer(char data);

#endif // AT25DF161_H
