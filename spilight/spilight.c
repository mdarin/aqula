// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : at25df161.c
* - Compiler          : avr-gcc 3...>
*
*
* $Revision: 1.0 $
* $Date: 20:09 20.08.2014 GMT+4  $
*****************************************************************************/

#include "spilight.h"




/*! \brief  Configure and initialize the SPI
 *
 *  This function configures and initializes the SPI in master mode.
 *  It selects the master mode, the MSB as first transmitted bit, the SPI mode 0, the data rate  Fosc/4.
 *  The port direction ared defined as follows : PB5/SCK, PB3/MOSI, CS/!SS as outputs, otherwise PBx as input.
 *  It ends up by clearing the SPIF and then enabling the interrupts.
 *
 *
 *  \return  void.
 */
void init_spi_master()
{
   volatile char IOreg;
   
   
   //SPCR |= (0<<SPE);            // Disable the SPI to be able to configure the #SS line as an input even if the SPI is still configured as a slave 
   SPCR &= ~(1 << SPE);
   //DDRB |= (1<<CS);  
 //  Deselect_Serial_Memory;     // Pull high the chip select line of the SPI serial memory
  // SPCR |= (1<<MSTR);           // select the master mode prior to enable the SPI
 
   //SPSR |= (1 <<SPI2X); // eperimental spi speed doubling :) 

   // Enable the SPI interrupt line, enable the SPI,select the MSB first transmitted bit
   // SPI mode 0, data rate is Fosc/4
   SPCR |= (1 << SPIE) | (1 << SPE) | ( 1<<MSTR );//|(1<<CPOL);//|(1<<CPHA); // |(1<<CPOL)|(1<<CPHA) наш допил
        
   // Port B use SPI master mode alternate function
   // define the port direction : PB5/SCK, PB3/MOSI, CS/!SS as outputs, otherwise PBx as input
   DDRB |= (1 << PB5) | (1 << PB3); //|(1<<CS);
 
   // Clear the SPIF flag
   IOreg = SPSR;         
   IOreg = SPDR;        

   sei();
   
 }
 
/*! \brief  the SPI  Transfer Complete Interrupt Handler
 *
 * The SPI interrupt handler manages the accesses to the I/O Reg SPDR during a memory write cycle.
 * The values of the global variable(state_spi_mem,  byte_cnt_spi_mem, address, data_ptr_spi_mem) enables the handler to compute the next byte to be send
 * over the SPI interface as well as the next state_spi_mem. The finite state_spi_mem machine diagram is provided in the application note document.  
 *
 *  \return  void.
 */
ISR( SPI_STC_vect )
{
} 

/*********************************************************************************************
 SPI transfer
*********************************************************************************************/
/*! \brief  Send and receive a byte through the SPI interface
 *
 *  This function send  a byte to the SPI peripherals, waits by polling the end of the transfer and returns the byte stored in the SPDR register
 *  It ends up by enabling the interrupts.
 *
 *  \param data : byte to send.
 *
 *  \return  received byte.
 */
char spi_transfer(volatile char data)
{
  SPDR = data;                    // Start the transmission
  // если устройсво возвращае, пусть даже эхо, то сроку можно раскомментить
  //while (!(SPSR & (1<<SPIF))) continue;    // Wait the end of the transmission
  return SPDR;                    // return the received byte
}






