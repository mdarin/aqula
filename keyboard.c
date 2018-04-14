#include "keyboard.h"


//================
volatile char irq8 = 0;
volatile char irq9 = 0;
volatile char irq10 = 0;
volatile char irq11 = 0;
// попытка обработать кнопку[работает как-то]
// http://www.avrfreaks.net/forum/pcint1-not-working-pcint0-works-fine
// There are three interrupt vectors:
//ISR(PCINT0_vect){} // for pins PCINT0-PCINT7   (PB0-PB7)  
//ISR(PCINT1_vect){} // for pins PCINT8-PCINT14  (PC0-PC6)
//ISR(PCINT2_vect){} // for pins PCINT16-PCINT23 (PD0-PD7)
static volatile char enc_direction = 0;
ISR (PCINT1_vect)
{

  // УСТАНОВИТЬ кнопка 
  if (bit_is_set(PINC, PC0)) {
    irq8 = 1;
  }
   
  // АВТО кнопка
  if (bit_is_clear(PINC, PC1)) {
    irq9 = 1;
  } 
  
  // Encoder code tab
  // A B
  // 0 1 - left
  // 1 0 - right
  // ============== working cod ex================
  /*
//====Залип=======
  if (bit_is_set(PINC, PC2) && bit_is_set(PINC, PC3)) {    	
	enc_direction = 0;
	return;
  }
  
  if (bit_is_clear(PINC, PC2) && bit_is_set(PINC, PC3) && !enc_direction) {    	
	enc_direction = 1;
  }
   
  if (bit_is_clear(PINC, PC3) && bit_is_set(PINC, PC2) && 1 == enc_direction) {
	irq10 = 1;
	enc_direction = 0;
	return;
  }  
   
  if (bit_is_clear(PINC, PC3) && bit_is_set(PINC, PC2) && !enc_direction) {
	enc_direction = 2;
  } 

  if (bit_is_clear(PINC, PC2) && bit_is_set(PINC, PC3) && 2 == enc_direction) {    	
	irq11 = 1;
	enc_direction = 0;
  	return;
  } 
  */
  //==========================

  // обработка переходного состояния
  if (bit_is_set(PINC, PC2) && bit_is_set(PINC, PC3)) {    	
	enc_direction = 0;
	return;
  }

  if (bit_is_clear(PINC, PC2) && bit_is_set(PINC, PC3) && !enc_direction) {    	
		enc_direction = 1;
  } else if (bit_is_clear(PINC, PC3) && bit_is_set(PINC, PC2) && !enc_direction) {
		enc_direction = 2;
  } 

  if (enc_direction) {
    if (bit_is_clear(PINC, PC3) && bit_is_set(PINC, PC2) && 1 == enc_direction) {
		irq11 = 1;
		enc_direction = 0;
    } else if (bit_is_clear(PINC, PC2) && bit_is_set(PINC, PC3) && 2 == enc_direction) {    	
		irq10 = 1;
		enc_direction = 0;
    } 
  }
  
  return; 
}


void init_keys(void)
{
  PCICR |= (1 << PCIE1); // Any change on any enabled PCINT14..8 pin will cause an interrupt.
  PCMSK1 |= (1 << PCINT11) | (1 << PCINT10) | (1 << PCINT9) | (1 << PCINT8); // Each PCINT14..8-bit selects whether pin change interrupt is enabled on the corresponding I/O pin. 
  return;
}

uint8_t get_key(void)
{
  uint8_t key = NO_KEYS;
  if (irq8) {
    irq8 = 0;  
    key = MENU_KEY;
  } else if (irq9) {
    irq9 = 0;  
    key = AUTO_KEY;
  } else if (irq10) {
    irq10 = 0;
    key = INC_KEY;
  } else if (irq11) {
    irq11 = 0;
	key = DEC_KEY;
  }
  return key;
}


