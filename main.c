/*****
 * name: Aqula - Aquarium cooler 
 * description: ����������� ��� ������ ���� � ������
 *     � ������ ����
 *     ���� ����������� ���� ������� ������, �� ����� ����
 *    ���� ����������� ���� �������� ������, �� ����� �������� �� ����
 *     ���� ����������� ������ ���������, �� ����� ������� �� ������ pwm = pos^2
 *     � ������ ������
 *     ���� ����������� ���� ������� ������, �� ����� ����
 *     ���� ����������� ���� �������� ������, �� ����� �������� �� ����
 *     ���� ����������� ������ ���������, �� ����� ������� � �����. � ���. ������������� �-��(�� ��������� ���� / 2)
 * 
 *    �������� ��������������� � ������ � ������ ������ ����������������� �����������
 *   version: 1.1rc
 *   create: 2014.12.13
 *   update: 2014.12.16
 *   location: ���������, �������
 *   
 *   ChangeLog:
 *     1.0 ��� ������� �����������
 *	   1.1 ������ ��������� ������, ������������ �������� �������(������ ����� ������ �������� ������� ���.1) 
 *
 * TODO:
 ***/

// ����������� ������ ds18b20
// ����� ����������� ��� �� �0
// ��� ������ � �������
// ���������(��� �����)

// int8 int9 int10 int11 ������ � �������


// ����������� ��������� � �������������� �� �-�� ��� ~>=50
// ����� ���� ������� ��� � ���������� ������(�������� �����)

 
#include <avr/version.h>
#if __AVR_LIBC_VERSION__ < 10606UL
#error "please update to avrlibc 1.6.6 or newer, not tested with older versions"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "spilight/spilight.h"
#include "uart/uart.h"
#include "uart/uart_addon.h"
#include "ds18b20/onewire.h"
#include "ds18b20/ds18x20.h"
#include "indication.h"
#include "keyboard.h"

#ifndef INFINITY
#define INFINITY 1
#endif
#ifndef F_CPU
#define F_CPU 16000000L
#endif


// 
#define ON 	1
#define OFF 0

// Nonvolatile memory settings 
// EEPROM status register
#define EEIF 0        
// 0   EEIF        
// 1   Reserved
// 2   Reserved
// 3   Reserved
// 4   Reserved
// 5   Reserved
// 6   Reserved
// 7   Reserved  
uint8_t EEMEM DVSR;

// Nonvolatile variables
int16_t EEMEM nv_t_lo;
int16_t EEMEM nv_t_hi;
// 


// TERMO SENSOR =======================================================
#define MAXSENSORS 5
#define NEWLINESTR "\r\n"

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
 
// ==================================================================


// Display
extern void (*display[])(uint8_t);

//__DEBUG
// UART settings
//static unsigned int baudrate = 19200;



#define PWM_MAX 255
#define PWM_MIN 70
static volatile uint16_t pwm_value = PWM_MAX/2;
//static volatile uint8_t pwm_max = 255; // ����������������� �������������
//static volatile uint8_t pwm_min = 0; // ����������� ����������

// ��� ������������
// ������� �������� ��� �� 50 �� 255 = 155 �������� ��������� ������� ����������
// ��� ������ ��� = ������� �������� ��� / 100 ����� = ��������� 2 �������� �� ��� 
// 
// 0 - ��� �������� (�������� ��� = 50) // �����������!!!
// 1 - ��� ������� (�������� ��� = 52)
// 2 - ��� ������� (�������� ��� = 54)
// � �.�. � ���� ��������� 2 

#define UP 1
#define DOWN 2
volatile uint8_t pwm_direction = UP; 


// function protos
void startup_init(void);
void nv_init(void);
void display_init(void);
void display_solut(void);
void display_clear(void);

//-------------------------------------------------------------
// ���������� ���������� �0, ���������� ���
ISR(TIMER0_COMPB_vect)
{
  // ����� ���������������� ���
  //put(HG1, 1);
}


volatile static uint8_t menu_timeout = 0;
//-------------------------------------------------------------
ISR (TIMER1_OVF_vect)
{
  TCCR1B = 0; // stop
  
  menu_timeout = 1; 
  TCNT1H = 0x88; // �������� 3 ���
  TCNT1L = 0xB8;
  //TCCR1B = (1 << CS12) | (1 << CS10); // start
}

//-------------------------------------------------------------
#define PRESCALER_VALUE 100//600 // ~1 sec 
static volatile uint16_t prog_prescaler = PRESCALER_VALUE;
static volatile uint8_t timeout = 0;
//#define MENU_PRESCALER_VALUE 3
//static volatile uint8_t menu_prescaler = MENU_PRESCALER_VALUE; 

// ������-��������� ��� ������ ��������� �������� � ������ ������� �����������
ISR (TIMER2_OVF_vect)
{
  TCCR2B = 0;
  if (prog_prescaler <= 0 ) {
    prog_prescaler = PRESCALER_VALUE;
	timeout = 1;
  } else {
    prog_prescaler--;
  } 
  TCNT2 = 0;
  TCCR2B |= (1 << CS22) | (1 << CS20);

}



void init_menutimeout(void)
{
  TIMSK1 |= (1 << TOIE1);
}

void start_menutimeout(void)
{
  menu_timeout = 0;
  TCNT1H = 0x88; // �������� ������� �����...
  TCNT1L = 0xB8;
  TCCR1B |= (1 << CS12) | (1 << CS10);
}

void restart_menutimeout(void)
{
  TCCR1B = 0;
  TCNT1H = 0x88; // �������� 3 ������� �����...
  TCNT1L = 0xB8;
  TCCR1B |= (1 << CS12) | (1 << CS10);
}

void stop_menutimeout(void)
{
  TCCR1B = 0;
}


//-------------------------------------------------------------
static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	//uart_puts_P( NEWLINESTR " [*] Scanning Bus for DS18X20..." NEWLINESTR );
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			//uart_puts_P( " [!] No Sensor found" NEWLINESTR );
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			//uart_puts_P( " [!] Bus Error" NEWLINESTR );
			break;
		}
		
		for( i=0; i < OW_ROMCODE_SIZE; i++ ) {
			gSensorIDs[nSensors][i] = id[i];
		}
		
		nSensors++;
	}
	
	return nSensors;
}
//-------------------------------------------------------------
int16_t get_temper(uint8_t nSensors, uint8_t *error)
{
  int16_t decicelsius = 0; 

  if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) 
		== DS18X20_OK) {
		_delay_ms( DS18B20_TCONV_12BIT );
		for (uint8_t i = 0; i < nSensors; i++ ) {
			if ( DS18X20_read_decicelsius( &gSensorIDs[i][0], &decicelsius )
			     == DS18X20_OK ) {
				 // TODO output temperature for examle
			}
			else {
				*error++;
			}
		}
	} else {
		*error++;
	}

	return decicelsius;
}


void set_pwm_off(void)
{
  // ������������ ���.0 �� ������
  OCR0B = 0; 
  DDRD &= ~(1 << PD5);
}


void set_pwm_on(void)
{
  // ��������� �0 ��� ������������� ���
  TIMSK0 |= (1 << OCIE0B);
  TCCR0A |= (1 << COM0B1) | (1 << WGM00);
  TCCR0B |= /*(1 << CS02) |*/ (1 << CS00);
  //OCR0B = pwm_value;
  // ���� ��������� ���� �� �����!!
  DDRD |= (1 << PD5);
}

void set_pwm_value(uint8_t value)
{
  OCR0B = value;
}

uint8_t get_pwm_value(void)
{
  return OCR0B;
}

void init_temper_timeout(void)
{
  TIMSK2 |= (1 << TOIE2);
  TCNT2 = 0;
  TCCR2B |= (1 << CS22) | (1 << CS20);
  return;
}


// ���������� ������ ������ 
// ���� ����� - ���� ����
// ���� �� ����� - ���� ���
void mode_led_on()
{
  DDRD |= (1 << PD0);
  PORTD |= (1 << PD0);
  return;
}

void mode_led_off()
{
  DDRD |= (1 << PD0);
  PORTD &= ~(1 << PD0);
  return;
}

//-------------------------------------------------------------
void startup_init(void)
{  
  init_menutimeout();
  // ��������� �2 ��� ������ ��������� ������� � ������ �����������
  init_temper_timeout();
  // ��������� ���������� ��� �������
  init_keys();  
  // ��� ���???
  //DDRC |= (1 << PC0) | (1 << PC1);
  //PORTC = 0xFF;
  display_init();
  // ������������������� ��� ������-������ �-��� ����� �������� ��������� :)
  nv_init();
  // ��������� ���� ����������
  sei();
  return;
}

//-------------------------------------------------------------
// main driver
// FSM states and transitions
//* -> operate,
//  -> fault.
//operate -inc_key,dec_key-> pwm_setup
//pwm_setup -timeout->operate
//operate -menu_key->  
//t_lo_setup -menu_key-> t_hi_setup
//           -timeout-> operate
//t_hi_setup -menu_key-> operate
//           -timeout-> operate 
int main(void)
{

  uint8_t automode = ON; // ��� ������� ���������� �� ��� �������!  
  // ������ ����� ����������� �� ���
  eeprom_busy_wait();
  int16_t t_lo = eeprom_read_word(&nv_t_lo);
  // ������� ����� �����������  �� ���
  eeprom_busy_wait();
  int16_t t_hi = eeprom_read_word(&nv_t_hi); 
  int16_t t_cur = 0; // ������� �����������
  int16_t t_prev = 0; // ������� �����������
  uint8_t key = NO_KEYS; // ������� �������
  uint8_t nSensors; // ��������� ��������
  uint8_t error_sens; // ������ ������� �����������
  uint8_t fault = 0; // ������� �������
  uint8_t cooler_state = ON; // ��������� ������ ������

#define OPERATE 1
#define T_LOW_THRESHOLD 2
#define T_HIGH_THRESHOLD 3
#define PWM_SETUP 4
#define FAULT 5
  uint8_t state = OPERATE;
  
  startup_init();

  //__DEBUG
  //uart_init(UART_BAUD_SELECT(baudrate, F_CPU));
  //uart_puts("Aqula started...\r\n");
  
  // ��������� �������� � ���� ���������
  // ���� ������ 
  set_pwm_value(PWM_MAX);
  set_pwm_on();
  display_solut();
  // ������� ����� ��������
  set_pwm_value(PWM_MIN+10);

  //__DEBUG 
  //uart_puts("*->operate\r\n");

  // �������� ����������� �������� � �������� ������� ��� �����. �����.
  nSensors = search_sensors();
  // ���� ��� ������� �����������, �� ���� � ����. ������
  if ( nSensors == 0 ) {
    // �������� ���������� �������
	set_pwm_off();
	error_sens++;
	state = FAULT;
	set_8segf(HG1, "ADEFG");
    set_8segf(HG2, "EG");
	//uart_puts("NO SENSORS!\r\n");
	// * -> [fault]
  } else {
    t_cur = t_prev = get_temper(nSensors, &error_sens);
  }	

  // main loop
  while (INFINITY) {		
    // FSM
    switch (state) {
	case T_LOW_THRESHOLD:
	  if (!menu_timeout) {
	    // ���������� ������� ����������	  
	    // �������� ������� ������ ���� ����� ���� 
        key = get_key();
	    switch (key) {
	    case MENU_KEY:
          //uart_puts("t lo ->t high\r\n");
	      // �������� � �������� 
		  for (uint8_t i = 0; i < 3; i++) {
		    set_8segf(HG1, "BCGEF");
		    set_8segf(HG2, "E");
		    _delay_ms(100);
		    display_clear();
		    _delay_ms(100);
		  }
		  // �������� �������� � ���
		  eeprom_busy_wait();
	      eeprom_update_word(&nv_t_lo, t_lo);
		  
		  state = T_HIGH_THRESHOLD;
		  start_menutimeout();
	    break;
        case INC_KEY:
          //uart_puts("t lo inc\r\n");		  
	      restart_menutimeout();
		  t_lo += 10;
		  if (t_lo >= 1000) t_lo = 1000;
	      break;
	    case DEC_KEY:
		  //uart_puts("t lo dec\r\n");
		  restart_menutimeout();
		  t_lo -= 10;
		  if (t_lo < 0) t_lo = 0;
	      break;     
	    } // oef switch

        // ������� �������� �������� ������        
		put_temper(t_lo);		
	  } else {
	    //uart_puts("t lo -> op\r\n");
	    // �������� �������� � ���
        eeprom_busy_wait();
	    eeprom_update_word(&nv_t_lo, t_lo); 
		state = OPERATE;
	  }
	  break;
    case T_HIGH_THRESHOLD:
	  if (!menu_timeout) {
	    // ���������� ������� ����������	  
	    // �������� ������� ������ ���� ����� ���� 
        key = get_key();
	    switch (key) {
	    case MENU_KEY:
          stop_menutimeout();
		  //uart_puts("t hi ->operate\r\n");
		  // �������� �������� � ���
          eeprom_busy_wait();
	      eeprom_update_word(&nv_t_hi, t_hi);
	      state = OPERATE;
	      break;
        case INC_KEY:
          //uart_puts("t hi inc\r\n");
	      restart_menutimeout();
          t_hi += 10;
		  if (t_hi >= 1000) t_hi = 1000;
	      break;
	    case DEC_KEY:
		  //uart_puts("t hi dec\r\n");
	      restart_menutimeout();
          t_hi -= 10;
		  if (t_hi < 0) t_hi = 0;
	      break;     
	    } // oef switch

        // ������� �������� �������� ������
  	    put_temper(t_hi);
	  } else {
	    //uart_puts("t hi -> op\r\n");
		// �������� �������� � ���
        eeprom_busy_wait();
	    eeprom_update_word(&nv_t_hi, t_hi);
	    state = OPERATE;
	  }
	  break;
    case PWM_SETUP:
	  if (!menu_timeout) {
	    // ���������� ������� ����������	  
	    // �������� ������� ������ ���� ����� ���� 
        key = get_key();
	    switch (key) {
        case INC_KEY:         
		  //pwm_value = get_pwm_value();
		  pwm_value += 5;//++;
		  if (pwm_value >= 255) pwm_value = PWM_MAX;
		  set_pwm_value(pwm_value);
	      restart_menutimeout();
	      //__DEBUG
		  //uart_puts("pwm inc[");
          //uart_put_int(pwm_value);
		  //uart_puts("]\r\n");
	      
		  break;
	    case DEC_KEY:
		  //pwm_value = get_pwm_value();
		  pwm_value -= 5;//--;
		  if (pwm_value <= PWM_MIN) pwm_value = PWM_MIN;
		  set_pwm_value(pwm_value);
		  restart_menutimeout();
	      
		  //uart_puts("pwm dec[");
		  //uart_put_int(pwm_value);
		  //uart_puts("]\r\n");
	      
		  break;     
	    } // oef switch
      
		// ������� �������� ���       
		// ��� ���� ������� ������� � ��������
		// 1 ������� = 100 / (PWM_MAX - PWM_MIN) * pwm_value - magic_offset
		uint8_t pwm4user = 0.54 * pwm_value - 38; 
		put_pwm_value(pwm4user);
	  } else {
	    //uart_puts("pwm -> op\r\n");
	    state = OPERATE;
	  }
	  break;
    case FAULT:
	  // �������� ����������� �������� � �������� ������� ��� �����. �����.
      nSensors = search_sensors();
	  // ���� ������ ������ ���� ������ �����������, �� ���� �� ����. ������
	  if (nSensors > 0) {
	    // �������� ���������� �������
		set_pwm_on();
		// �������� �������� ����������� � ������� ���
	    t_cur = get_temper(nSensors, &error_sens);
		put_temper(t_cur); 
	    state = OPERATE;		
	  } 
	  break;
    default: // OPERATE
	  // ������� ��������� �������� �����������
      nSensors = search_sensors();
	  // ���� ��� ������� �����������, �� ���� � ����. ������
	  if ( nSensors == 0 ) {
		// �������� ���������� �������
	    set_pwm_off();
		error_sens++;
		state = FAULT;
		set_8segf(HG1, "ADEFG");
        set_8segf(HG2, "EG");
		//uart_puts("NO SENSORS!\r\n");
		
		//faults |= (1 << TEMPER_SENS_FAULT);
		//fault_led_on();
      	//norm_led_off();
		// [operate] -> [fault]
		continue; // ���������� ���� ��������, ������� ���������� else
	  }
	  
	  // �������� �����������
	  // ���������� �����������
	  if (timeout) {
	    timeout = 0;
		t_cur = get_temper(nSensors, &error_sens);
		put_temper(t_cur);    
	  }

	  // ��������� �������	 
      // �������� ������� �������� ���       
	  uint8_t cur_pwm_value = get_pwm_value(); 
	  switch (cooler_state) {
	  case ON:
	    if (cur_pwm_value <= PWM_MIN) {
		  cooler_state = OFF;
		  set_pwm_off();
		} 
		break;
      case OFF:
	    if (cur_pwm_value > PWM_MIN) {
		  // ����� ���������� ������ � ����� ���� ��� ����
		  if (cur_pwm_value < PWM_MIN/3) {		    
			set_pwm_on();
			// ���������� ������������� �������� ��� ��� ������ �� 1 ���.
			set_pwm_value(PWM_MAX);
			_delay_ms(2000);
			// ���������� ���������� �������� ��� ���������� ������
			set_pwm_value(pwm_value);
		  } else {
		    // ������ �������� �����
			set_pwm_on();
		  }
		  cooler_state = ON; 
		}
		break; 
	  }
	   
	  
      // � ������ ���� ������������ ����������� � � ����������� �� �� ��������� ��������� ������
      // ������� �������� ������ ������� �� �����������		
      // ��������	
      // ��������� ������� �� ������ ������� �� ������� ������� ����������� �� ��������� ��������� ���	
      // � �� ���� ����������� ������� ����������� ���� ��� ������� �������� �������� ������	
      // ������� ����� ����� y = x^2	
      	  
	  // ��������� ������� t_hi > t_lo ����� ���������� �� ��������� t_hi = t_lo + offset
	  if (t_hi <= t_lo) {
	    t_hi = t_lo + 50; // +5 *C
		// �������� �������� � ���
        eeprom_busy_wait();
	    eeprom_update_word(&nv_t_hi, t_hi);
	  }
	  
	  // ��������� ������������ �������� ��������� �������������� ���������� � ������ ���� ������������,�� ������ 10, �� ������ ��������
	  #define NLEVELS 11
	  if (automode) {
	    if (t_cur != t_prev) {
	      t_prev = t_cur;
	      uint16_t diapasone = t_hi - t_lo;
	  //__DEBUG
	  //uart_puts("doapasone:");
	  //uart_put_int(diapasone);
	  //uart_puts("\r\n");	  
	      uint16_t step = diapasone / (NLEVELS-1); 
      //__DEBUG
	  //uart_puts("step:");
	  //uart_put_int(step);
	  //uart_puts("\r\n");
	  // ���������� ������ � ��������� ����� �� ���������	  
	      uint16_t levels[NLEVELS] = {0};
	  // ��������� ������� � ������ ������ ����������
	      levels[0] = t_lo;
	      levels[NLEVELS-1] = t_hi;
	  //__DEBUG	  
	  //uart_puts("levels[0]:");
	  //uart_put_int(levels[0]);
	  //uart_puts("\r\n");	  
         for (uint8_t i = 1; i < NLEVELS-1; i++) {
	       levels[i] = levels[i-1] + step;
		//__DEBUG
	    //uart_puts("levels[");
		//uart_put_int(i);
		//uart_puts("]:");
	    //uart_put_int(levels[i]);
	    //uart_puts("\r\n");
	     }
      //__DEBUG	  
	  //uart_puts("levels[10]:");
	  //uart_put_int(levels[NLEVELS-1]);
	  //uart_puts("\r\n");
	  
      // ���������� ������� ����������� �� ��������� � ��������� ����������(� ����� �� ��� ������ ��� ���������)
	  // � ���������� �������� ��� � ����������� � ������� ���������� ����������� �� ������ ������������� 
	     uint8_t pos = 0; // ������� ��������� �����������, �� ��������������� ���������, ���� 0, �� ���� ������� ���������, ����� ��������
	  // ���� ������������� ���� �������� ������ �� ��������� ��� �� PWM_MAX 
	     if (t_cur > levels[NLEVELS-1] ) {
	       set_pwm_value(PWM_MAX);
		   //uart_puts("set PWM_MAX\r\n");
	     } else if(t_cur < levels[0]) {
	       // ���� ����������� ���� ������� ��������� �� ��������� ����������
           set_pwm_value(PWM_MIN);
		   //uart_puts("set PWM_MIN - disabled\r\n");
	     } else {
	     // ����� �������� ������� �� ������ �������������
         // ���� ����������� � ��������� ���������
		   if (levels[NLEVELS-2] < t_cur && t_cur <= levels[NLEVELS-1]) {
		     pos = NLEVELS-1;
		     set_pwm_value(pos*pos+PWM_MIN);
	         //uart_puts("set PWM ");
			 //uart_put_int(10);
			 //uart_puts("\r\n");
		   } else {  
		     // ���������� ���. ������� �����������
		     for (uint8_t i = 0; i < NLEVELS-1; i++) {
               if (levels[i] < t_cur && t_cur <= levels[i+1]) {			  
			     pos = i+1;
			     set_pwm_value(pos*pos+PWM_MIN);
			     //__DEBUG
				 //uart_puts("set PWM ");
			     //uart_put_int(pos);
			     //uart_puts("\r\n");
			     i = NLEVELS; // ����� �� �����
			   }
             }
		   }
	     }	 
	   } // eof if t_cur != t_prev
     } // eof if automode
	 else { // ������� ����� �� ���������������� �������� ���� ����������� �� ����� ���� ������� 
       if (t_cur != t_prev) {
	     t_prev = t_cur;
	     // �������� ���� ���� �������
	     if(t_cur < t_lo) {
	       // ���� ����������� ���� ������� ��������� �� ����� ����������
           set_pwm_value(PWM_MIN);
		   //uart_puts("set PWM_MIN OFF\r\n");
	     } else if (t_cur > t_hi) {
	       // ���� ����������� ���� �������� ������ ������ ����� �� ���������
	       set_pwm_value(PWM_MAX);
		   //uart_puts("set PWM_MAX FULL\r\n");
	     } else {
	       // �������� ����� �� ���������������� ��������
	       set_pwm_value(pwm_value);
		   //uart_puts("set PWM ");			  			  
		   //uart_put_int(pwm_value);
           //uart_puts("\r\n");
	     }
	   } // eof t_cur != t_prev
	 }

      // ���������� ������� ����������	  
	  // �������� ������� ������ ���� ����� ���� 
      key = get_key();
	  switch (key) {
	  case MENU_KEY:
        //uart_puts("op ->t_low\r\n");
		state = T_LOW_THRESHOLD;
		// �������� � �������� 
		for (uint8_t i = 0; i < 3; i++) {
		  set_8segf(HG1, "DEF");
		  set_8segf(HG2, "EGCD");
		  _delay_ms(100);
		  display_clear();
		  _delay_ms(100);
		}
	    start_menutimeout();
	    break;
      case AUTO_KEY:
	    //uart_puts("auto[");
		// ����������� ��� ������� ����� ����
		switch(automode) {
		case ON:
		  //uart_puts("OFF]\r\n");
		  automode = OFF;
		  // �������� ���� ����� ���� ����
		  mode_led_on();
		  break;
		case OFF:
		  //uart_puts("ON]\r\n");
		  automode = ON;
		  // ��������� ���� ����� ���� ���
		  mode_led_off();
		  break;
		}
	    break;
      case INC_KEY: 
	  case DEC_KEY:
	    if (!automode) {
          //uart_puts("op -> pwm_setup\r\n");
          _delay_ms(100);
		  // ��� ���� ������� ������� � ��������
		  // 1 ������� = 100 / (PWM_MAX - PWM_MIN) * pwm_value - magic_offset
		  // ������� �������� �� ������ ���������� ����������
		  uint8_t pwm4user = 0.54 * pwm_value - 38; 
		  put_pwm_value(pwm4user);
		  
		  state = PWM_SETUP;
	      start_menutimeout();
		}
	    break;
	  } // oef switch
	  
	  break;
	} // eof switch
    
  } // eof main loop
  return 0;
}

//-------------------------------------------------------------
void nv_init(void)
{
  // ��������� ������� ��������� ����������
  eeprom_busy_wait();
  uint8_t dvsreg = eeprom_read_byte(&DVSR);

  // ���� ��� �� �������������������, �� �������� ���� ��������� �� ���������
  if (bit_is_set(dvsreg, EEIF)) {
  
    // �������� ��������(�����-����)
	
	// �������� ������� ������ �����������
	eeprom_busy_wait();
	eeprom_update_word(&nv_t_lo, 220);
    
	// �������� �������� ������ �����������
	eeprom_busy_wait();
	eeprom_update_word(&nv_t_hi, 280);


    // ���������� ��� ������������� � ���(������� ���������)
    dvsreg &= ~(1 << EEIF);
	// �������� ������ ��������� � ���
	eeprom_busy_wait(); 
    eeprom_update_byte(&DVSR, &dvsreg);
    
	// __DEBUG
	//uart0_puts("WR 'empty'|0 to EEMEM\r\n");
  }
  
  return;
}


//-------------------------------------------------------------
void display_init(void)
{
  // ��������� ���� ������������ �������� �� �����
  DDRB |= (1 << HG1_STROBE); 
  DDRB |= (1 << HG2_STROBE); 
  
  init_spi_master();
  return;
}


//-------------------------------------------------------------
// �������� �� ���������������� ��� ���������
void display_solut(void)
{

  display_clear();

  set_8segf(HG1, "EF");
  _delay_ms(150);
  set_8segf(HG1, "BC");
  _delay_ms(150);
  set_8segf(HG1, "");
  set_8segf(HG2, "EF");
  _delay_ms(150);
  set_8segf(HG2, "BC");
  _delay_ms(150);

  set_8segf(HG1, "D");
  set_8segf(HG2, "D");
  _delay_ms(150);
  set_8segf(HG1, "G");
  set_8segf(HG2, "G");
  _delay_ms(150);
  set_8segf(HG1, "A");
  set_8segf(HG2, "A");
  _delay_ms(150);

  set_8segf(HG1, "D");
  set_8segf(HG2, "D");
  _delay_ms(150);  
  set_8segf(HG1, "DE");
  set_8segf(HG2, "DE");
  _delay_ms(150);
  set_8segf(HG1, "DEF");
  set_8segf(HG2, "DEF");
  _delay_ms(150);

  set_8segf(HG1, "DEF");
  set_8segf(HG2, "DEF");
  _delay_ms(150);
  set_8segf(HG1, "DEFA");
  set_8segf(HG2, "DEFA");

  _delay_ms(150);
  set_8segf(HG1, "DEFAB");
  set_8segf(HG2, "DEFAB");
  _delay_ms(150);
  set_8segf(HG1, "DEFABC");
  set_8segf(HG2, "DEFABC");
  _delay_ms(150);

  set_8segf(HG1, "");
  set_8segf(HG2, "");
  _delay_ms(500);
  
  set_8segf(HG1, "DEFABGCH");
  set_8segf(HG2, "DEFABGCH");
  _delay_ms(500);

  display_clear();
  return;
}


void display_clear(void)
{
  set_8segf(HG1, "");
  set_8segf(HG2, "");
  return;
}




