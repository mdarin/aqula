#include "indication.h"




static uint8_t hg_code_reg[] = {CLR_CODE, CLR_CODE};
// описание дисплее
void (*display[])(uint8_t) = {
  set_hg1, 
  set_hg2,
};

// диод Норма
void norm_led_on(void)
{
  NORM_LED_DDR |= (1 << NORM_LED);
  NORM_LED_PORT |= (1 << NORM_LED);
  return;
}

void norm_led_off(void)
{
  NORM_LED_DDR &= ~(1 << NORM_LED);
  NORM_LED_PORT &= ~(1 << NORM_LED);
  
  return;
}

// диод Отаказ
void fault_led_on(void)
{
  FAULT_LED_DDR |= (1 << FAULT_LED);
  FAULT_LED_PORT |= (1 << FAULT_LED);
  return;
}

void fault_led_off(void)
{
  FAULT_LED_DDR &= ~(1 << FAULT_LED);
  FAULT_LED_PORT &= ~(1 << FAULT_LED);
  return;
}


// диод Работа
void run_led_on(void)
{
  RUN_LED_DDR |= (1 << RUN_LED);
  RUN_LED_PORT |= (1 << RUN_LED);
  return;
}

void run_led_off(void)
{
  RUN_LED_DDR &= ~(1 << RUN_LED);
  RUN_LED_PORT &= ~(1 << RUN_LED);
  return;
}

// диод Пол бака
void half_tank_led_on(void)
{
  hg_code_reg[HG2] |= (1 << 0);
  display[HG2](hg_code_reg[HG2]);
  return;
}

void half_tank_led_off(void)
{
  hg_code_reg[HG2] &= ~(1 << 0);
  display[HG2](hg_code_reg[HG2]);
  return;
}

// диод Нет давления воды в тубе
void no_water_pressure_led_on(void)
{
  hg_code_reg[HG1] |= (1 << 0);
  display[HG1](hg_code_reg[HG1]);
  return;
}

void no_water_pressure_led_off(void)
{
  hg_code_reg[HG1] &= ~(1 << 0);
  display[HG1](hg_code_reg[HG1]);
  return;
}

// эту чатсть надо переписать правильно, не знаю пока как это реализовать...
//ow_set_bus( volatile uint8_t* in,
//	volatile uint8_t* out,
//	volatile uint8_t* ddr,
//	uint8_t pin );
// пример использования
//	ow_set_bus(&PIND, &PORTD, &DDRD, PD4);



// драйверы инидкаторов, аппаратно зависимая часть
void set_hg1(uint8_t code)
{
  // установить лог.0 на линию строб
  PORTB &= ~(1 << PB2);   
  // передать байт по spi в сдвиговый регистр 
  spi_transfer(code);
  // задержка перед стробом   
  _delay_ms(1); 
  // подать строб на оба индикатора, лог.1
  PORTB |= (1 << PB2);
  // установить лог.0 на линию строба
  PORTB &= ~(1 << PB2);
  return;
}

void set_hg2(uint8_t code)
{
  // установить лог.0 на линию строб
  PORTB &= ~(1 << PB1);
  // передать байт по spi в сдвиговый регистр   
  spi_transfer(code);  
  // задержка перед стробом
  _delay_ms(1);        
  // подать строб на оба индикатора, лог.1   
  PORTB |= (1 << PB1);
  // установить лог.0 на линию строба
  PORTB &= ~(1 << PB1);
  return;
}


// функция форматированой засветки сегментов
// можно зажигать сегменты в любой последовательности
// название сегмета заглавной буквой A, B, C etc
// например: segments = "BC", на дисплее засветсятся сегменты B и С
// здесь сегменты активируются значеним лог.0
void set_8segf(uint8_t hg, char *segments)
{  
  if (NULL != segments) {    
	// put clear code into the register
	// костыль, нужен для корректной работы светодиода на сегменте H
	//uint8_t temp = hg_code_reg[hg]; 
	hg_code_reg[hg] = CLR_CODE;// | temp;
	// if segmets format is not empty then set segments
    if (strcmp(segments, "")) {	  
	  while (*segments) {
		switch (*segments) {
		case 'A':
	  	  hg_code_reg[hg] &= ~(1 << A_SEG); 
	  	  break;
        case 'B':
	  	  hg_code_reg[hg] &= ~(1 << B_SEG);
	  	  break;
	    case 'C':
	  	  hg_code_reg[hg] &= ~(1 << C_SEG);
		  break;
	    case 'D':
	      hg_code_reg[hg] &= ~(1 << D_SEG);
	  	  break;
	    case 'E':
	      hg_code_reg[hg] &= ~(1 << E_SEG);
	  	  break;
	    case 'F':
	      hg_code_reg[hg] &= ~(1 << F_SEG);
	  	  break;
	    case 'G':
	      hg_code_reg[hg] &= ~(1 << G_SEG);
	  	  break;
	    case 'H':
	      hg_code_reg[hg] &= ~(1 << H_SEG);
	  	  break;
		} 	
		segments++;
	  }
    } // eof if     
	// очистить знакоместо
	display[hg](CLR_CODE);
    // вывести символ
	display[hg](hg_code_reg[hg]);
  } // eof if
  return;
}


// функция вывода на дисплей, принимает символ числа('0','1','2' etc) [или цифру (0, 1, 2 etc)]
// реализована для того чтобы не заморачиваться каждый раз с таблицами перекодировки
// в дефайнах объявляется соответствие бит сегментам, и проблема таблиц исчезает
void put(uint8_t hg, uint8_t c)
{
  switch (c) {
  case '1':  case 1:
  	set_8segf(hg, "BC");
	break;
  case '2':  case 2:
  	set_8segf(hg, "ABGED");
    break;
  case '3':  case 3:
  	set_8segf(hg, "ABCDG");
	break;
  case '4':  case 4:
  	set_8segf(hg, "BCGF");
	break;
  case '5':  case 5: 
  	set_8segf(hg, "ACDFG");
	break;
  case '6':  case 6:
  	set_8segf(hg, "ACDEFG");
    break;
  case '7':  case 7:
    set_8segf(hg, "ABC");
	break;
  case '8':  case 8:
    set_8segf(hg, "ABCDEFG");
	break;
  case '9':  case 9:
    //set_8segf(hg, "ABCDFG");
	set_8segf(hg, "ABCFG");
	break;
  case '0': // case 0:
    set_8segf(hg, "ABCDEF");
	break;				  
  }
  return;
}


// void set_ind_reg(int16_t value)


// функция для отображения температуры в формате данной разраобтки, не универсальна
void put_temper(int16_t decicelsius) 
{
  if (0 <= decicelsius) { 
    int16_t t = decicelsius;
	char buff[8] = {0};
	
	t /= 10;

    itoa(t, buff, 10);
	//__DEBUG
	//uart_puts("====>");
	//uart_puts(buff);
	//uart_puts("\r\n");
	//uart_putc(buff[0]);
	//uart_puts("\r\n");
    //uart_putc(buff[1]);
	//uart_puts("\r\n");

    // определить значение температуры до 10, от 10 до 99
    if (0 <= t && t < 10) {
	  set_8segf(HG1, "");
	  put(HG2, buff[0]);
	} else if (t < 100) {
	  put(HG1, buff[0]);
	  put(HG2, buff[1]);
	}

  } else { 
  // если температура ниже нуля
  
  }
  return;
}



// функция для отображения температуры в формате данной разраобтки, не универсальна
void put_inactive_interval(int16_t interval) 
{
  if (0 <= interval) { 
	char buff[8] = {0};

    itoa(interval, buff, 10);

    // определить значение интервала до 10, от 10 до 99
    if (0 <= interval && interval < 10) {
	  set_8segf(HG1, "");
	  put(HG2, buff[0]);
	} else if (interval < 100) {
	  put(HG1, buff[0]);
	  put(HG2, buff[1]);
	}

  }// else { 

  
  //}
  return;
}


void put_pwm_value(uint8_t value) 
{
  if (0 <= value) { 
	char buff[8] = {0};

    itoa(value, buff, 10);

    // определить значение интервала до 10, от 10 до 99
    if (0 <= value && value < 10) {
	  set_8segf(HG1, "");
	  put(HG2, buff[0]);
	} else if (value < 100) {
	  put(HG1, buff[0]);
	  put(HG2, buff[1]);
	}

  }// else { 

  
  //}
  return;
}



// функция для отображения номера ошибки
void put_fault(uint8_t faultno)
{
  if (faultno) {
    char buff[8] = {0};

    itoa(faultno, buff, 10);

    // определить значение интервала до 10, от 10 до 99
    if (0 <= faultno && faultno < 10) {
	  set_8segf(HG1, "");
	  put(HG2, buff[0]);
	} else if (faultno < 100) {
	  put(HG1, buff[0]);
	  put(HG2, buff[1]);
	}
  }
  return;
}
