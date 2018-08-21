#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "wiring.h"
#include "eeprom.h"
#include "config.h"

#define REG(addr) (addr <= 0x3F ? _SFR_IO8(addr) : _SFR_MEM8(addr))

#define __SREG 0x3f

static volatile uint16_t timer_0_overflow_count = 0;

//static uint8_t led_ctrl_port_addr = pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[PIN_LED_SK6812])) >> 4].data_out));
//static uint8_t led_ctrl_bit_no = pgm_read_byte(&(pin_port_bit[PIN_LED_SK6812])) & 0x0f;

void led_SendDataFLASH(const uint8_t *data, uint8_t lenght, uint8_t intensity)
{
    uint8_t buffer[STATIC_LED_BUFFER_SIZE];
    uint8_t bLedON= 0;

    for(uint8_t x=0; x<lenght; x++)
    {
        buffer[x] = pgm_read_byte(&data[x]) * intensity / 100;
        /*buffer[x] = pgm_read_byte(&data[x]);
        if(buffer[x])
            bLedON = 1;*/
    }
    led_SendData(buffer, lenght);
    //SetPin(5, bLedON);
}


/*
SK6812 LED CHIP

0 code:

   0.3us +/- .15us
   --------
  /        \------------------/
              .9us +/- .15us


1 code:

   0.6us +/- .15us
   -------------
  /             \-------------/
                 .6us +/- .15us

   TH+TL: 1.25 us +/- .6us

Reset code:

   80 us
*/
//
// Version valid for CPU @ 8 MHz only
void led_SendData( uint8_t * dataPtr, uint8_t uDataLen)
{
  uint8_t u8Flags = SREG;

  asm (
    "cli \n"

    "mov r26, %A0 \n" // load X low dataPtr
    "mov r27, %B0 \n" // load X high dataPtr
    "mov r2, %1 \n" // dataLen
"loop_bytes: \n"
    "ld r0, x+ \n"
    "ldi r17, 8 \n"
"loop_bits: \n"
    "rol r0 \n"
    "sbi " _STR( LED_CTRL_PORT_ADDR) "," _STR( LED_CTRL_BIT_NO) "\n"
// "nop \n"
    "brcs bit_un \n"
// "nop \n"
"bit_un: \n"
    "cbi " _STR( LED_CTRL_PORT_ADDR) "," _STR( LED_CTRL_BIT_NO) "\n"

    "dec r17 \n"
    "brne loop_bits \n"

    "dec r2 \n"
    "brne loop_bytes \n"

      : : "r" (dataPtr),  "r" (uDataLen)
      : "r0", "r2", "r26", "r27", "r17"
    );

   if( u8Flags & (1<<7))
     asm ( "sei\n");

   /*asm(
    "ldi r16, " _STR(80 * CPU_CLOCK_MHZ /6) " \n" // 80 uSec (6 clocks per loop)
"delay_sync: \n"
    "dec r16 \n"
    "nop \n"
    "nop \n"
    "nop \n"
    "brne delay_sync \n"
    : : : "r16"
    );*/

}

// N81 (no parity, 8 bit, 1 stop), 230400 bps (4.34 us/bit)
// CPU @ 1 MHz (+/- 2%)
void uart_vPutDataBuff_230k( uint8_t * pData, uint8_t len)
{
   asm(
     "mov r2, %1 \n" // len
     "or r2, r2 \n"
     "breq done1 \n"

     "mov r26, %A0 \n" // load X.low pData
     "mov r27, %B0 \n" // load X.high pData
     "in r4, " _STR( __SREG) "\n" // save interrupt flag
     "ld r0, x+ \n" // 2
"loop1: \n"
     "cli \n"
// start bit:
     "cbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // 0
     "in r3, " _STR( UART_TX_PORT_ADDR) "\n" // 1

#define _ONE_BIT(n) \
     "bst r0, " _STR( n) " \n" \
     "bld r3, " _STR( UART_TX_BN) "\n" \
     "out " _STR( UART_TX_PORT_ADDR) ", r3 \n"

// data bits:
     _ONE_BIT( 0) "adiw r24, 0\n" // 6
     _ONE_BIT( 1) "nop\n"         // 10
     _ONE_BIT( 2) "nop\n"         // 14
     _ONE_BIT( 3) "adiw r24, 0\n" // 19
     _ONE_BIT( 4) "nop\n"         // 23
     _ONE_BIT( 5) "nop\n"         // 27
     _ONE_BIT( 6) "adiw r24, 0\n" // 32
     _ONE_BIT( 7) "nop\n"         // 35

     "ld r0, x+ \n"      // 37
// stop bit:
     "sbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // 39
     "out " _STR( __SREG) ", r4\n" // restore interrupt flag
     "dec r2 \n"
     "brne loop1 \n" // 46 us/byte = 21739 bytes/sec
"done1:\n"

   : : "r" (pData), "r" (len) : "r0", "r2", "r3", "r4", "r26", "r27");
} // uart_vPutDataBuff()

// N81 (no parity, 8 bit, 1 stop), 460800 bps (2.17 us/bit)
// @ 8 MHz CPU
void uart_vPutDataBuff_460k( uint8_t * pData, uint8_t len)
{
   asm(
     "mov r2, %1 \n" // len
     "or r2, r2 \n"
     "brne continue_460 \n"
     "rjmp done1_460 \n"

"continue_460: \n"
     "mov r26, %A0 \n" // load X.low pData
     "mov r27, %B0 \n" // load X.high pData
     "in r4, " _STR( __SREG) "\n" // save interrupt flag
     "ld r0, x+ \n" // 2
"loop1_460: \n"
     "cli \n"
// start bit:
     "cbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // 0
     "in r3, " _STR( UART_TX_PORT_ADDR) "\n" // 1

#define ONE_BIT(n) \
     "lpm r6, z \n" \
     "lpm r6, z \n" \
     "lpm r6, z \n" \
     "lpm r6, z \n" \
     "nop \n"      \
     "bst r0, " _STR( n) " \n" \
     "bld r3, " _STR( UART_TX_BN) "\n" \
     "out " _STR( UART_TX_PORT_ADDR) ", r3 \n" // 16 clocks

// data bits:
     ONE_BIT( 0) // 17
     ONE_BIT( 1) "adiw r24, 0\n" // 35
     ONE_BIT( 2) "nop \n"// 52
     ONE_BIT( 3) "nop\n" // 69
     ONE_BIT( 4) "adiw r24, 0\n" // 87
     ONE_BIT( 5) "nop \n" // 104
     ONE_BIT( 6) "adiw r24, 0\n" // 122
     ONE_BIT( 7) "nop \n" // 139
     "lpm r6, z \n"
     "lpm r6, z \n"
     "lpm r6, z \n"
     "lpm r6, z \n"
     "nop \n"

     "ld r0, x+ \n"
// stop bit:
     "sbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // 156
     "out " _STR( __SREG) ", r4\n" // restore interrupt flag
     "lpm r6, z \n"
     "lpm r6, z \n"
     "dec r2 \n"
     "breq done1_460 \n"
     "rjmp loop1_460 \n" // 46 us/byte = 21739 bytes/sec
"done1_460:\n"

   : : "r" (pData), "r" (len) : "r0", "r2", "r3", "r4", "r6", "r26", "r27");
} // uart_vPutDataBuff()



// N81 (no parity, 8 bit, 1 stop), 460800 bps (2.17 us/bit)
// @ 8 MHz CPU
void uart_vPutDataBuff_460k_v2( uint8_t * pData, uint8_t len)
{
   asm(
     "mov r2, %1 \n" // len
     "or r2, r2 \n"
     "breq done1_460_v2 \n"

     "mov r26, %A0 \n" // load X.low pData
     "mov r27, %B0 \n" // load X.high pData
     "in r4, " _STR( __SREG) "\n" // save interrupt flag
"loop1_460_v2: \n"
     "ld r0, x+ \n" // 2
     "cli \n"
// start bit:
     "cbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // 0
     "in r3, " _STR( UART_TX_PORT_ADDR) "\n" // 1
     "lpm r6, z \n"
     "lpm r6, z \n"
     "lpm r6, z \n"
     "lpm r6, z \n"

     "ldi r16, 8 \n"
"bit_loop_460_v2: \n"
     "bst r0, 0 \n"
     "bld r3, " _STR( UART_TX_BN) "\n"
     "out " _STR( UART_TX_PORT_ADDR) ", r3 \n" // start to first bit: 17 clocks
     "ror r0 \n"

     "bst r16, 0 \n"
     "brts odd_460_v2 \n"
"odd_460_v2: \n" // 6 or 7
     "lpm r6, z \n"
     "lpm r6, z \n"
     "adiw r24, 0\n"

     "dec r16 \n"
     "brne bit_loop_460_v2 \n" // bit loop = 17/18 clocks

     "nop \n"
// stop bit:
     "sbi " _STR( UART_TX_PORT_ADDR) ", " _STR( UART_TX_BN) "\n" // last bit to stop : 17 clocks
     "out " _STR( __SREG) ", r4\n" // restore interrupt flag
     "lpm r6, z \n"
     "lpm r6, z \n"
     "lpm r6, z \n"
     "nop \n"
     "dec r2 \n"
     "brne loop1_460_v2 \n"
"done1_460_v2:\n"

   : : "r" (pData), "r" (len) : "r0", "r2", "r3", "r4", "r6", "r16", "r26", "r27");
} // uart_vPutDataBuff_460k_v2()

void uart_vPutDataBuff( uint8_t * pData, uint8_t len)
{
    //uart_vPutDataBuff_460k(pData, len);
    uart_vPutDataBuff_460k_v2(pData, len);
    //uart_vPutDataBuff_230k(pData, len);
}

void put_uart_CarriageReturn()
{
    put_uart_FLASH_string(PSTR("\n\r"));
}

void EvalTiming()
{
    eeprom_Write(0, (uint8_t)CALIBRATION_OSCCAL);
    eeprom_Write(1,  ~(uint8_t)CALIBRATION_OSCCAL);

   cli();
   OSCCAL = CALIBRATION_OSCCAL; // 22 degre C, 5 Volt, 1.00 MHz
   asm(
    "sbi " _STR(UART_TX_PORT_ADDR) "," _STR( UART_TX_BN) "\n"
"ici:\n"
    "sbi " _STR(UART_TX_PORT_ADDR) "," _STR( UART_TX_BN) "\n"
    "nop\n nop\n"
    "nop\n nop\n"
    "nop\n nop\n"
    "nop\n nop\n"
    "cbi " _STR(UART_TX_PORT_ADDR) "," _STR( UART_TX_BN) "\n"
    "rjmp ici\n" // 10 clocks high pin
   );
}

void put_uart_string(char* str)
{
    uart_vPutDataBuff((uint8_t*)str, strlen(str));
}

void put_uart_FLASH_string(const char* str)
{
    uint8_t data;

    while((data = pgm_read_byte(str++)) != '\0')
        uart_vPutDataBuff(&data,1);
}

uint8_t strlen(char *str)
{
    uint8_t x;
    for(x = 0; str[x] != '\0'; x++);
    return x;
}

char *FormatNumber(uint32_t number, uint8_t largeur_min, int8_t base, char *buffer)
{
    uint8_t x=0,y;
    int8_t num;
    char bNeg = 0;
    uint8_t ret=base;

    if( base == -10) { // signed decimal number..
       base = 10;
       if( (int32_t)number < 0) {
          number = -number;
          bNeg = 1;
       }
    }
    do{
        num = (number%base);
        if(num > 9)
            num += 'A' - 10;
        else
            num += '0';
        buffer[x++] = num;
        number = number/base;
    } while(number != 0);

    while(x<largeur_min)
        buffer[x++] = '0';

    if( bNeg)
       buffer[x++] = '-';
    else
        if((int8_t)ret<0)
        buffer[x++] = '+';

    buffer[x] = 0;
    ret = x;

    for(uint8_t u=0; x-- > u; u++)
    {
        y = buffer[u];
        buffer[u] = buffer[x];
        buffer[x] = y;
    }

    return buffer;
}

void GeneralInit()
{
    uint8_t osccal = eeprom_Read8(0);

    if(osccal == (uint8_t)~eeprom_Read8(1))
        OSCCAL = osccal; // tune internal oscillator for 1.00 MHz @ 22 degre C, 5 Volt CHIP 8mhz
    else
        OSCCAL = CALIBRATION_OSCCAL; // tune internal oscillator for 1.00 MHz @ 22 degre C, 5 Volt 1mhz chip

    ConfigureTimer(TIMER_INTERRPUT_OVF, TIMER0);

#ifdef STARTUP_TIMER_CLOCK
    SetTimerClock(TIMER0, STARTUP_TIMER_CLOCK);
#else
    SetTimerClock(TIMER0, TIMER0_CLOCK_1024);
#endif // STARTUP_TIMER_CLOCK

#ifdef STARTUP_ADC_VREF
#ifdef STARTUP_ADC_CLOCK
    ConfigureADC(STARTUP_ADC_VREF, STARTUP_ADC_CLOCK);
#else
    ConfigureADC(STARTUP_ADC_VREF, ADC_CLOCK_32);
#endif // STARTUP_ADC_CLOCK

#else
#ifdef STARTUP_ADC_CLOCK
    ConfigureADC(ADC_VREF_VCC, STARTUP_ADC_CLOCK);
#else
    ConfigureADC(ADC_VREF_VCC, ADC_CLOCK_32);
#endif // STARTUP_ADC_CLOCK
#endif // STARTUP_ADC_VREF
    sei();
}

/*
void SetPWM(uint8_t pin, uint8_t pwm)
{
    if(bTimer_16[pinMap[pin].timer])
    {
        if((1<<pinMap[pin].timer_reg_msb)&0x80)
            _SFR_MEM16(timerRegAddr[pinMap[pin].timer].OCRxA) = (uint16_t)pwm*256;
        else
            _SFR_MEM16(timerRegAddr[pinMap[pin].timer].OCRxB) = (uint16_t)pwm*256;
    }
    else
        if((1<<pinMap[pin].timer_reg_msb)&0x80)
                REG(timerRegAddr[pinMap[pin].timer].OCRxA) = pwm;
            else
                REG(timerRegAddr[pinMap[pin].timer].OCRxB) = pwm;
}*/

void TogglePin(uint8_t pin)
{
    _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_in))) = 1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07);
}

void ConfigureTimer(uint8_t mode, uint8_t timer)
{
    switch(mode)
    {
        case TIMER_BOTTOM_MAX:
            //_SFR_MEM8(timerRegAddr[timer].TCCRxA) &= ~((1<<WGMxy[timer][1]) | (1<<WGMxy[timer][0]));
            //_SFR_MEM8(timerRegAddr[timer].TCCRxB) &= ~(1<<WGMxy[timer][2]);
            break;
        case FAST_PWM:
            if(timer == TIMER1) // 16 bit timer
            {
                //_SFR_MEM8(timerRegAddr[TIMER1].TCCRxA) |= ((1<<WGM11) | (pin != NO_PIN ? (1<<pinMap[pin].timer_reg_msb) : 0));
                //_SFR_MEM8(timerRegAddr[TIMER1].TCCRxA) &= ~(1<<WGMxy[timer][0]);
                _SFR_MEM8(pgm_read_byte(&(timerRegAddr[TIMER1].TCCRxB))) |= ((1<<pgm_read_byte(&(WGMxy[timer][2]))) | (1<<pgm_read_byte(&(WGMxy[timer][3]))));
                _SFR_MEM16(0x86)= 0xffff; // ICR1
            }
            else
                //REG(timerRegAddr[timer].TCCRxA) |= ((1<<WGM01) | (1<<WGM00) | (pin != NO_PIN ? (1<<pinMap[pin].timer_reg_msb) : 0));
                //REG(timerRegAddr[timer].TCCRxB) &= ~(1<<WGMxy[timer][2]);
                REG(pgm_read_byte(&(timerRegAddr[timer].TCCRxA))) |= 1<<pgm_read_byte(&(WGMxy[timer][1])) | 1<<pgm_read_byte(&(WGMxy[timer][0]));
            break;
        case TIMER_INTERRPUT_OVF:
            //REG(timerRegAddr[timer].TCCRxA) &= ~((1<<WGMxy[timer][0]) | (1<<WGMxy[timer][0]));
            //REG(timerRegAddr[timer].TCCRxB) &= ~(1<<WGMxy[timer][2]);
            REG(pgm_read_byte(&(timerRegAddr[timer].TIMSKx))) |= (1<<TOIE0);
            break;
        case PHASE_CORRECT_PWM:
            REG(pgm_read_byte(&(timerRegAddr[timer].TCCRxB))) |= 1<<pgm_read_byte(&(WGMxy[timer][0]));
        default:
            break;
    }
}
// WGMxy
// pgm_read_byte(&(WGMxy[timer][2]))
void SetTimerClock(uint8_t timer, uint8_t clock)
{
    //REG(timerRegAddr[timer].TCCRxB) &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
    //REG(timerRegAddr[timer].TCCRxB) |= clock;
    REG(pgm_read_byte(&(timerRegAddr[timer].TCCRxB))) |= clock;
}

void WritePort(uint8_t port, uint8_t data)
{
    _SFR_IO8(pgm_read_byte(&(portDef[port].data_out))) = data;
}

void SetPin(uint8_t pin, uint8_t state) // write pin
{
    uint8_t port = _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_out))) & ~(1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07));

    if(state)
        _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_out))) = port | (1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07));
    else
        _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_out))) = port;
}

void ConfigurePin(uint8_t pin, uint8_t config) // configure pin as INPUT || OUTPUT || INPUT_PULLUP || PWM
{
    _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_out))) &= ~(1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07));
    _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].dir))) &= ~(1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07));

    if(config == OUTPUT)
        _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].dir))) |= 1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07);
    if(config == INPUT_PULLUP)
        _SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_out))) |= 1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07);
}

bool ReadPin(uint8_t pin) // read pin state
{
    return (_SFR_IO8(pgm_read_byte(&(portDef[pgm_read_byte(&(pin_port_bit[pin])) >> 4].data_in))) & (1 << (pgm_read_byte(&(pin_port_bit[pin])) & 0x07))) ? 1:0;
}

uint16_t ReadADC(uint8_t pin) // read ADC. Returns 0 to 1023
{
    if(pin != (ADMUX & 0x0f))
    {
        ADMUX &= 0xf0;
        ADMUX |= pin;

        if(pin == ADC_1v1_REF_PIN)
            DelayMs(2);
    }

    ADCSRA |= 0b01000000; // start conversion
    while(ADCSRA & 0b01000000); // wait until conversion is done

    return ADCL | (ADCH<<8);
}

void ConfigureADC(uint8_t vRef, uint8_t clock) // configure ADC voltage reference and clock source
{
    cli();
    ADMUX = vRef << 6;
    ADCSRA = 0x80 | clock; // set clock
    sei();
}

void DelayMs(uint16_t time)
{
    uint32_t time_enter = ReadTimer0();
    uint32_t timer_stop_tick = (uint32_t)CPU_CLOCK_MHZ * time * 1000 / TIMER0_EFFECTIVE_PRESCALE;
    while((ReadTimer0()- time_enter) < timer_stop_tick);
}

void DelayUs(uint16_t time)
{
    uint32_t time_enter = ReadTimer0();
    uint32_t timer_tick = (uint32_t)CPU_CLOCK_MHZ * time;
    while((ReadTimer0() - time_enter) < timer_tick);
}

uint32_t ReadTimer0()
{
    uint8_t timer_count, timer_count_1, saveSREG=SREG, ovf_flag;
    uint16_t ovf_tmp;

    cli();

    ovf_tmp = timer_0_overflow_count;
    timer_count = TCNT0;
    ovf_flag = TIFR & 2;
    timer_count_1 = TCNT0;

    SREG=saveSREG;
    if(ovf_flag)
    {
        ovf_tmp++;

        if(timer_count_1 < timer_count)
            timer_count = timer_count_1;
    }
    return ((uint32_t)ovf_tmp << 8) + timer_count;
}

ISR(TIM0_OVF_vect)
{
    timer_0_overflow_count++;
}
