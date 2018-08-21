#include <stdint.h>
#include <avr/pgmspace.h>

#define WIRING

#define STATIC_LED_BUFFER_SIZE 3

// bigger number means faster clock
#define CALIBRATION_OSCCAL 90

/* port manipulation */
    // used to define pin function with ConfigurePin()
#define OUTPUT 1 // configure pin as output
#define INPUT 2 // configure pin as input
#define INPUT_PULLUP 3 // configure pin as input ans activate 10K vcc pullup resistor
#define FAST_PWM 4 // configure pin as PWM. It does not configure corresponding timer clock
#define PHASE_CORRECT_PWM 5

#define PORTb 0
//#define PORTc 2
//#define PORTd 3
//#define PORTe 4
//#define PORTf 5
//#define PORTg 6

    // PORTx data register address
#define PORTb_addr 0x18

    // PORTx input pin address
#define PINb_addr 0x16

    // PORTx data direction register address
#define DDRb_addr 0x17

/* timer */
#define TIMER0 0 // 8 bit
#define TIMER1 1 // 8 bit

    // timers working mode
#define TIMER_BOTTOM_MAX 1 // normal counting from BOTTOM to MAX
#define TIMER_INTERRPUT_OVF 2 // interrupt from corresponding timer on overflow

    // clock source selection for timers
#define TIMER0_NO_CLOCK 0 // No clock
#define TIMER0_CLOCK_1 1
#define TIMER0_CLOCK_8 2
#define TIMER0_CLOCK_64 3
#define TIMER0_CLOCK_256 4
#define TIMER0_CLOCK_1024 5
#define TIMER0_CLOCK_T0_rising 6
#define TIMER0_CLOCK_T0_falling 7

#define TIMER1_NO_CLOCK 0 // No clock
#define TIMER1_CLOCK_1 1
#define TIMER1_CLOCK_2 2
#define TIMER1_CLOCK_4 3
#define TIMER1_CLOCK_16 4
#define TIMER1_CLOCK_32 5
#define TIMER1_CLOCK_64 6
#define TIMER1_CLOCK_128 7
#define TIMER1_CLOCK_256 8

/* ADC */
#define ADC_MAX_COUNT 1023 // ADC max return count

    // ADC voltage reference for ConfigureADC()
#define ADC_VREF_VCC 0 // chip VCC as ADC reference voltage
#define ADC_VREF_AREF_PIN 0b01 // ARef pin as ADC reference voltage
#define ADC_1v1_REF_PIN 12 // 0b1100 in ADMUX register
#define ADC_1v1_REF_VOLTAGE_mv 1100 // real Vbg voltage in mv

    // ADC clock source. Faster means less precision/repetability between two lectures
#define ADC_CLOCK_2 1 // CPU clock / 2
#define ADC_CLOCK_4 2 // CPU clock / 4
#define ADC_CLOCK_8 3 // CPU clock / 8
#define ADC_CLOCK_16 4 // CPU clock /16
#define ADC_CLOCK_32 5 // CPU clock / 32
#define ADC_CLOCK_64 6 // CPU clock / 64
#define ADC_CLOCK_128 7 // CPU clock / 128

/*#define ADC0_MUX 0
#define ADC1_MUX 1
#define ADC2_MUX 2
#define ADC3_MUX 3*/

#define _STR(n) __STR(n)
#define __STR(n) #n

#define SET_BIT( port, bitNo) asm( "sbi " _STR(port) "," _STR( bitNo) "\n" )
#define CLR_BIT( port, bitNo) asm( "cbi " _STR(port) "," _STR( bitNo) "\n" )

const uint8_t pin_port_bit[6] PROGMEM =
{
(PORTb << 4) | 5, // 0
(PORTb << 4) | 3, // 1
(PORTb << 4) | 4, // 2
(PORTb << 4) | 0, // 3
(PORTb << 4) | 1, // 4
(PORTb << 4) | 2, // 5
};

struct timDef
{
    uint8_t TCCRxA;
    uint8_t TCCRxB;
    uint8_t TCNTx;
    uint8_t OCRxA;
    uint8_t OCRxB;
    uint8_t TIMSKx;
    uint8_t TIFRx;
};

struct pDef
{
    uint8_t data_out;
    uint8_t dir;
    uint8_t data_in;
};

const static struct timDef timerRegAddr[2] PROGMEM ={
{0x2A,0x33,0x32,0x29,0x28,0x39,0x38}, // timer0
{0x30,0x2C,0x2F,0x2E,0x2B,0x39,0x38}, // timer1
}; // timers registers address

/*
const static uint16_t timer_clock_division_factor[3][8] = {
{0,1,8,64,256,1024,1,1},
{0,1,8,64,256,1024,1,1},
{0,1,8,32,64,128,256,1024},
};*/ // used to convert clock define to effective clock divider

const static struct pDef portDef[1] PROGMEM = {
    {PORTb_addr,DDRb_addr,PINb_addr},
};

const static uint8_t WGMxy[3][4] PROGMEM = {
{0,1,3,255}, // timer0
{0,1,3,4}, // timer1
{6,3,8,8}, // timer2
}; // WGMxy bit position un register for each timer

void DelayUs(uint16_t time);
void TogglePin(uint8_t pin);
void SetPin(uint8_t pin, uint8_t state); //DigitalWrite equivalent. Toggles pin to corresponding state
void ConfigurePin(uint8_t pin, uint8_t config); // PinMode equivalent. Configures pin as INPUT || OUTPUT || INPUT_PULLUP
bool ReadPin(uint8_t pin); // DigitalRead equivalent. Read digital IO pin state. Range 0 - 1
uint16_t ReadADC(uint8_t pin); // AnalogRead equivalent. Range from 0 to 1023
void ConfigureADC(uint8_t vRef, uint8_t clock); // Configures ADC voltage reference and clock source. Faster clock means less precision/repetability between two lectures but less time to read analog pin
uint32_t ReadTimer0(); // Millis flavour. Returns TIMER0 tick since powerup. Range 0-0xFFFFFFFF
void DelayMs(uint16_t time); // Delay equivalent
void ConfigureTimer(uint8_t mode, uint8_t timer); // configure timer to working mode
void SetPWM(uint8_t pin, uint8_t pwm); // analogWrite equivalent. Must export pin as PWM with ConfigureTimer first
void SetTimerClock(uint8_t timer, uint8_t clock); // set TIMERx clock
void GeneralInit(); // init timebase and ADC vRef to VCC
void uart_vPutDataBuff( uint8_t * pData, uint8_t len);
char *FormatNumber(uint32_t number, uint8_t largeur_min, int8_t base, char *buffer);
void EvalTiming();
uint8_t strlen(char *str);
void put_uart_string(char* str);
void put_uart_FLASH_string(const char* str);
void put_uart_CarriageReturn();
void led_SendData( uint8_t * dataPtr, uint8_t uDataLen);
void led_SendDataFLASH(const uint8_t *data, uint8_t lenght, uint8_t intensity = 255);
