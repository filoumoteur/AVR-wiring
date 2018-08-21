#include "wiring.h"
#include <stdint.h>

#include <avr/sfr_defs.h>
#include <avr/io.h>

#define LED 1
#define TX 3

int main()
{
    GeneralInit();

    ConfigurePin(pin[LED],OUTPUT);
    ConfigurePin(pin[TX], OUTPUT);
    //ConfigurePin(pin[12], INPUT);

    put_uart_string("hello world");

    char buffer[12];

    while(1)
    {
        //put_uart_string(FormatNumber(ReadTimer0(), 1, 10, buffer));
        //put_uart_string("\n\r");
        //uart_vPutDataBuff(string, 13);
        SetPin(pin[LED],1);
        DelayMs(500);
        SetPin(pin[LED],0);
        DelayMs(500);
    }
}

/*
int main()
{
    uint8_t x, increment=1;
    uint16_t wait_time;

    General_Init();

    ConfigurePin(LED, PWM);
    ConfigurePin(19, INPUT);

    /*while(1)
    {
        wait_time = ((uint32_t)ReadADC(5) * 5 / ADC_MAX_COUNT) + 3;
        if(increment)
        {
            SetPWM(LED,x);
            Delay_ms(wait_time);
            x++;
            if(x == 0)
            {
                x=255;
                increment = 0;
            }
        }
        else
        {
            SetPWM(LED,x);
            Delay_ms(wait_time);
            x--;
            if(x == 255)
            {
                x=0;
                increment = 1;
            }
        }
    }*/

    /*while(1)
    {
        wait_time = ReadADC(5) * 5 / ADC_MAX_COUNT;
        Delay_ms(wait_time);
        SetPWM(LED,x);
        x++;
    }

    return 0;
}*/
