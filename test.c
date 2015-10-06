#include "main.h"
#include <stdint.h>

uint16_t sec = 0;
uint8_t num = 0;

void testDisplay()
{
    uint16_t n = 0;
    uint8_t a, b, c, d;

    if (sec == 200)
    {
        a = num;
        b = (num + 1) % 10;
        c = (num + 2) % 10;
        d = (num + 3) % 10;

        n = a * 1000 + b * 100 + c * 10 + d;
        inttoind(n);

        if (num == 9)
        {
            num = 0;
        }
        else
        {
            num++;
        }

        sec = 0;
    }
    else
    {
        sec++;
    }
}

void testButtons()
{
    uint16_t num = 0;

    if (buttons.direction_r)
        num = 1;

    if (buttons.down_r)
        num = 2;

    if (buttons.menu_r)
        num = 3;

    if (buttons.speed_r)
        num = 4;

    if (buttons.start_stop_r)
        num = 5;

    if (buttons.up_r)
        num = 6;

    inttoind(num);
}

void testMotor()
{
	setSpeed(127);	

    if (sec == 1000)
    {
        if (num)
        {
            rotateLeft();
   
            num = 0;
        }
        else
        {
            rotateRight();

            num = 1;
        }

        sec = 0;
    }
    else
    {
        sec++;
    }
}

void testLeds()
{
    if (sec == 200)
    {
        if (num)
        {
            ledOn();

            num = 0;
        }
        else
        {
            ledOff();

            num = 1;
        }

        sec = 0;
    }
    else
    {
        sec++;
    }
}

void testEncoder()
{
    inttoind(encoder.counter);
}

void testADC()
{
	inttoind(adcValue);
}	

void testAdcMotor()
{
	rotateLeft();
	setSpeed(adcValue);
	inttoind(adcValue);
}
