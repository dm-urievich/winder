#include <compat/deprecated.h>	// позволяет писать cbi (порт, бит)
#include <stdint.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include "main.h"

#define DATA PORTC3
#define CLK PORTC1
#define K4 PORTC4
#define K3 PORTC5
#define K2 PORTD1
#define K1 PORTD0

extern struct BlinkMenu blinkMenu;

uint8_t seg[4];		// массив того, что выводить на инд. (просто пихается как по spi)
uint8_t numofseg;	// текущий сегмент

uint8_t tableseg[10] = {(1 << 7) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1), // 0
                        (1 << 7) | (1 << 1), // 1
                        (1 << 5) | (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0), // 2
                        (1 << 7) | (1 << 5) | (1 << 3) | (1 << 1) | (1 << 0), // 3
                        (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0), // 4
                        (1 << 7) | (1 << 5) | (1 << 3) | (1 << 2) | (1 << 0), // 5
                        (1 << 7) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 0), // 6
                        (1 << 7) | (1 << 3) | (1 << 1), // 7
                        (1 << 7) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0), // 8
                        (1 << 7) | (1 << 5) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0), // 9
                       };

void inttoind(uint16_t n)
{
    uint8_t i;
    // можно попробовать сделать искуственно деление, как когда-то на ассемблере

    // узнаем количество тисяч
    for (i=0; n >= 1000; i++)
        n -= 1000;
    seg[3] = tableseg[i];

    // сотен
    for (i=0; n >= 100; i++)
        n -= 100;
    seg[2] = tableseg[i];

    // десятков
    for (i=0; n >= 10; i++)
        n -= 10;
    seg[1] = tableseg[i];

    // одиницы сразу переносим
    seg[0] = tableseg[n];
}

void view_ind(void)
{
    uint8_t i, p;
    switch (numofseg) {
    case  0 :
        PORTD &= ~(1 << K1);
        break;
    case  1 :
        PORTD &= ~(1 << K2);
        break;
    case  2 :
        PORTC &= ~(1 << K3);
        break;
    case  3 :
        PORTC &= ~(1 << K4);
        break;
    default :
        break;
    }

    numofseg++;

    if (numofseg == 4)
        numofseg = 0;

    p = seg[numofseg];

    if (blinkMenu.dot == numofseg + 1) {
        p |= (1 << 6);
    }

    for (i=0; i<8; i++) {
        // софтовый spi со старшего по младший бит
        if (p & 0b10000000)
            PORTC |= (1 << DATA);
        else PORTC &= ~(1 << DATA);

        PORTC |= (1 << CLK);

        p = p << 1;

        PORTC &= ~(1 << CLK);
    }

    if (blinkMenu.catode && (numofseg == (blinkMenu.catode - 1)))
    {
        if (!blinkMenu.timer)
        {
            blinkMenu.state = blinkMenu.state ? 0 : 1;

            blinkMenu.timer = 30;
        }
        else
        {
            blinkMenu.timer--;
        }
    }

    if ((numofseg != (blinkMenu.catode - 1)) || blinkMenu.state || blinkMenu.catode == 0)
    {
        switch (numofseg) {
        case 0 :
            PORTD |= (1 << K1);
            break;
        case 1 :
            PORTD |= (1 << K2);
            break;
        case 2 :
            PORTC |= (1 << K3);
            break;
        case 3 :
            PORTC |= (1 << K4);
            break;
        default :
            break;
        }
    }
}
