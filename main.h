#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

void initADC(void);
void initEncoder(void);
void initDisplay(void);
void initMotor(void);
void initButtons(void);
void initLeds();

#define KEY_DIRECTION PINB0
#define KEY_DOWN PINB5
#define KEY_MENU PINB6
#define KEY_SPEED PINB3
#define KEY_START_STOP PINB4
#define KEY_UP PINB7

struct Buttons {
    unsigned menu:1;
    unsigned start_stop:1;
    unsigned direction:1;
    unsigned up:1;
    unsigned down:1;
    unsigned speed:1;

    unsigned menu_p:1;
    unsigned start_stop_p:1;
    unsigned direction_p:1;
    unsigned up_p:1;
    unsigned down_p:1;
    unsigned speed_p:1;

    unsigned menu_r:1;
    unsigned start_stop_r:1;
    unsigned direction_r:1;
    unsigned up_r:1;
    unsigned down_r:1;
    unsigned speed_r:1;

    volatile uint8_t timer;
};

extern struct Buttons buttons;

struct Encoder {
    volatile uint16_t counter;
    uint16_t endValue;
    uint16_t value;
    unsigned a:1;
    unsigned b:1;
	volatile uint8_t timer;
    uint8_t k;
};

extern struct Encoder encoder;

struct Motor {
    uint8_t speed;
    unsigned direction:1;
};

struct Display {
    uint16_t data;
};

struct BlinkMenu {
    uint8_t timer;
    uint8_t catode;
    uint8_t state;
    uint8_t isSpeed;
    uint8_t dot;        // точка на дисплее
};

enum State {
    READY,
    START,
    WINDING,
    ACCEL
};

enum ToInd {
    VALUE,
    SPEED,
    COEF
};

extern uint8_t adcValue;

void inttoind(uint16_t n);

void ledOn();
void ledOff();

void rotateLeft();
void rotateRight();
void stopMotor();
void setSpeed(uint8_t speed);

void readButtons();

void view_ind();

void readADC();

void testDisplay();
void testButtons();
void testLeds();
void testEncoder();
void testMotor();
void testADC();
void testAdcMotor();

#endif // MAIN_H

