/*
 * намоточный станок
 *
 * динамическая индикация на прерывании
 * считывание кнопок в прерывании (в том же что и индикация)
 *
 * 2 ШИМ канала для задания скорости, работают по очереди в зависимости от направления вращения
 * тактирование от внетреннего генератора на 8 МГц
 * энкодер в прерывании индикации?
 *
 * придумать как все отображать на одном индикаторе с четырьмя сегнментами и без точки
 *
 * кнопка "меню" переключает разряды в индикаторе
 *
 * если скорость установлена в "0" то задание скорости от аналогового входа
 * так скорость устанавливается от 0 до 100 в процентах
 *
 * сохранять в еепром значение скорости, последнее количество витков, направление вращения
 *
 * подключение:
 * PC0 - ADC, задание скорости
 *
 * PC1 - данные сдвигового регистра
 * PC2 - тактовые импульсы сдвигового регистра
 * PC3 - сброс сдвигового регистра
 *
 * PC4 - светодиод
 * PC5 - светодиод
 *
 * PD0-PD3 катоды индикатора
 * PD0 - 4
 * PD1 - 3
 * PD2 - 2
 * PD3 - 1
 *
 * PD4, PD5 направление вращения
 *
 * PD6, PD7 энкодер
 *
 * PB1, PB2 скорость вращения, ШИМ
 *
 * PB0, PB3-PB7 кнопки
 *
 * функциональные блоки в ПО:
 * кнопки
 * индикатор
 * энкодер
 * мотор
 * аналоговый вход
 * системный таймер (динамическая индикация, считывание кнопок, энкодреа, запуск АЦП)
 *
 * кнопка "меню" работает как некий ресет, после запуска намотки можно останавливаться
 * и продолжать сколько угодно, чтобы выйти и начать заново нужно остановиться и нажать меню
 *
 * состояния конечного автомата:
 * начало (установка скорости, колечество витков, запись в еепром)
 * работа (двигатель вращается с нужной скоростью, считаются витки)
 * остановка (все остается в работе, можно продолжить намотку, вручную отмотать, изменить
 * направление намотки)
 * конец (все отработало, по сути после только переход в начало)
*/

#include "main.h"

#include <stdint.h>
#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

volatile uint8_t timer;

uint8_t adcValue;

uint8_t* EEPROM_SPEED  = (uint8_t*)0;
uint8_t* EEPROM_VAL_1  = (uint8_t*)1;
uint8_t* EEPROM_VAL_2  = (uint8_t*)2;
uint8_t* EEPROM_K      = (uint8_t*)3;
uint8_t* EEPROM_DIR    = (uint8_t*)4;

struct Buttons buttons;
struct Encoder encoder;
enum State state;
struct Motor motor;
struct Display display;
struct BlinkMenu blinkMenu;

int main()
{
    uint8_t ledBlink = 0;
    uint8_t toInd = VALUE;
    uint16_t compValue = 0;
    uint16_t encoderInd = 0;

    initADC();
    initEncoder();
    initDisplay();
    initMotor();
    initButtons();
    initLeds();

    // таймер
    TCCR0 |= (0 << CS02) | (1 << CS01) | (0 << CS00);	// пределение на 8, прерывание примерно через 3,9 мс.
    TIMSK |= (1 << TOIE0);	// прерывания от таймеров

    // вычитать данные из еепром
    motor.direction = eeprom_read_byte(EEPROM_DIR);
    motor.speed = eeprom_read_byte(EEPROM_SPEED);

    encoder.endValue = eeprom_read_byte(EEPROM_VAL_1);
    encoder.endValue |= eeprom_read_byte(EEPROM_VAL_2) << 8;

    if (encoder.endValue > 9999) {
        encoder.endValue = 0;
    }

    encoder.k = eeprom_read_byte(EEPROM_K);

    blinkMenu.timer = 0;
    blinkMenu.catode = 0;
    blinkMenu.dot = 0;

    setSpeed(0);

    if (motor.direction) {
        rotateLeft();
    } else {
        rotateRight();
    }

    blinkMenu.isSpeed = 0;

    sei();

    for(;;) {
        if (!buttons.timer) {
            readButtons();
        }

        if (!encoder.timer) {
            encoder.timer = 5;
        }

        readADC();

        encoderInd = encoder.counter / encoder.k;

        switch (state) {
        case READY: {
            ledOff();

            // установка скорости и количества витков
            if (buttons.menu) {
                if (blinkMenu.catode == 4) {
                    blinkMenu.catode = 0;
                } else {
                    blinkMenu.catode++;
                }
            }

            if (buttons.speed) {
                blinkMenu.catode = 0;

                if (buttons.menu) {
                    toInd = COEF;
                } else {
                    if (blinkMenu.isSpeed) {
                        blinkMenu.isSpeed = 0;
                        toInd = VALUE;
                    } else {
                        blinkMenu.isSpeed = 1;
                        toInd = SPEED;
                    }
                }
            }

            if (blinkMenu.catode && (buttons.up || buttons.down)) {
                uint16_t tmp;
                uint8_t seg[4];
                uint8_t i;

                switch (toInd) {
                case VALUE: {
                    tmp = encoder.endValue;
                    break;
                }
                case SPEED: {
                    tmp = motor.speed;
                    break;
                }
                case COEF: {
                    tmp = encoder.k;
                    break;
                }
                default:
                    break;
                }

                // узнаем количество тисяч
                for (i=0; tmp >= 1000; i++)
                    tmp -= 1000;
                seg[3] = i;

                // сотен
                for (i=0; tmp >= 100; i++)
                    tmp -= 100;
                seg[2] = i;

                // десятков
                for (i=0; tmp >= 10; i++)
                    tmp -= 10;
                seg[1] = i;

                // единицы сразу переносим
                seg[0] = tmp;

                if (buttons.up) {
                    if (seg[blinkMenu.catode - 1] < 9) {
                        seg[blinkMenu.catode - 1]++;
                    }
                } else {
                    if (buttons.down) {
                        if (seg[blinkMenu.catode - 1] > 0) {
                            seg[blinkMenu.catode - 1]--;
                        }
                    }
                }

                tmp = seg[0] + seg[1] * 10 + seg[2] * 100 + seg[3] * 1000;


                switch (toInd) {
                case VALUE: {
                    encoder.endValue = tmp;
                    break;
                }
                case SPEED: {
                    if (tmp > 100) {
                        tmp = 100;
                    }

                    motor.speed = tmp;
                    break;
                }
                case COEF: {
                    encoder.k = tmp;
                    break;
                }
                default:
                    break;
                }
            }

            switch (toInd) {
            case VALUE: {
                inttoind(encoder.endValue);
                blinkMenu.dot = 1;
                break;
            }
            case SPEED: {
                inttoind(motor.speed);
                blinkMenu.dot = 2;
                break;
            }
            case COEF: {
                inttoind(encoder.k);
                blinkMenu.dot = 3;
                break;
            }
            default:
                break;
            }

            //break; он здесь не нужен, чтобы не повторять функционал
        }
        case START: {
            if (buttons.direction) {
                motor.direction ^= 1;

                if (motor.direction) {
                    rotateLeft();
                } else {
                    rotateRight();
                }
            }

            if (buttons.start_stop) {
                if (state == READY) {       // save config
                    eeprom_update_byte(EEPROM_DIR, motor.direction);
                    eeprom_update_byte(EEPROM_SPEED, motor.speed);

                    eeprom_update_byte(EEPROM_K, encoder.k);

                    eeprom_update_byte(EEPROM_VAL_1, encoder.endValue & 0xFF);
                    eeprom_update_byte(EEPROM_VAL_2, encoder.endValue >> 8);

                    encoder.counter = 0;

                    compValue = encoder.endValue * encoder.k;
                }

                state = WINDING;

                ledOn();

                toInd = VALUE;
                blinkMenu.catode = 0;
                blinkMenu.dot = 0;
                blinkMenu.isSpeed = 0;

                if (motor.speed != 0) {
                    setSpeed(motor.speed + 27); // потому что максимум что задаем это 100 %, дальше сдвиг на 1 и надо получить 255, если задали 1 то не страшно
                }
            }

            if (state == START) {
                // тут мигать светодиодом
                if (ledBlink == 200) {
                    ledBlink = 0;
                    PORTD ^= (1 << PORTD7);
                } else {
                    ledBlink++;
                }

                inttoind(encoderInd);

                // не проверенное условие, могут быть баги
                if (buttons.menu) {
                    state = READY;
                }
            }

            break;
        }
        case WINDING: {
            if (motor.speed == 0) {
                uint8_t speed = adcValue >> 1;

                setSpeed(speed);
            }

            if (encoder.counter >= compValue) {
                state = READY;

                setSpeed(0);
            }

            if (buttons.start_stop) {
                state = START;

                setSpeed(0);
            }

            inttoind(encoderInd);

            break;
        }
        case ACCEL: {
            break;
        }
        default : {
            break;
        }
        }

        buttons.direction = 0;
        buttons.start_stop = 0;
        buttons.down = 0;
        buttons.menu = 0;
        buttons.speed = 0;
        buttons.up = 0;

        if (timer) {
//            testDisplay();
//            testButtons();
//            testLeds();
//            testEncoder();
//            testADC();
//            testMotor();

            view_ind();

            timer = 0;
        }
    }

    return 0;
}

// АЦП, нулевой канал, выравнивание влево, берем только старшие биты
void initADC()
{
    // АЦП
    ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << ADLAR);		// выравнивание влево
    ADCSR = (1 << ADEN) | (1 << ADFR) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // включаем, частота на минимум
    ADCSRA |= (1 << ADSC);
}

void readADC()
{
    adcValue = ADCH;
}

// энкодер будет по прерыванию
void initEncoder()
{
    PORTD &= ~((1 << PORTD2) | (1 << PORTD3));
    DDRD &= ~((1 << DDD2) | (1 << DDD3));

    MCUCR |= (1 << ISC01);
    GICR |= (1 << INT0);
}

// инициализация портов и структур
void initDisplay()
{
    // катоды
    PORTD &= ~((1 << PORTD0) | (1 << PORTD1));
    PORTC &= ~((1 << PORTD2) | (1 << PORTD3));
    DDRD |= (1 << DDD0) | (1 << DDD1);
    DDRC |= (1 << DDD2) | (1 << DDD3);

    // сдвиговый регистр
    PORTC &= ~((1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3));
    DDRC |= (1 << DDC1) | (1 << DDC2) | (1 << DDC3);

    PORTC |= (1 << PORTC2);     // пока /CLR всегда 1
}

// инициализация ШИМ и пинов которые для мотора
void initMotor()
{
    PORTD &= ~((1 << PORTD5) | (1 << PORTD6));
    DDRD |= (1 << DDD5) | (1 << DDD6);

    PORTB &= ~((1 << PORTB1) | (1 << PORTB2));
    DDRB |= (1 << DDB1) | (1 << DDB2);

    // ШИМ нормальный режим, 8 бит
//    TCCR1A = (1 << COM1A1) | (0 << COM1A0) | (0 << WGM11) | (1 << WGM10) | (1 << COM1B1) | (0 << COM1B0);
    TCCR1A = (0 << WGM11) | (1 << WGM10);
    TCCR1B = (0 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);

    stopMotor();    // на всякий случай
}

void stopMotor()
{
    TCCR1A &= ~((1 << COM1A1) | (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0));
    PORTD &= ~((1 << PORTD5) | (1 << PORTD6));
    PORTB &= ~((1 << PORTB1) | (1 << PORTB2));
}

void rotateLeft()
{
    stopMotor();

    PORTD |= (1 << PORTD5);
    TCCR1A |= (1 << COM1B1) | (0 << COM1B0);
}

void rotateRight()
{
    stopMotor();

    PORTD |= (1 << PORTD6);
    TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
}

void setSpeed(uint8_t speed)
{
    speed = speed << 1;

    OCR1BL = speed;
    OCR1AL = speed;
}

// просто инициализация портов
void initButtons()
{
    DDRB &= ~((1 << DDB0) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5) | (1 << DDB6) | (1 << DDB7));
    PORTB |= (1 << PORTB0) | (1 << PORTB3) | (1 << PORTB4) | (1 << PORTB5) | (1 << PORTB6) | (1 << PORTB7);
}

void initLeds()
{
    PORTD &= ~(1 << PORTD7);
    DDRD |= (1 << DDD7);
}

void ledOn()
{
    PORTD |= (1 << PORTD7);
}

void ledOff()
{
    PORTD &= ~(1 << PORTD7);
}

void readButtons(void)
{
    buttons.direction_r = 0;
    buttons.down_r = 0;
    buttons.menu_r = 0;
    buttons.speed_r = 0;
    buttons.start_stop_r = 0;
    buttons.up_r = 0;

    if (~PINB & (1 << KEY_MENU))
        buttons.menu_r = 1;

    if (~PINB & (1 << KEY_START_STOP))
        buttons.start_stop_r = 1;

    if (~PINB & (1 << KEY_DIRECTION))
        buttons.direction_r = 1;

    if (~PINB & (1 << KEY_UP))
        buttons.up_r = 1;

    if (~PINB & (1 << KEY_DOWN))
        buttons.down_r = 1;

    if (~PINB & (1 << KEY_SPEED))
        buttons.speed_r = 1;


    if (!buttons.direction_p && buttons.direction_r)
        buttons.direction = 1;

    if (!buttons.down_p && buttons.down_r)
        buttons.down = 1;

    if (!buttons.menu_p && buttons.menu_r)
        buttons.menu = 1;

    if (!buttons.speed_p && buttons.speed_r)
        buttons.speed = 1;

    if (!buttons.start_stop_p && buttons.start_stop_r)
        buttons.start_stop = 1;

    if (!buttons.up_p && buttons.up_r)
        buttons.up = 1;


    buttons.direction_p = buttons.direction_r;
    buttons.down_p = buttons.down_r;
    buttons.menu_p = buttons.menu_r;
    buttons.speed_p = buttons.speed_r;
    buttons.start_stop_p = buttons.start_stop_r;
    buttons.up_p = buttons.up_r;

    buttons.timer = 25;
}

// Индикация, программный таймер
ISR (TIMER0_OVF_vect)
{
    timer = 1;

    buttons.timer--;

    encoder.timer--;
}

ISR(INT0_vect)
{
    if (PIND & (1 << PIND3)) {
        encoder.counter++;
    } else {
        if (encoder.counter) {
            encoder.counter--;
        }
    }
}
