#include <avr/io.h>
#include <avr/iom8.h>
#include <avr/interrupt.h>

uint8_t readadc (uint8_t n)
{
	// очищаем значение текущего порта
	ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));

	ADMUX |= n;
	ADCSRA |= (1 << ADSC);

/*	while (!(ADCSR & (1 << ADIF))
		;
	ADCSR |= (1 << ADIF);
*/
    while (ADCSRA & (1 << ADSC))
        ;

	return ADCH;
}

uint8_t reeprom (uint16_t addr)
{
    while (EECR & (1 << EEWE))
        ;

	EEAR = addr;
	EECR |= (1 << EERE);

	return EEDR;
}

void weeprom (uint16_t addr, uint8_t data)
{
    while (EECR & (1 << EEWE))
        ;

	EEAR = addr;
	EEDR = data;
	cli();
	EECR |= (1 << EEMWE);
	EECR |= (1 << EEWE);
	sei();
}

ISR (BADISR_vect)
{
	reti ();
}
