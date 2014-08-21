#include <avr/io.h>
#include <util/delay.h>
#include "usi_i2c_master.h"

#define SPICS PB1
#define SPIDATA PB4
#define SPICLK PB3
#define SPIPORT PORTB
#define SPIDDR DDRB
#define SPIIN PINB

#define RTC_SEC 0x81
#define RTC_MIN 0x83
#define RTC_HOUR 0x85

#define IO_IODIRA 0x00u
#define IO_IODIRB 0x01u
#define IO_IOPOLA 0x02u
#define IO_IOPOLB 0x03u
#define IO_GPINTENA 0x04u
#define IO_GPINTENB 0x05u
#define IO_DEFVALA 0x06u
#define IO_DEFVALB 0x07u
#define IO_INTCONA 0x08u
#define IO_INTCONB 0x09u
#define IO_IOCON 0x0au
#define IO_GPPUA 0x0cu
#define IO_GPPUB 0x0du
#define IO_INTFA 0x0eu
#define IO_INTFB 0x0fu
#define IO_INTCAPA 0x10u
#define IO_INTCAPB 0x11u
#define IO_GPIOA 0x12u
#define IO_GPIOB 0x13u
#define IO_OLATA 0x14u
#define IO_OLATB 0x15u

uint8_t bcd_decode(uint8_t value)
{
    return (value & 0x0f) + ((value & 0xf0) >> 4) * 10;
}

void rtc_init()
{
	SPIDDR |= (1 << SPICLK) | (1 << SPICS);
	SPIPORT |= (1 << SPIDATA);
}

void rtc_clk_pulse()
{
    SPIPORT |= (1 << SPICLK);
    _delay_us(100);
    SPIPORT &= ~(1 << SPICLK);
    _delay_us(100);
}

void rtc_begin()
{
    SPIPORT |= (1 << SPICS);
}

void rtc_end()
{
    SPIPORT &= ~(1 << SPICS);
}

void rtc_write_octet(uint8_t value)
{
    SPIDDR |= (1 << SPIDATA);
    
    for (uint8_t i = 0; i < 8; i++)
    {
        if (value & (1 << i))
            SPIPORT |= (1 << SPIDATA);
        else
            SPIPORT &= ~(1 << SPIDATA);

        rtc_clk_pulse();
    }

    SPIDDR &= ~(1 << SPIDATA);
    SPIPORT |= (1 << SPIDATA);
}

uint8_t rtc_read_octet()
{
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
       value |= ((SPIIN & (1 << SPIDATA)) >> SPIDATA) << i;
       rtc_clk_pulse();
    }

    return value;
}

uint8_t rtc_octet_transaction(uint8_t cmd, uint8_t mask)
{
    rtc_begin();
    rtc_write_octet(cmd);
    uint8_t value = rtc_read_octet();
    rtc_end();
    return bcd_decode(value & mask);
}

#define IO_ADDRESS 0x20u

char io_set_reg(uint8_t reg, uint8_t value)
{
	uint8_t buf[3] = {IO_ADDRESS << 1, reg, value};
	return USI_I2C_Master_Start_Transmission(buf, 3, 1);
}

char io_get_reg(uint8_t reg, uint8_t* value)
{
	uint8_t buf[2] = {IO_ADDRESS << 1, reg};
	char e = USI_I2C_Master_Start_Transmission(buf, 2, 0);
	if (!e) return e;
	buf[0] |= 0x1;
	e = USI_I2C_Master_Start_Transmission(buf, 2, 1);
	if (!e) return e;
	*value = buf[1];
	return e;
}

typedef union
{
	struct
	{
		unsigned sec : 6;
		unsigned min : 6;
		unsigned hour : 4;
	};
	
	struct
	{
		uint8_t latchA;
		uint8_t latchB;
	};
} Output;

int main()
{
	PORT_USI |= (1 << PIN_USI_SDA) | (1 << PIN_USI_SCL);
	DDR_USI &= ~((1 << PIN_USI_SDA) | (1 << PIN_USI_SCL));
	
	USIDR = 0xFF;
	
	USICR = (1 << USIWM1) | (1 << USICS1) | (1 << USICLK);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
	
	rtc_init();
	
	_delay_ms(100);
	io_set_reg(IO_IODIRA, 0);
	io_set_reg(IO_IODIRB, 0);
	
	while(1)
	{
		_delay_ms(20);
		
		uint8_t sec = rtc_octet_transaction(RTC_SEC, 0x7f);
		uint8_t min = rtc_octet_transaction(RTC_MIN, 0x7f);
		uint8_t hour = rtc_octet_transaction(RTC_HOUR, 0x3f);
		
		if (hour >= 12) hour -= 12;
		if (hour == 0) hour = 12;
		
		Output output = { .sec = sec, .min = min, .hour = hour };
		
		io_set_reg(IO_OLATA, output.latchA);
		io_set_reg(IO_OLATB, output.latchB);
	}
}
