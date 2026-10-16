
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/io.h>

#define CAN_CS 10
#define INA_ADDR 0x40

#define REG_CONFIG  0x00
#define REG_BUS     0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CAL     0x05

#define INA_CAL 4096U

MCP_CAN CAN0(CAN_CS);

// ---------- I2C/TWI thanh ghi AVR ----------
// F_CPU = 16 MHz, prescaler = 1, SCL = 100 kHz
void i2c_init(void)
{
    TWSR = 0;
    TWBR = 12;
    TWCR = (1 << TWEN);
}

void i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

uint8_t i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) {}
    return TWDR;
}

uint8_t i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
    return TWDR;
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

// ---------- INA219 ----------
static void ina_write(uint8_t reg, uint16_t value)
{
    i2c_start();
    i2c_write((INA_ADDR << 1) | 0);
    i2c_write(reg);
    i2c_write((uint8_t)(value >> 8));
    i2c_write((uint8_t)value);
    i2c_stop();
}

static uint16_t ina_read(uint8_t reg)
{
    i2c_start();
    i2c_write((INA_ADDR << 1) | 0);
    i2c_write(reg);

    i2c_start();
    i2c_write((INA_ADDR << 1) | 1);

    uint8_t hi = i2c_read_ack();
    uint8_t lo = i2c_read_nack();
    i2c_stop();

    return ((uint16_t)hi << 8) | lo;
}

static void ina_init(void)
{
    // 32 V range, PGA /8, 12-bit ADC, continuous shunt + bus
    ina_write(REG_CONFIG, 0x399F);
    ina_write(REG_CAL, INA_CAL);
    delay(10);
}

static uint16_t read_voltage_mV(void)
{
    uint16_t raw = ina_read(REG_BUS);
    return (uint16_t)((raw >> 3) * 4U);
}

static int16_t read_current_mA(void)
{
    // Với calibration 4096 và shunt 0.1 ohm:
    // 1 count = 0.1 mA
    int16_t raw = (int16_t)ina_read(REG_CURRENT);
    return raw / 10;
}

static uint16_t read_power_mW(void)
{
    // Power LSB = 2 mW/count theo cấu hình này
    return (uint16_t)(ina_read(REG_POWER) * 2U);
}

static void send_power(uint16_t v, int16_t i, uint16_t p)
{
    byte d[6];

    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)v;
    d[2] = (uint8_t)((uint16_t)i >> 8);
    d[3] = (uint8_t)i;
    d[4] = (uint8_t)(p >> 8);
    d[5] = (uint8_t)p;

    CAN0.sendMsgBuf(0x200, 0, 6, d);
}

void setup()
{
    Serial.begin(9600);
    i2c_init();
    ina_init();

    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.println(F("MCP2515 init error"));
        delay(500);
    }

    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("Power node ready"));
}

void loop()
{
    uint16_t v = read_voltage_mV();
    int16_t i = read_current_mA();
    uint16_t p = read_power_mW();

    send_power(v, i, p);

    Serial.print(F("V="));
    Serial.print(v);
    Serial.print(F(" mV, I="));
    Serial.print(i);
    Serial.print(F(" mA, P="));
    Serial.print(p);
    Serial.println(F(" mW"));

    delay(500);
}
