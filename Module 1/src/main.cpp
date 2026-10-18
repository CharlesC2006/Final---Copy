
#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/io.h>
#include <math.h>

#define CAN_CS 10
#define NTC1_CH 0
#define NTC2_CH 1

MCP_CAN CAN0(CAN_CS);

#define R_FIXED 10000.0
#define R25     10000.0
#define BETA    3950.0
#define T25     298.15

// ADC thanh ghi AVR, Vref = AVcc
static void adc_init_avr(void)
{
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D);
}

static uint16_t adc_read_avr(uint8_t ch)
{
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {}
    return ADC;
}

// Wiring: 5V -> NTC -> ADC -> 10k -> GND
static float read_ntc_c(uint8_t ch)
{
    uint16_t raw = adc_read_avr(ch);

    if (raw == 0) return -100.0f;
    if (raw >= 1023) return 200.0f;

    float r_ntc = R_FIXED * raw / (1023.0f - raw);
    float invT = (1.0f / T25) +
                 (1.0f / BETA) * logf(r_ntc / R25);

    return (1.0f / invT) - 273.15f;
}

static int16_t to_tenths(float t)
{
    float x = t * 10.0f;
    return (int16_t)(x >= 0 ? x + 0.5f : x - 0.5f);
}

static void send_temperatures(int16_t t1, int16_t t2)
{
    byte d[4];

    d[0] = (uint8_t)((uint16_t)t1 >> 8);
    d[1] = (uint8_t)t1;
    d[2] = (uint8_t)((uint16_t)t2 >> 8);
    d[3] = (uint8_t)t2;

    CAN0.sendMsgBuf(0x100, 0, 4, d);
}

void setup()
{
    Serial.begin(9600);
    adc_init_avr();

    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(F("MCP2515 init error"));
        delay(500);
    }

    CAN0.setMode(MCP_NORMAL);
    Serial.println(F("Temperature node ready"));
}

void loop()
{
    float t1 = read_ntc_c(NTC1_CH);
    float t2 = read_ntc_c(NTC2_CH);

    send_temperatures(to_tenths(t1), to_tenths(t2));

    Serial.print(F("T1="));
    Serial.print(t1, 1);
    Serial.print(F(" C, T2="));
    Serial.print(t2, 1);
    Serial.println(F(" C"));

    delay(500);
}
