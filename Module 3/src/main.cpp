
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>

// =====================================================
// MODULE 3: CENTRAL ECU - ATmega328PB, 16 MHz
// CAN IDs:
//   0x100: T1, T2; signed int16, 0.1 degree C, big-endian
//   0x200: voltage mV, current mA, power mW, big-endian
// =====================================================

// MCP2515
#define CAN_CS 10
MCP_CAN CAN0(CAN_CS);

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- GPIO mapping ----------------
// Arduino D3 = PD3: buzzer
// Arduino D4 = PD4: RGB red
// Arduino D5 = PD5: RGB green
// Arduino D6 = PD6: RGB blue
// Arduino D7 = PD7: relay IN
// Arduino D8 = PB0: mute button, active LOW
// Arduino D9 = PB1: reset button, active LOW
//
// Assumptions:
// - RGB LED is common cathode.
// - Relay module is active LOW.
// - Buttons connect to GND when pressed.

#define BUZZER_BIT PD3
#define RGB_R_BIT  PD4
#define RGB_G_BIT  PD5
#define RGB_B_BIT  PD6
#define RELAY_BIT  PD7

#define MUTE_BIT   PB0
#define RESET_BIT  PB1

#define OUTPUT_MASK ((1 << BUZZER_BIT) | \
                     (1 << RGB_R_BIT)  | \
                     (1 << RGB_G_BIT)  | \
                     (1 << RGB_B_BIT)  | \
                     (1 << RELAY_BIT))

// ---------------- Thresholds ----------------
const int16_t TEMP_WARN_X10 = 500;   // 50.0 C
const int16_t TEMP_CRIT_X10 = 600;   // 60.0 C

const int32_t CURRENT_LIMIT_MA = 800;  // 0.8 A
const uint16_t VOLTAGE_MIN_MV = 7000;
const uint16_t VOLTAGE_MAX_MV = 15000;

const unsigned long CAN_TIMEOUT_MS = 2000;
const unsigned long LCD_UPDATE_MS = 300;
const unsigned long BLINK_INTERVAL_MS = 250;

// ---------------- Received data ----------------
int16_t temp1_x10 = 0;
int16_t temp2_x10 = 0;

uint16_t voltage_mV = 0;
int16_t current_mA = 0;
uint16_t power_mW = 0;

bool haveTempData = false;
bool havePowerData = false;

unsigned long lastTempMs = 0;
unsigned long lastPowerMs = 0;

// ---------------- System state ----------------
bool faultLatched = true;  // Start safe: relay open until data is valid
bool muted = false;
bool lastMuteButton = false;
bool lastResetButton = false;

unsigned long lastLcdMs = 0;
unsigned long lastBlinkMs = 0;
bool blinkState = false;

// =====================================================
// GPIO helpers - AVR registers
// =====================================================

void gpio_init()
{
    // D3-D7 as outputs
    DDRD |= OUTPUT_MASK;

    // Initial state:
    // buzzer OFF, RGB OFF, relay OFF/open (active LOW)
    PORTD &= ~((1 << BUZZER_BIT) |
               (1 << RGB_R_BIT) |
               (1 << RGB_G_BIT) |
               (1 << RGB_B_BIT));

    PORTD |= (1 << RELAY_BIT);

    // D8/D9 as inputs with internal pull-ups
    DDRB &= ~((1 << MUTE_BIT) | (1 << RESET_BIT));
    PORTB |= (1 << MUTE_BIT) | (1 << RESET_BIT);
}

bool mute_pressed()
{
    return !(PINB & (1 << MUTE_BIT));
}

bool reset_pressed()
{
    return !(PINB & (1 << RESET_BIT));
}

void buzzer_set(bool on)
{
    if (on && !muted)
        PORTD |= (1 << BUZZER_BIT);
    else
        PORTD &= ~(1 << BUZZER_BIT);
}

void rgb_set(bool red, bool green, bool blue)
{
    uint8_t mask = (1 << RGB_R_BIT) |
                   (1 << RGB_G_BIT) |
                   (1 << RGB_B_BIT);

    uint8_t value = 0;

    if (red)   value |= (1 << RGB_R_BIT);
    if (green) value |= (1 << RGB_G_BIT);
    if (blue)  value |= (1 << RGB_B_BIT);

    PORTD = (PORTD & ~mask) | value;
}

void relay_set(bool closed)
{
    // Active-LOW relay:
    // LOW  = relay energized, contact COM-NO closed
    // HIGH = relay de-energized, contact COM-NO open
    if (closed)
        PORTD &= ~(1 << RELAY_BIT);
    else
        PORTD |= (1 << RELAY_BIT);
}

// =====================================================
// CAN decoding helpers
// =====================================================

uint16_t read_u16_be(const byte *d, uint8_t i)
{
    return ((uint16_t)d[i] << 8) | d[i + 1];
}

int16_t read_i16_be(const byte *d, uint8_t i)
{
    uint16_t raw = read_u16_be(d, i);
    return (int16_t)raw;
}

void receive_can()
{
    if (CAN0.checkReceive() != CAN_MSGAVAIL)
        return;

    unsigned long id = 0;
    byte len = 0;
    byte data[8] = {0};

    CAN0.readMsgBuf(&id, &len, data);

    if (id == 0x100 && len >= 4)
    {
        temp1_x10 = read_i16_be(data, 0);
        temp2_x10 = read_i16_be(data, 2);

        haveTempData = true;
        lastTempMs = millis();
    }
    else if (id == 0x200 && len >= 6)
    {
        voltage_mV = read_u16_be(data, 0);
        current_mA = read_i16_be(data, 2);
        power_mW = read_u16_be(data, 4);

        havePowerData = true;
        lastPowerMs = millis();
    }
}

// =====================================================
// Fault logic
// =====================================================

bool temp_data_fresh()
{
    return haveTempData &&
           (millis() - lastTempMs <= CAN_TIMEOUT_MS);
}

bool power_data_fresh()
{
    return havePowerData &&
           (millis() - lastPowerMs <= CAN_TIMEOUT_MS);
}

bool communication_fault()
{
    return !temp_data_fresh() || !power_data_fresh();
}

bool temperature_warning()
{
    return temp1_x10 >= TEMP_WARN_X10 ||
           temp2_x10 >= TEMP_WARN_X10;
}

bool temperature_critical()
{
    return temp1_x10 >= TEMP_CRIT_X10 ||
           temp2_x10 >= TEMP_CRIT_X10;
}

bool current_fault()
{
    int32_t i = current_mA;

    if (i < 0)
        i = -i;

    return i >= CURRENT_LIMIT_MA;
}

bool voltage_fault()
{
    return voltage_mV < VOLTAGE_MIN_MV ||
           voltage_mV > VOLTAGE_MAX_MV;
}

bool all_conditions_safe()
{
    return temp_data_fresh() &&
           power_data_fresh() &&
           !temperature_critical() &&
           !current_fault() &&
           !voltage_fault();
}

void update_buttons()
{
    bool muteNow = mute_pressed();
    bool resetNow = reset_pressed();

    // Toggle mute on button press edge
    if (muteNow && !lastMuteButton)
    {
        muted = !muted;
    }

    // Reset a latched fault only if all data is fresh
    // and all critical conditions are clear.
    if (resetNow && !lastResetButton)
    {
        if (all_conditions_safe())
        {
            faultLatched = false;
            muted = false;
        }
    }

    lastMuteButton = muteNow;
    lastResetButton = resetNow;
}

void update_fault_state()
{
    // A serious fault latches until manually reset.
    if (communication_fault() ||
        temperature_critical() ||
        current_fault() ||
        voltage_fault())
    {
        faultLatched = true;
    }

    // Warning temperature alone does not latch the relay open.
}

// =====================================================
// Outputs
// =====================================================

void update_outputs()
{
    bool commFault = communication_fault();
    bool critical = temperature_critical() ||
                    current_fault() ||
                    voltage_fault();

    bool warning = temperature_warning();

    // Keep load disconnected while a fault is latched.
    relay_set(!faultLatched);

    unsigned long now = millis();

    if (now - lastBlinkMs >= BLINK_INTERVAL_MS)
    {
        lastBlinkMs = now;
        blinkState = !blinkState;
    }

    if (commFault || critical || faultLatched)
    {
        // Red blinking for latched fault / critical fault
        rgb_set(blinkState, false, false);
        buzzer_set(true);
    }
    else if (warning)
    {
        // Yellow for temperature warning
        rgb_set(true, true, false);
        buzzer_set(false);
    }
    else
    {
        // Green means normal operation
        rgb_set(false, true, false);
        buzzer_set(false);
    }
}

// =====================================================
// LCD
// =====================================================

void print_temp_value(int16_t value)
{
    lcd.print(value / 10);
    lcd.print('.');
    int16_t decimal = abs(value % 10);
    lcd.print(decimal);
}

void update_lcd()
{
    unsigned long now = millis();

    if (now - lastLcdMs < LCD_UPDATE_MS)
        return;

    lastLcdMs = now;

    bool commFault = communication_fault();
    bool critical = temperature_critical() ||
                    current_fault() ||
                    voltage_fault();

    lcd.setCursor(0, 0);

    if (commFault)
    {
        lcd.print(F("CAN COMM ERROR  "));
    }
    else if (faultLatched || critical)
    {
        lcd.print(F("FAULT - LOAD OFF"));
    }
    else if (temperature_warning())
    {
        lcd.print(F("TEMP WARNING    "));
    }
    else
    {
        lcd.print(F("T1:"));
        print_temp_value(temp1_x10);
        lcd.print(F(" T2:"));
        print_temp_value(temp2_x10);
        lcd.print(F("C "));
    }

    lcd.setCursor(0, 1);

    if (!power_data_fresh())
    {
        lcd.print(F("Power data lost "));
    }
    else
    {
        lcd.print(F("V:"));
        lcd.print(voltage_mV / 1000.0, 1);
        lcd.print(F(" I:"));
        lcd.print(current_mA / 1000.0, 2);
        lcd.print(F("A"));

        // Overwrite any leftover characters
        lcd.print(F("  "));
    }
}

// =====================================================
// Setup and main loop
// =====================================================

void setup()
{
    gpio_init();

    SPI.begin();
    Wire.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print(F("Battery ECU"));
    lcd.setCursor(0, 1);
    lcd.print(F("Starting CAN..."));

    // Initialize CAN at 500 kbps, MCP2515 crystal 16 MHz.
    while (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK)
    {
        lcd.setCursor(0, 1);
        lcd.print(F("MCP2515 ERROR   "));
        delay(500);
    }

    CAN0.setMode(MCP_NORMAL);

    // Keep relay open until both data streams are valid.
    relay_set(false);
    buzzer_set(false);
    rgb_set(false, false, true);

    delay(500);
    lcd.clear();
}

void loop()
{
    receive_can();
    update_buttons();
    update_fault_state();
    update_outputs();
    update_lcd();
}
