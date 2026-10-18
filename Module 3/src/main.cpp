#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/io.h>
#include <stdint.h>

// ==================== CAN ====================
#define CAN_CS 10
MCP_CAN CAN0(CAN_CS);

#define CAN_ID_TEMP   0x100
#define CAN_ID_POWER  0x200

#define CAN_BITRATE   CAN_500KBPS
#define CAN_CLOCK     MCP_8MHZ

// ==================== GPIO: ATmega328P/PB ====================
// D3 = PD3: Buzzer
// D4 = PD4: RGB Red
// D5 = PD5: RGB Green
// D6 = PD6: RGB Blue
// D7 = PD7: Relay IN (assumed active-low)
// D8 = PB0: Mute button, active-low
// D9 = PB1: Reset button, active-low

#define BUZZER_BIT  PD3
#define RED_BIT     PD4
#define GREEN_BIT   PD5
#define BLUE_BIT    PD6
#define RELAY_BIT   PD7

#define MUTE_BIT    PB0
#define RESET_BIT   PB1

// ==================== THRESHOLDS ====================
const int16_t TEMP_WARNING_X10 = 500;  // 50.0 C
const int16_t TEMP_CRITICAL_X10 = 600; // 60.0 C

const int16_t CURRENT_LIMIT_MA = 800;
const uint16_t VOLTAGE_MIN_MV = 7000;
const uint16_t VOLTAGE_MAX_MV = 15000;

const unsigned long DATA_TIMEOUT_MS = 2000;
const unsigned long PRINT_INTERVAL_MS = 1000;

// ==================== RECEIVED DATA ====================
int16_t temp1_x10 = 0;
int16_t temp2_x10 = 0;

uint16_t voltage_mV = 0;
int16_t current_mA = 0;
uint16_t power_mW = 0;

unsigned long lastTempRx = 0;
unsigned long lastPowerRx = 0;
unsigned long lastPrint = 0;

bool tempDataReceived = false;
bool powerDataReceived = false;

bool faultLatched = true;
bool buzzerMuted = false;

bool previousMutePressed = false;
bool previousResetPressed = false;

// ==================== GPIO HELPERS ====================
void gpio_init() {
    // D3-D7 outputs
    DDRD |= (1 << BUZZER_BIT) |
            (1 << RED_BIT) |
            (1 << GREEN_BIT) |
            (1 << BLUE_BIT) |
            (1 << RELAY_BIT);

    // D8, D9 inputs with internal pull-ups
    DDRB &= ~((1 << MUTE_BIT) | (1 << RESET_BIT));
    PORTB |= (1 << MUTE_BIT) | (1 << RESET_BIT);

    // Safe initial output state
    PORTD &= ~((1 << BUZZER_BIT) |
               (1 << RED_BIT) |
               (1 << GREEN_BIT) |
               (1 << BLUE_BIT));

    // Active-low relay: HIGH releases the relay coil
    PORTD |= (1 << RELAY_BIT);
}

void set_rgb(bool red, bool green, bool blue) {
    if (red)   PORTD |=  (1 << RED_BIT);
    else       PORTD &= ~(1 << RED_BIT);

    if (green) PORTD |=  (1 << GREEN_BIT);
    else       PORTD &= ~(1 << GREEN_BIT);

    if (blue)  PORTD |=  (1 << BLUE_BIT);
    else       PORTD &= ~(1 << BLUE_BIT);
}

void set_buzzer(bool enabled) {
    if (enabled) PORTD |=  (1 << BUZZER_BIT);
    else         PORTD &= ~(1 << BUZZER_BIT);
}

void set_relay(bool connected) {
    // Assumes an active-low relay module:
    // LOW = relay energized; HIGH = relay released.
    if (connected) PORTD &= ~(1 << RELAY_BIT);
    else           PORTD |=  (1 << RELAY_BIT);
}

// ==================== CAN DATA HELPERS ====================
int16_t read_i16_be(const byte *data) {
    uint16_t value = ((uint16_t)data[0] << 8) | data[1];
    return (int16_t)value;
}

uint16_t read_u16_be(const byte *data) {
    return ((uint16_t)data[0] << 8) | data[1];
}

void receive_can_data() {
    if (CAN0.checkReceive() != CAN_MSGAVAIL) {
        return;
    }

    unsigned long rxId = 0;
    byte len = 0;
    byte data[8] = {0};

    if (CAN0.readMsgBuf(&rxId, &len, data) != CAN_OK) {
        return;
    }

    if (rxId == CAN_ID_TEMP && len >= 4) {
        temp1_x10 = read_i16_be(&data[0]);
        temp2_x10 = read_i16_be(&data[2]);

        lastTempRx = millis();
        tempDataReceived = true;
    }
    else if (rxId == CAN_ID_POWER && len >= 6) {
        voltage_mV = read_u16_be(&data[0]);
        current_mA = read_i16_be(&data[2]);
        power_mW = read_u16_be(&data[4]);

        lastPowerRx = millis();
        powerDataReceived = true;
    }
}

// ==================== SAFETY LOGIC ====================
bool data_is_fresh() {
    unsigned long now = millis();

    return tempDataReceived &&
           powerDataReceived &&
           (now - lastTempRx <= DATA_TIMEOUT_MS) &&
           (now - lastPowerRx <= DATA_TIMEOUT_MS);
}

bool warning_condition() {
    return tempDataReceived &&
           (temp1_x10 >= TEMP_WARNING_X10 ||
            temp2_x10 >= TEMP_WARNING_X10);
}

bool critical_condition() {
    if (!data_is_fresh()) {
        return true;
    }

    if (temp1_x10 >= TEMP_CRITICAL_X10 ||
        temp2_x10 >= TEMP_CRITICAL_X10) {
        return true;
    }

    if (current_mA > CURRENT_LIMIT_MA ||
        current_mA < -CURRENT_LIMIT_MA) {
        return true;
    }

    if (voltage_mV < VOLTAGE_MIN_MV ||
        voltage_mV > VOLTAGE_MAX_MV) {
        return true;
    }

    return false;
}

bool all_conditions_safe() {
    if (!data_is_fresh()) {
        return false;
    }

    if (temp1_x10 >= TEMP_WARNING_X10 ||
        temp2_x10 >= TEMP_WARNING_X10) {
        return false;
    }

    if (current_mA > CURRENT_LIMIT_MA ||
        current_mA < -CURRENT_LIMIT_MA) {
        return false;
    }

    if (voltage_mV < VOLTAGE_MIN_MV ||
        voltage_mV > VOLTAGE_MAX_MV) {
        return false;
    }

    return true;
}

void handle_buttons() {
    bool mutePressed = !(PINB & (1 << MUTE_BIT));
    bool resetPressed = !(PINB & (1 << RESET_BIT));

    // Mute buzzer on a new button press
    if (mutePressed && !previousMutePressed) {
        buzzerMuted = true;
        Serial.println(F("[BUTTON] Buzzer muted"));
    }

    // Reset only if every monitored condition is safe
    if (resetPressed && !previousResetPressed) {
        if (all_conditions_safe()) {
            faultLatched = false;
            buzzerMuted = false;
            Serial.println(F("[BUTTON] Fault reset: conditions safe"));
        } else {
            Serial.println(F("[BUTTON] Reset rejected: conditions unsafe"));
        }
    }

    previousMutePressed = mutePressed;
    previousResetPressed = resetPressed;
}

void update_safety_state() {
    if (critical_condition()) {
        if (!faultLatched) {
            Serial.println(F("[FAULT] Critical condition detected"));
        }

        faultLatched = true;
    }
}

void update_outputs() {
    bool warning = warning_condition();
    bool critical = faultLatched;

    // Red = latched fault; yellow = warning; green = normal
    if (critical) {
        set_rgb(true, false, false);
    }
    else if (warning) {
        set_rgb(true, true, false);
    }
    else {
        set_rgb(false, true, false);
    }

    set_buzzer((critical || warning) && !buzzerMuted);

    // Disconnect the load whenever a fault is latched
    set_relay(!faultLatched);
}

// ==================== SERIAL MONITOR ====================
void print_temperature(const __FlashStringHelper *label,
                       int16_t value_x10) {
    Serial.print(label);
    Serial.print(value_x10 / 10);
    Serial.print('.');
    int16_t fraction = value_x10 % 10;
    if (fraction < 0) fraction = -fraction;
    Serial.print(fraction);
    Serial.println(F(" C"));
}

void print_status() {
    Serial.println();
    Serial.println(F("========== BATTERY ECU =========="));

    Serial.print(F("CAN temperature node: "));
    Serial.println(tempDataReceived ? F("DATA RECEIVED") : F("NO DATA"));

    if (tempDataReceived) {
        print_temperature(F("Temperature 1: "), temp1_x10);
        print_temperature(F("Temperature 2: "), temp2_x10);
    }

    Serial.print(F("CAN power node: "));
    Serial.println(powerDataReceived ? F("DATA RECEIVED") : F("NO DATA"));

    if (powerDataReceived) {
        Serial.print(F("Voltage: "));
        Serial.print(voltage_mV / 1000.0, 2);
        Serial.println(F(" V"));

        Serial.print(F("Current: "));
        Serial.print(current_mA / 1000.0, 3);
        Serial.println(F(" A"));

        Serial.print(F("Power: "));
        Serial.print(power_mW / 1000.0, 2);
        Serial.println(F(" W"));
    }

    Serial.print(F("Warning: "));
    Serial.println(warning_condition() ? F("YES") : F("NO"));

    Serial.print(F("Fault latched: "));
    Serial.println(faultLatched ? F("YES") : F("NO"));

    Serial.print(F("Buzzer: "));
    Serial.println(((faultLatched || warning_condition()) &&
                    !buzzerMuted) ? F("ON") : F("OFF"));

    Serial.print(F("Relay command: "));
    Serial.println(faultLatched ? F("DISCONNECT") : F("CONNECT"));

    if (!data_is_fresh()) {
        Serial.println(F("[WARNING] Missing or stale CAN data"));
    }

    Serial.println(F("================================="));
}

// ==================== SETUP ====================
void setup() {
    gpio_init();

    Serial.begin(9600);
    SPI.begin();

    // Keep SPI master mode enabled and CS inactive before init
    pinMode(CAN_CS, OUTPUT);
    digitalWrite(CAN_CS, HIGH);

    Serial.println();
    Serial.println(F("Battery Safety ECU starting..."));
    Serial.println(F("MCP2515 clock: 8 MHz"));
    Serial.println(F("CAN bitrate: 500 kbps"));

    while (CAN0.begin(MCP_ANY, CAN_BITRATE, CAN_CLOCK) != CAN_OK) {
        Serial.println(F("[ERROR] MCP2515 init failed. Check SPI, power and clock."));
        delay(1000);
    }

    CAN0.setMode(MCP_NORMAL);

    Serial.println(F("[OK] MCP2515 initialized"));
    Serial.println(F("[OK] CAN normal mode"));
    Serial.println(F("Waiting for CAN IDs 0x100 and 0x200..."));
    Serial.println(F("Serial Monitor: 9600 baud"));
}

// ==================== MAIN LOOP ====================
void loop() {
    receive_can_data();
    handle_buttons();
    update_safety_state();
    update_outputs();

    unsigned long now = millis();

    if (now - lastPrint >= PRINT_INTERVAL_MS) {
        lastPrint = now;
        print_status();
    }
}