
#include <Arduino.h>
#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
#include <SPI.h>
#include <mcp_can.h>

// ================= PIN CONFIG =================
#define CAN_CS_PIN 10

// ================= CAN CONFIG =================
MCP_CAN CAN(CAN_CS_PIN);
const uint32_t CAN_ID_POWER = 0x200;

// ================= INA219 CONFIG =================
uint8_t INA219_ADDR = 0x40;

#define INA_REG_CONFIG  0x00
#define INA_REG_SHUNT   0x01
#define INA_REG_BUS     0x02
#define INA_REG_POWER   0x03
#define INA_REG_CURRENT 0x04
#define INA_REG_CAL     0x05

#define INA_CAL_VALUE   4096

// =================================================
// AVR TWI: 100 kHz at 16 MHz
// =================================================
void twi_init() {
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

void twi_start() {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {}

    uint8_t status = TWSR & 0xF8;

    if (status != 0x08 && status != 0x10) {
        Serial.print(F("TWI START error: 0x"));
        Serial.println(status, HEX);
    }
}

void twi_stop() {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    delayMicroseconds(10);
}

uint8_t twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {}

    return TWSR & 0xF8;
}

uint8_t twi_read_ack() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}

uint8_t twi_read_nack() {
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {}

    return TWDR;
}

// =================================================
// Scan INA219 address range 0x40-0x4F
// =================================================
bool ina_probe(uint8_t addr) {
    twi_start();

    uint8_t status = twi_write((addr << 1) | 0);

    twi_stop();

    // 0x18 = SLA+W transmitted, ACK received
    return status == 0x18;
}

bool ina_find_address() {
    Serial.println(F("Scanning I2C addresses 0x40-0x4F..."));

    for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
        if (ina_probe(addr)) {
            INA219_ADDR = addr;

            Serial.print(F("I2C device ACK at 0x"));
            Serial.println(addr, HEX);

            return true;
        }
    }

    Serial.println(F("No I2C device found in 0x40-0x4F"));
    return false;
}

// =================================================
// Read one INA219 register
// =================================================
bool ina_read_reg(uint8_t reg, uint16_t &value) {
    twi_start();

    uint8_t status = twi_write((INA219_ADDR << 1) | 0);

    if (status != 0x18) {
        twi_stop();
        return false;
    }

    status = twi_write(reg);

    if (status != 0x28) {
        twi_stop();
        return false;
    }

    twi_start();

    status = twi_write((INA219_ADDR << 1) | 1);

    if (status != 0x40) {
        twi_stop();
        return false;
    }

    uint8_t hi = twi_read_ack();
    uint8_t lo = twi_read_nack();

    twi_stop();

    value = ((uint16_t)hi << 8) | lo;

    return true;
}

// =================================================
// Write one INA219 register
// =================================================
bool ina_write_reg(uint8_t reg, uint16_t value) {
    twi_start();

    uint8_t status = twi_write((INA219_ADDR << 1) | 0);

    if (status != 0x18) {
        twi_stop();
        return false;
    }

    status = twi_write(reg);

    if (status != 0x28) {
        twi_stop();
        return false;
    }

    status = twi_write((uint8_t)(value >> 8));

    if (status != 0x28) {
        twi_stop();
        return false;
    }

    status = twi_write((uint8_t)value);

    if (status != 0x28) {
        twi_stop();
        return false;
    }

    twi_stop();
    return true;
}

// =================================================
// INA219 initialization
// =================================================
bool ina_init() {
    uint16_t config = 0;

    if (!ina_read_reg(INA_REG_CONFIG, config)) {
        Serial.println(F("INA219 CONFIG read failed"));
        return false;
    }

    Serial.print(F("Initial CONFIG = 0x"));
    Serial.println(config, HEX);

    // Configuration: 32V range, gain /8, 12-bit ADC,
    // continuous shunt + bus voltage conversion.
    if (!ina_write_reg(INA_REG_CONFIG, 0x399F)) {
        Serial.println(F("INA219 CONFIG write failed"));
        return false;
    }

    if (!ina_write_reg(INA_REG_CAL, INA_CAL_VALUE)) {
        Serial.println(F("INA219 CAL write failed"));
        return false;
    }

    delay(10);

    uint16_t check_config = 0;
    uint16_t check_cal = 0;

    if (!ina_read_reg(INA_REG_CONFIG, check_config) ||
        !ina_read_reg(INA_REG_CAL, check_cal)) {
        Serial.println(F("INA219 verification read failed"));
        return false;
    }

    Serial.print(F("CONFIG = 0x"));
    Serial.println(check_config, HEX);

    Serial.print(F("CAL = 0x"));
    Serial.println(check_cal, HEX);

    return check_config == 0x399F &&
           check_cal == INA_CAL_VALUE;
}

// =================================================
// Read and display INA219 measurements
// =================================================
bool read_power(uint16_t &voltage_mV,
                int16_t &current_mA,
                uint16_t &power_mW) {
    uint16_t config, shunt_raw, bus_raw;
    uint16_t power_raw, current_raw, cal_raw;

    if (!ina_read_reg(INA_REG_CONFIG, config) ||
        !ina_read_reg(INA_REG_SHUNT, shunt_raw) ||
        !ina_read_reg(INA_REG_BUS, bus_raw) ||
        !ina_read_reg(INA_REG_POWER, power_raw) ||
        !ina_read_reg(INA_REG_CURRENT, current_raw) ||
        !ina_read_reg(INA_REG_CAL, cal_raw)) {
        Serial.println(F("ERROR: INA219 register read failed"));
        return false;
    }

    // Bus register bits [15:3] are voltage code.
    uint16_t bus_code = (bus_raw >> 3) & 0x1FFF;
    voltage_mV = bus_code * 4U;

    // Shunt register is signed, 10 uV per bit at PGA /8.
    int16_t shunt_signed = (int16_t)shunt_raw;
    int32_t shunt_uV = (int32_t)shunt_signed * 10L;

    // CAL=4096 and current LSB=0.1 mA.
    int16_t current_signed = (int16_t)current_raw;
    current_mA = current_signed / 10;

    // Power LSB = 2 mW.
    power_mW = power_raw * 2U;

    Serial.println();
    Serial.println(F("========== INA219 RAW REGISTERS =========="));

    Serial.print(F("I2C ADDRESS = 0x"));
    Serial.println(INA219_ADDR, HEX);

    Serial.print(F("CONFIG_RAW  = 0x"));
    Serial.println(config, HEX);

    Serial.print(F("SHUNT_RAW   = 0x"));
    Serial.println(shunt_raw, HEX);

    Serial.print(F("BUS_RAW     = 0x"));
    Serial.print(bus_raw, HEX);
    Serial.print(F(" (decimal "));
    Serial.print(bus_raw);
    Serial.println(F(")"));

    Serial.print(F("POWER_RAW   = 0x"));
    Serial.println(power_raw, HEX);

    Serial.print(F("CURRENT_RAW = 0x"));
    Serial.println(current_raw, HEX);

    Serial.print(F("CAL_RAW     = 0x"));
    Serial.println(cal_raw, HEX);

    Serial.println(F("------------- DECODED VALUES -------------"));

    Serial.print(F("Bus voltage   : "));
    Serial.print(voltage_mV);
    Serial.println(F(" mV"));

    Serial.print(F("Shunt voltage : "));
    Serial.print(shunt_uV);
    Serial.println(F(" uV"));

    Serial.print(F("Current       : "));
    Serial.print(current_mA);
    Serial.println(F(" mA"));

    Serial.print(F("Power         : "));
    Serial.print(power_mW);
    Serial.println(F(" mW"));

    Serial.print(F("CNVR           : "));
    Serial.println((bus_raw & 0x0002) ? F("READY") : F("NOT READY"));

    Serial.print(F("Math overflow  : "));
    Serial.println((bus_raw & 0x0001) ? F("YES") : F("NO"));

    Serial.print(F("Calibration    : "));
    Serial.println(cal_raw == INA_CAL_VALUE ? F("OK") : F("MISMATCH"));

    Serial.println(F("=========================================="));

    return true;
}

// =================================================
// Setup
// =================================================
void setup() {
    Serial.begin(9600);
    delay(500);

    Serial.println();
    Serial.println(F("Initializing AVR TWI..."));
    twi_init();

    Serial.println(F("Searching for INA219..."));

    if (!ina_find_address()) {
        Serial.println(F("INA219 NOT FOUND. Check I2C."));
        while (1) {
            delay(1000);
        }
    }

    Serial.println(F("Initializing INA219..."));

    if (!ina_init()) {
        Serial.println(F("INA219 initialization failed"));
        while (1) {
            delay(1000);
        }
    }

    Serial.println(F("INA219 INIT OK"));

    Serial.println(F("Initializing MCP2515..."));

    // This project uses an 8 MHz MCP2515 oscillator.
    while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(F("MCP2515 init failed; retrying..."));
        delay(1000);
    }

    CAN.setMode(MCP_NORMAL);

    Serial.println(F("MCP2515 INIT OK"));
}

// =================================================
// Main loop
// =================================================

void loop() {
    uint16_t voltage_mV = 0;
    int16_t current_mA = 0;
    uint16_t power_mW = 0;

    // Read normal measurements
    read_power(voltage_mV, current_mA, power_mW);

    // Test BUS register independently, 5 readings
    Serial.println(F("\n--- BUS REGISTER TEST ---"));

    for (uint8_t i = 0; i < 5; i++) {
        uint16_t bus = 0;

        if (ina_read_reg(INA_REG_BUS, bus)) {
            uint16_t code = (bus >> 3) & 0x1FFF;

            Serial.print(F("BUS_RAW = 0x"));
            Serial.print(bus, HEX);

            Serial.print(F(" | Voltage = "));
            Serial.print(code * 4U);
            Serial.println(F(" mV"));
        } else {
            Serial.println(F("BUS register read FAILED"));
        }

        delay(200);
    }

    // Send CAN data
    uint8_t data[6];

    data[0] = (uint8_t)(voltage_mV >> 8);
    data[1] = (uint8_t)voltage_mV;

    data[2] = (uint8_t)((uint16_t)current_mA >> 8);
    data[3] = (uint8_t)current_mA;

    data[4] = (uint8_t)(power_mW >> 8);
    data[5] = (uint8_t)power_mW;

    byte result = CAN.sendMsgBuf(0x200, 0, 6, data);

    if (result == CAN_OK) {
        Serial.println(F("CAN TX accepted: ID=0x200"));
    } else {
        Serial.println(F("CAN TX FAILED"));
    }

    delay(1000);
}
