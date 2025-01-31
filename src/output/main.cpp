#include <Arduino.h>
#include <wiring_private.h>

// DAC resolution and values
#define DAC_RESOLUTION    12      // 12-bit DAC
#define DAC_MAX_VALUE    4095    // Full 12-bit range
#define DAC_CENTER_VALUE 1817    // Value for 1.25V output (2.5V after gain)

// SBUS values
#define SBUS_MAX_VALUE   2047    
#define SBUS_MIN_VALUE   0
#define SBUS_MID_VALUE   1000    // Center point from controller

// Motor control settings
#define MOTOR_DEADBAND   20      // Deadband around center point
#define CONTROL_SMOOTHING 0.5    // Smoothing factor for control

// Structure to hold motor control data
struct MotorControl {
    uint16_t raw_value;      
    float filtered_value;    
    int dac_value;          
    uint8_t dac_pin;        
};

// Motor controls
MotorControl motor1 = {SBUS_MID_VALUE, SBUS_MID_VALUE, DAC_CENTER_VALUE, PIN_DAC0};  // Left track
MotorControl motor2 = {SBUS_MID_VALUE, SBUS_MID_VALUE, DAC_CENTER_VALUE, PIN_DAC1};  // Right track

void setupDAC() {
    analogWriteResolution(DAC_RESOLUTION);
    analogWrite(motor1.dac_pin, DAC_CENTER_VALUE);
    analogWrite(motor2.dac_pin, DAC_CENTER_VALUE);
}

void updateMotor(MotorControl &motor, uint16_t new_value, bool is_ch3) {
    // Apply smoothing
    motor.filtered_value = (motor.filtered_value * CONTROL_SMOOTHING) + 
                          (new_value * (1.0 - CONTROL_SMOOTHING));
    
    // Apply deadband around center
    float offset = motor.filtered_value - SBUS_MID_VALUE;
    if (abs(offset) < MOTOR_DEADBAND) {
        motor.dac_value = DAC_CENTER_VALUE;  // Center position (2.5V after gain)
        return;
    }
    
    // Map SBUS range to DAC range with INVERTED output
    if (motor.filtered_value > SBUS_MID_VALUE) {
        // For CH3, swap the range to fix direction
        if (is_ch3) {
            motor.dac_value = map(motor.filtered_value,
                                SBUS_MID_VALUE, SBUS_MAX_VALUE,
                                DAC_CENTER_VALUE, DAC_MAX_VALUE);  // Map to higher voltages
        } else {
            motor.dac_value = map(motor.filtered_value,
                                SBUS_MID_VALUE, SBUS_MAX_VALUE,
                                DAC_CENTER_VALUE, 0);  // Map to lower voltages
        }
    } else {
        // For CH3, swap the range to fix direction
        if (is_ch3) {
            motor.dac_value = map(motor.filtered_value,
                                SBUS_MIN_VALUE, SBUS_MID_VALUE,
                                0, DAC_CENTER_VALUE);  // Map to lower voltages
        } else {
            motor.dac_value = map(motor.filtered_value,
                                SBUS_MIN_VALUE, SBUS_MID_VALUE,
                                DAC_MAX_VALUE, DAC_CENTER_VALUE);  // Map to higher voltages
        }
    }
    
    // Write to DAC
    analogWrite(motor.dac_pin, motor.dac_value);
}

uint16_t processSBUSFrame(uint8_t *buffer, uint8_t channel) {
    int byte_index = 1 + (channel * 11) / 8;
    int bit_offset = (channel * 11) % 8;
    
    uint16_t value = (buffer[byte_index] >> bit_offset) |
                     (buffer[byte_index + 1] << (8 - bit_offset));
    
    if (bit_offset > 5)
        value |= (buffer[byte_index + 2] << (16 - bit_offset));
    
    return value & 0x07FF;
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    
    Serial.println("SBUS Tank Control Starting...");
    Serial1.begin(100000, SERIAL_8E2);
    pinPeripheral(PIN_SERIAL1_RX, PIO_SERCOM);
    
    setupDAC();
    Serial.println("System Ready!");
}

void loop() {
    static uint8_t sbus_buffer[25];
    static uint8_t buffer_index = 0;
    static uint8_t last_byte = 0;
    
    while (Serial1.available()) {
        uint8_t inByte = Serial1.read();
        
        if (last_byte == 0x00 && inByte == 0x0F) {
            buffer_index = 0;
        }
        
        if (buffer_index < 25) {
            sbus_buffer[buffer_index++] = inByte;
            
            if (buffer_index == 25) {
                if (sbus_buffer[0] == 0x0F && !(sbus_buffer[23] & 0x08)) {
                    uint16_t ch1_value = processSBUSFrame(sbus_buffer, 0);  // Left track
                    uint16_t ch3_value = processSBUSFrame(sbus_buffer, 2);  // Right track (CH3)
                    
                    updateMotor(motor1, ch1_value, false);  // CH1
                    updateMotor(motor2, ch3_value, true);   // CH3
                    
                    // Debug output
                    float dac_v1 = (motor1.dac_value * 2.82) / 4095.0;  // DAC voltage
                    float dac_v2 = (motor2.dac_value * 2.82) / 4095.0;
                    float out_v1 = dac_v1 * 2.0;  // After gain
                    float out_v2 = dac_v2 * 2.0;
                    
                    Serial.print("Left(CH1): "); Serial.print(ch1_value);
                    Serial.print(" DAC_L: "); Serial.print(motor1.dac_value);
                    Serial.print(" V_L: "); Serial.print(out_v1);
                    Serial.print(" Right(CH3): "); Serial.print(ch3_value);
                    Serial.print(" DAC_R: "); Serial.print(motor2.dac_value);
                    Serial.print(" V_R: "); Serial.println(out_v2);
                }
            }
        }
        
        last_byte = inByte;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}