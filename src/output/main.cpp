#include <Arduino.h>
#include <wiring_private.h>

// DAC resolution
#define DAC_RESOLUTION    12      // 12-bit DAC
#define DAC_MAX_VALUE    4095    // 2^12 - 1

// SBUS values
#define SBUS_MAX_VALUE   2047    
#define SBUS_MIN_VALUE   0
#define SBUS_MID_VALUE   1000    // Center point from controller

// Motor control settings
#define MOTOR_DEADBAND   20      // Deadband around center point
#define CONTROL_SMOOTHING 0.5    // No smoothing for direct control

// Structure to hold motor control data
struct MotorControl {
    uint16_t raw_value;      
    float filtered_value;    
    int dac_value;          
    uint8_t dac_pin;        
};

// Motor controls
MotorControl motor1 = {SBUS_MID_VALUE, SBUS_MID_VALUE, 2048, PIN_DAC0};  // Left track
MotorControl motor2 = {SBUS_MID_VALUE, SBUS_MID_VALUE, 2048, PIN_DAC1};  // Right track

void setupDAC() {
    analogWriteResolution(DAC_RESOLUTION);
    
    // Initialize DACs to middle position (half of max DAC value)
    analogWrite(motor1.dac_pin, DAC_MAX_VALUE / 2);
    analogWrite(motor2.dac_pin, DAC_MAX_VALUE / 2);
}

void updateMotor(MotorControl &motor, uint16_t new_value) {
    // Apply smoothing (currently disabled)
    motor.filtered_value = (motor.filtered_value * CONTROL_SMOOTHING) + 
                          (new_value * (1.0 - CONTROL_SMOOTHING));
    
    // Apply deadband around center
    float offset = motor.filtered_value - SBUS_MID_VALUE;
    if (abs(offset) < MOTOR_DEADBAND) {
        motor.dac_value = DAC_MAX_VALUE / 2;  // Center position
        return;
    }
    
    // Map SBUS range to DAC range with INVERTED output
    // Higher SBUS value = Lower voltage (Lower DAC value)
    if (motor.filtered_value > SBUS_MID_VALUE) {
        // Upper half of range (SBUS > mid) = Lower half of voltage
        motor.dac_value = map(motor.filtered_value,
                            SBUS_MID_VALUE, SBUS_MAX_VALUE,
                            DAC_MAX_VALUE/2, 0);  // Map to lower voltages
    } else {
        // Lower half of range (SBUS < mid) = Upper half of voltage
        motor.dac_value = map(motor.filtered_value,
                            SBUS_MIN_VALUE, SBUS_MID_VALUE,
                            DAC_MAX_VALUE, DAC_MAX_VALUE/2);  // Map to higher voltages
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
                    // Get channel 1 & 3 values for tank controls
                    uint16_t ch1_value = processSBUSFrame(sbus_buffer, 0);  // Left track
                    uint16_t ch3_value = processSBUSFrame(sbus_buffer, 2);  // Right track (CH3)
                    
                    updateMotor(motor1, ch1_value);
                    updateMotor(motor2, ch3_value);
                    
                    // Debug output
                    Serial.print("Left(CH1): "); Serial.print(ch1_value);
                    Serial.print(" DAC_L: "); Serial.print(motor1.dac_value);
                    Serial.print(" Right(CH3): "); Serial.print(ch3_value);
                    Serial.print(" DAC_R: "); Serial.println(motor2.dac_value);
                }
            }
        }
        
        last_byte = inByte;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}