#include <Arduino.h>
#include <wiring_private.h>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Start debug serial
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial to be ready
    }
    
    Serial.println("SBUS Debug Monitor Starting...");
    
    // Initialize SBUS UART (100K baud, 8E2)
    Serial1.begin(100000, SERIAL_8E2);
    
    // Configure pin for SERCOM use
    pinPeripheral(PIN_SERIAL1_RX, PIO_SERCOM);
    
    Serial.println("UART Configured for 100000 baud, 8E2");
    Serial.println("Waiting for SBUS data...");
}

void loop() {
    static uint32_t last_print = 0;
    static uint32_t byte_count = 0;
    static uint8_t last_byte = 0;
    static uint8_t sbus_buffer[25];  // SBUS frame is 25 bytes
    static uint8_t buffer_index = 0;
    
    // Check for incoming bytes
    while (Serial1.available()) {
        uint8_t inByte = Serial1.read();
        byte_count++;
        
        // Real-time hex display of each byte
        if (inByte < 0x10) Serial.print("0");
        Serial.print(inByte, HEX);
        Serial.print(" ");
        
        // Look for SBUS frame start (0x0F after 0x00)
        if (last_byte == 0x00 && inByte == 0x0F) {
            Serial.println("\n--- New Frame Start ---");
            buffer_index = 0;
        }
        
        // Store byte in buffer
        if (buffer_index < 25) {
            sbus_buffer[buffer_index++] = inByte;
            
            // If we have a complete frame
            if (buffer_index == 25) {
                Serial.println("\nComplete Frame Contents:");
                // Print all bytes in hex
                for (int i = 0; i < 25; i++) {
                    if (sbus_buffer[i] < 0x10) Serial.print("0");
                    Serial.print(sbus_buffer[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();
                
                // Decode all channels
                if (sbus_buffer[0] == 0x0F) {
                    Serial.println("Decoded Channel Values:");
                    
                    // SBUS protocol packs 16 11-bit channels into 22 bytes
                    uint16_t channels[16];
                    for (int i = 0; i < 16; i++) {
                        int byte_index = 1 + (i * 11) / 8;
                        int bit_offset = (i * 11) % 8;
                        
                        // Extract 11 bits for each channel
                        channels[i] = (sbus_buffer[byte_index] >> bit_offset) |
                                    (sbus_buffer[byte_index + 1] << (8 - bit_offset));
                        
                        if (bit_offset > 5)  // Need third byte
                            channels[i] |= (sbus_buffer[byte_index + 2] << (16 - bit_offset));
                        
                        channels[i] &= 0x07FF;  // Mask to 11 bits
                        
                        Serial.print("CH");
                        Serial.print(i + 1);
                        Serial.print(": ");
                        Serial.print(channels[i]);
                        Serial.print("\t");
                        
                        if ((i + 1) % 4 == 0) Serial.println(); // New line every 4 channels
                    }
                    
                    // Decode digital channels and flags
                    bool ch17 = sbus_buffer[23] & 0x01;
                    bool ch18 = sbus_buffer[23] & 0x02;
                    bool failSafe = sbus_buffer[23] & 0x08;
                    bool frameLost = sbus_buffer[23] & 0x04;
                    
                    Serial.print("Digital CH17: "); Serial.print(ch17);
                    Serial.print(" CH18: "); Serial.println(ch18);
                    Serial.print("Failsafe: "); Serial.print(failSafe);
                    Serial.print(" Frame Lost: "); Serial.println(frameLost);
                    Serial.println("------------------------");
                }
            }
        }
        
        last_byte = inByte;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  // Toggle LED on data
    }
    
    // Status update every second if no data received
    uint32_t now = millis();
    if (now - last_print > 1000) {
        if (byte_count == 0) {
            Serial.println("No data received in last second");
        }
        byte_count = 0;
        last_print = now;
    }
}