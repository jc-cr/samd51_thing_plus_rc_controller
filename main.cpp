#include <Arduino.h>
#include <wiring_private.h> // Required for SERCOM configuration

// SBUS Protocol Constants
#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS_FRAME_SIZE 25
#define SBUS_NUM_CHANNELS 16

// Create a new UART instance using SERCOM2
Uart SerialSBUS(&sercom2, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);

class SBUSReceiver {
private:
    Uart* serial;
    uint8_t buffer[SBUS_FRAME_SIZE];
    uint16_t channels[SBUS_NUM_CHANNELS];
    bool failsafe = false;
    bool frameLost = false;
    int bufferIndex = 0;
    unsigned long lastFrameTime = 0;
    
public:
    SBUSReceiver(Uart* _serial) : serial(_serial) {
        for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
            channels[i] = 992;
        }
    }
    
    void begin() {
        // Configure pin for SERCOM functionality
        pinPeripheral(PIN_SERIAL1_RX, PIO_SERCOM);
        
        // Configure UART for SBUS (100K baud, 8E2)
        serial->begin(100000, SERIAL_8E2);
    }
    
    bool read() {
        while (serial->available()) {

            

            uint8_t byte = serial->read();
            
            // Check for start of new frame
            if (bufferIndex == 0 && byte != SBUS_HEADER) {
                continue;
            }
            
            buffer[bufferIndex++] = byte;
            
            // Complete frame received
            if (bufferIndex == SBUS_FRAME_SIZE) {
                bufferIndex = 0;
                unsigned long currentTime = millis();
                unsigned long frameInterval = currentTime - lastFrameTime;
                lastFrameTime = currentTime;
                
                // Validate frame
                if (buffer[0] != SBUS_HEADER || buffer[24] != SBUS_FOOTER) {
                    Serial.println("Invalid frame!");
                    return false;
                }
                
                // Decode channels
                channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
                channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
                channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
                channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
                channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
                channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
                channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
                channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
                channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
                channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
                channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
                channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
                channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
                channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
                channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
                channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
                
                // Decode flags
                failsafe = buffer[23] & (1 << 3);
                frameLost = buffer[23] & (1 << 2);
                
                // Print frame rate
                Serial.print("Frame interval: ");
                Serial.print(frameInterval);
                Serial.println("ms");
                
                return true;
            }
        }
        return false;
    }
    
    uint16_t getChannel(uint8_t channel) {
        if (channel >= SBUS_NUM_CHANNELS) return 0;
        return channels[channel];
    }
    
    bool isFailsafe() {
        return failsafe;
    }
    
    bool isFrameLost() {
        return frameLost;
    }

    void printChannels() {
        Serial.println("\n--- SBUS Channel Values ---");
        for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
            Serial.print("CH");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(channels[i]);
            if (channels[i] < 1000) Serial.print("\t");  // Align values
            Serial.print("\t");
            if ((i + 1) % 4 == 0) Serial.println();  // New line every 4 channels
        }
        Serial.print("Failsafe: ");
        Serial.print(failsafe ? "YES" : "NO");
        Serial.print("\tFrame Lost: ");
        Serial.println(frameLost ? "YES" : "NO");
        Serial.println("------------------------");
    }
};

SBUSReceiver sbus(&SerialSBUS);
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 500;  // Print every 500ms

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);  // Debug serial
    while (!Serial) {
        ; // Wait for Serial to be ready
    }
    
    Serial.println("SBUS Channel Monitor Starting...");
    sbus.begin();
}

void loop() {
    if (sbus.read()) {
        digitalWrite(LED_BUILTIN, HIGH);
        
        // Print channel values every PRINT_INTERVAL ms
        unsigned long currentTime = millis();
        if (currentTime - lastPrint >= PRINT_INTERVAL) {
            sbus.printChannels();
            lastPrint = currentTime;
        }
        
        digitalWrite(LED_BUILTIN, LOW);
    }
}