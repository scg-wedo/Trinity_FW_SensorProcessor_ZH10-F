#include <Arduino.h>
#include <Wire.h>
#include <IWatchdog.h>
#include <HardwareSerial.h>


// Pin Definitions
#define LED_PIN PA5
#define LOWPOWER_SET_PIN PA1
#define PIN_SERIAL_TX1 PB6
#define PIN_SERIAL_RX1 PB7

// Low Power Timings (milliseconds)
#define LOWPOWER_OFF_TIME (60 * 1000)
#define LOWPOWER_ON_TIME  (20 * 1000)

// Device Modes
#define MODE_ON  1
#define MODE_LOW 2

// Error Codes
#define ERROR_NONE 0x00000000lu
#define ERROR_I2C 0xCCCCCCCClu
#define ERROR_INIT 0xFFFFFFFFlu

// I2C Definitions
#define I2C_ADDRESS 10
#define I2C_TX_DATA_SIZE 27

// Sensor Model
#define SENSOR_PROCESSOR_MODEL 0x0003u

// Retries and Limits
#define MAX_INIT_RETRIES 3
#define MAX_ERROR_COUNTER 5

// Watchdog Timeout
#define WATCHDOG_TIMEOUT_MICROSECONDS 5000000

// I2C Communication
TwoWire WireSlave;
uint8_t i2c_tx_data[I2C_TX_DATA_SIZE] = {0};

// Serial Communication
HardwareSerial sensorSerial(PIN_SERIAL_RX1, PIN_SERIAL_TX1);
HardwareSerial debugSerial(PA3, PA2); // Debugging

// Buffer for sensor data
byte dataBuffer[32];
int bufferIndex = 0;

// Error Status
uint32_t device_status_error = ERROR_INIT;

// Measurement Variables
int pm1_0 = 0;
int pm2_5 = 0;
int pm10 = 0;

// Latest Measurements
float latest_pm1_0 = 0.0f;
float latest_pm2_5 = 0.0f;
float latest_pm10 = 0.0f;
uint32_t float2hex = 0;

// Timer Variables
uint32_t tick = 0;
uint32_t Lowpower_timer_tick = 0;
uint8_t mode;

void setup() {
    // Initialize Pins
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(LOWPOWER_SET_PIN, OUTPUT);
    digitalWrite(LOWPOWER_SET_PIN, HIGH);

    // Initialize I2C
    UpdateTrinityProtocol();
    WireSlave.setSCL(PA11);
    WireSlave.setSDA(PA12);
    WireSlave.begin(I2C_ADDRESS);
    WireSlave.onRequest(requestEvent);

    // Initialize Serial Communication
    debugSerial.begin(115200);
    sensorSerial.begin(9600, SERIAL_8N1);
    debugSerial.println("Initializing sensor...");

    // Initialize Watchdog Timer
    Lowpower_timer_tick = HAL_GetTick();
    mode = MODE_ON;
    IWatchdog.begin(WATCHDOG_TIMEOUT_MICROSECONDS);

    // Clear sensor serial buffer
    while (sensorSerial.available()) {
        sensorSerial.read();
    }
}

void loop() {
    IWatchdog.reload();
    tick = HAL_GetTick();

    // LED Blink Indication
    digitalWrite(LED_PIN, (tick % 2 == 0) ? HIGH : LOW);
    delay(500);

    switch (mode) {
        case MODE_ON:
            if (ZH10StartMeasurement()) {
                latest_pm1_0 = pm1_0;
                latest_pm2_5 = pm2_5;
                latest_pm10 = pm10;
            }
            if (tick - Lowpower_timer_tick > LOWPOWER_ON_TIME) {
                Lowpower_timer_tick = tick;
                debugSerial.println("Switching to LOW POWER mode");
                digitalWrite(LOWPOWER_SET_PIN, LOW);
                mode = MODE_LOW;
            }
            break;
        
        case MODE_LOW:
            if (tick - Lowpower_timer_tick > LOWPOWER_OFF_TIME) {
                Lowpower_timer_tick = tick;
                debugSerial.println("Switching to ON mode");
                digitalWrite(LOWPOWER_SET_PIN, HIGH);
                mode = MODE_ON;
                while (sensorSerial.available()) {
                    sensorSerial.read(); // Clear RX buffer
                }
            }
            break;
        
        default:
            mode = MODE_LOW;
            break;
    }
}

bool ZH10StartMeasurement() {
    if (sensorSerial.available()) {
        IWatchdog.reload();
        sensorSerial.readBytes(dataBuffer, 32);

        if (dataBuffer[0] == 0x42 && dataBuffer[1] == 0x4D) {
            unsigned int calculatedChecksum = calculateChecksum(dataBuffer, 30);
            unsigned int receivedChecksum = (dataBuffer[30] << 8) | dataBuffer[31];

            if (calculatedChecksum == receivedChecksum) {
                pm1_0 = (dataBuffer[10] << 8) | dataBuffer[11];
                pm2_5 = (dataBuffer[12] << 8) | dataBuffer[13];
                pm10 = (dataBuffer[14] << 8) | dataBuffer[15];

                debugSerial.printf("PM1.0: %d ug/m3, PM2.5: %d ug/m3, PM10: %d ug/m3\n", pm1_0, pm2_5, pm10);
                device_status_error = ERROR_NONE;
                return true;
            } else {
                debugSerial.println("Checksum mismatch");
                device_status_error = ERROR_I2C;
                return false;
            }
        }
        device_status_error = ERROR_I2C;
    }
    return false;
}

void requestEvent() {
  IWatchdog.reload();
  WireSlave.write(i2c_tx_data, I2C_TX_DATA_SIZE);
}
void UpdateTrinityProtocol()
{
  i2c_tx_data[0]  = I2C_TX_DATA_SIZE - 1;
  i2c_tx_data[1]  = (SENSOR_PROCESSOR_MODEL >> 8) & 0xFF;
  i2c_tx_data[2]  = (SENSOR_PROCESSOR_MODEL >> 0) & 0xFF;

  i2c_tx_data[3]  = 0x00; // Data num 0
  i2c_tx_data[4]  = 0x06; // Unsigned int 32 bits
  i2c_tx_data[5]  = (device_status_error >> 24) & 0xFF;
  i2c_tx_data[6]  = (device_status_error >> 16) & 0xFF;
  i2c_tx_data[7]  = (device_status_error >> 8) & 0xFF;
  i2c_tx_data[8]  = (device_status_error >> 0) & 0xFF;

  memcpy(&float2hex, &latest_pm1_0, sizeof(float));
  i2c_tx_data[9]  = 0x01; // Data num 1
  i2c_tx_data[10] = 0x09; // Float
  i2c_tx_data[11] = (float2hex >> 24) & 0xFF;
  i2c_tx_data[12] = (float2hex >> 16) & 0xFF;
  i2c_tx_data[13] = (float2hex >> 8) & 0xFF;
  i2c_tx_data[14] = (float2hex >> 0) & 0xFF;

  memcpy(&float2hex, &latest_pm2_5, sizeof(float));
  i2c_tx_data[15] = 0x02; // Data num 2
  i2c_tx_data[16] = 0x09; // Float
  i2c_tx_data[17] = (float2hex >> 24) & 0xFF;
  i2c_tx_data[18] = (float2hex >> 16) & 0xFF;
  i2c_tx_data[19] = (float2hex >> 8) & 0xFF;
  i2c_tx_data[20] = (float2hex >> 0) & 0xFF;

  memcpy(&float2hex, &latest_pm10, sizeof(float));
  i2c_tx_data[21] = 0x03; // Data num 3
  i2c_tx_data[22] = 0x09; // Float
  i2c_tx_data[23] = (float2hex >> 24) & 0xFF;
  i2c_tx_data[24] = (float2hex >> 16) & 0xFF;
  i2c_tx_data[25] = (float2hex >> 8) & 0xFF;
  i2c_tx_data[26] = (float2hex >> 0) & 0xFF;
}

// Function to calculate checksum for initiative upload mode
unsigned int calculateChecksum(byte *data, int length) {
    unsigned int checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}