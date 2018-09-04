/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
    #include <avr/power.h>
#endif

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 2;      // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define PIN 22
#define NUMPIXELS 144
int delayval = 500; // delay for half a second
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};
void colorWipe(uint32_t c, uint8_t wait)
{
    for (uint16_t i = 0; i < pixels.numPixels()+1; i++)
    {
        pixels.setPixelColor(i, c);
        pixels.show();
        delay(wait);
    }
}

uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbow(uint8_t wait)
{
    uint16_t i, j;

    for (j = 0; j < 256; j++)
    {
        for (i = 0; i < pixels.numPixels(); i++)
        {
            pixels.setPixelColor(i, Wheel((i + j) & 255));
        }
        pixels.show();
        delay(wait);
    }
}

void theaterChase(uint32_t c, uint8_t wait)
{
    for (int j = 0; j < 10; j++)
    { //do 10 cycles of chasing
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 0; i < pixels.numPixels(); i = i + 3)
            {
                pixels.setPixelColor(i + q, c); //turn every third pixel on
            }
            pixels.show();

            delay(wait);

            for (uint16_t i = 0; i < pixels.numPixels(); i = i + 3)
            {
                pixels.setPixelColor(i + q, 0); //turn every third pixel off
            }
        }
    }
}

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0)
        {
            Serial.print("Received Value: ");

            for (int i = 0; i < rxValue.length(); i++)
            {
                Serial.print(rxValue[i]);
            }

            Serial.println();

            // Do stuff based on the command received from the app
            if (rxValue.find("A") != -1)
            {
                Serial.print("Turning ON!");

                colorWipe(pixels.Color(0, 255, 255), 50); // Green

                Serial.print("koniec animacji on");
            }
            else if (rxValue.find("B") != -1)
            {
                Serial.print("Turning OFF!");

                colorWipe(pixels.Color(0, 255, 0), 50); // Green

                Serial.print("koniec animacji off");
            }

            Serial.println();
            Serial.println("*********");
        }
    }
};

void setup()
{
    Serial.begin(115200);
    pixels.begin();

    // Create the BLE Device
    BLEDevice::init("ESP32 UART Test"); // Give it a name

    // Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setCallbacks(new MyCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
}

void loop()
{
    if (deviceConnected)
    {
        // Fabricate some arbitrary junk for now...
        txValue = analogRead(readPin) / 3.456; // This could be an actual sensor reading!

        // Let's convert the value to a char array:
        char txString[8];                 // make sure this is big enuffz
        dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer

        //    pCharacteristic->setValue(&txValue, 1); // To send the integer value
        //    pCharacteristic->setValue("Hello!"); // Sending a test message
        pCharacteristic->setValue(txString);

        pCharacteristic->notify(); // Send the value to the app!
        // Serial.print("*** Sent Value: ");
        // Serial.print(txString);
        // Serial.println(" ***");

        // You can add the rxValue checks down here instead
        // if you set "rxValue" as a global var at the top!
        // Note you will have to delete "std::string" declaration
        // of "rxValue" in the callback function.
        //    if (rxValue.find("A") != -1) {
        //      Serial.println("Turning ON!");
        //      colorWipe(pixels.Color(255, 255, 0), 2);
        //    }
        //    else if (rxValue.find("B") != -1) {
        //      Serial.println("Turning OFF!");
        //      colorWipe(pixels.Color(0, 0, 255), 2);
        //    }
    }
    delay(1000);
}