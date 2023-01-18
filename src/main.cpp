#include <Arduino.h>

#include <knx.h>
#define PIN_SPI1_MISO (28u)
#define PIN_SPI1_MOSI (27u)
#define PIN_SPI1_SCK (26u)
#define PIN_SPI1_SS (25u)
#include <Ethernet_Generic.h>

#define NUMBER_OF_MAC 20

#define VERSION_MAJOR 0
#define VERSION_MINOR 2

byte mac[][NUMBER_OF_MAC] =
    {
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x01},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x02},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x03},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x04},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x05},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x06},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x07},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x08},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x09},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0A},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0B},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0C},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0D},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x0E},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x0F},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x11},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x12},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x13},
        {0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x14},
};

#define PIN_PROG_SWITCH 5
#define PIN_PROG_LED 11

// ITSY BITSY
#define PIN_TPUART_RX 1
#define PIN_TPUART_TX 0

// EC-KNX-2040
// #define PIN_TPUART_RX 13              // stm32 knx uses Serial2 (pins 16,17)
// #define PIN_TPUART_TX 12

// Restore ram after reset (brownout)
#define INIT_MASK 0x12345678
volatile uint32_t Inited __attribute__((section(".noinit")));

static SerialUART serialTpuart(uart0, PIN_TPUART_TX, PIN_TPUART_RX);

/*
EthernetUDP udp;
uint8_t test[24] = "\0";
IPAddress mcastaddr = IPAddress(htonl((uint32_t)0xE000170C));
int port = 3671;
uint8_t result;
*/

void setup()
{
    Serial.begin(115200);
    ArduinoPlatform::SerialDebug = &Serial;
    while (!Serial)
    {
        delay(10);
    }
    delay(300);
    Serial.println("Loading ....");

    // Prepare KNX interface
    knx.platform().knxUart(&serialTpuart);
    knx.ledPin(PIN_PROG_LED);
    knx.ledPinActiveOn(HIGH);
    knx.buttonPin(PIN_PROG_SWITCH);

    // Init KNX
    knx.version((VERSION_MAJOR << 6) | (VERSION_MINOR & 0x3F)); // PID_VERSION
    knx.orderNumber((const uint8_t *)"50000");                  // PID_ORDER_INFO
    knx.manufacturerId(0xfa);                                   // PID_SERIAL_NUMBER (2 first bytes) - 0xfa for KNX Association
    knx.hardwareType((const uint8_t *)"M-091A");                // PID_HARDWARE_TYPE

    // Prepare Ethernet
    randomSeed(millis());
    pinMode(USE_THIS_SS_PIN, OUTPUT);
    digitalWrite(USE_THIS_SS_PIN, HIGH);
    Ethernet.init(USE_THIS_SS_PIN);
    Serial.println(F("Initialized "));
    uint16_t index = millis() % NUMBER_OF_MAC;
    // Use Static IP
    // Ethernet.begin(mac[index], ip);
    Ethernet.begin(mac[index]);
    Serial.print(F("Using mac index = "));
    Serial.println(index);

    Serial.print(F("Connected! IP address: "));
    Serial.println(Ethernet.localIP());

    if ((Ethernet.getChip() == w5500) || (Ethernet.getChip() == w6100) || (Ethernet.getAltChip() == w5100s))
    {
        if (Ethernet.getChip() == w6100)
            Serial.print(F("W6100 => "));
        else if (Ethernet.getChip() == w5500)
            Serial.print(F("W6100 => "));
        else
            Serial.print(F("W5100S => "));

        Serial.print(F("Speed: "));
        Serial.print(Ethernet.speedReport());
        Serial.print(F(", Duplex: "));
        Serial.print(Ethernet.duplexReport());
        Serial.print(F(", Link status: "));
        Serial.println(Ethernet.linkReport());
    }

    /*result = udp.beginMulticast(mcastaddr, port);
    KNX_DEBUG_SERIAL.printf("Setup Mcast addr: ");
    mcastaddr.printTo(KNX_DEBUG_SERIAL);
    KNX_DEBUG_SERIAL.printf(" on port: %d result %d\n", port, result);
    result = udp.beginPacket(mcastaddr, port);
    // KNX_DEBUG_SERIAL.printf("begin:%d ", result);
    udp.write(test, 24);
    result = udp.endPacket();
    // KNX_DEBUG_SERIAL.printf("end:%d \n", result);
*/
    // read adress table, association table, groupobject table and parameters from eeprom
    knx.readMemory();

    // print values of parameters if device is already configured
    if (knx.configured())
    {
        Serial.println("Coupler configured.");

        // KNX_DEBUG_SERIAL.printf("IP Address: %x.%x.%x.%x\n", ip[0], ip[1], ip[2], ip[3]);
    }
    else
    {
        Serial.println("Coupler not configured");
    }

    // pin or GPIO the programming led is connected to. Default is LED_BUILTIN
    // knx.ledPin(LED_BUILTIN);
    // is the led active on HIGH or low? Default is LOW
    // knx.ledPinActiveOn(HIGH);
    // pin or GPIO programming button is connected to. Default is 0
    // knx.buttonPin(0);

    // start the framework.
    knx.start();
}

void loop()
{
    // don't delay here to much. Otherwise you might lose packages or mess up the timing with ETS
    knx.loop();

    // only run the application code if the device was configured with ETS
    if (!knx.configured())
        return;

    // Send UDP packet every second
    /*if (millis() % 1000)
    {
        result = udp.beginPacket(mcastaddr, port);
        // KNX_DEBUG_SERIAL.printf("begin:%d ", result);
        udp.write(test, 24);
        result = udp.endPacket();
        // KNX_DEBUG_SERIAL.printf("end:%d \n", result);
    }
    */
}
