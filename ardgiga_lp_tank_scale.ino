#include <PubSubClient.h>
#include <WiFi.h>
#include "arduino_secrets.h"
#include <ModbusRTUMaster.h>

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

#define MODBUS_DE 2
#define MODBUS_BAUD 9600
ModbusRTUMaster modbus(Serial1, MODBUS_DE); // serial port, driver enable pin for rs-485 (optional)

// Update these with values suitable for your hardware/network.
IPAddress server(192, 168, 1, 100);

WiFiClient wClient;
PubSubClient client(wClient);

long lastReconnectAttempt = 0;

unsigned long lastTimer = 0UL;


// Modbus
bool coils[2];
bool discreteInputs[2];
uint16_t holdingRegisters[2];
uint16_t inputRegisters[3];

boolean reconnect()
{
    if (client.connect("ArduinoTrain1"))
    {
        client.subscribe("trains/track/turnout/#");
        client.subscribe("trains/track/light/#");

    }
    return client.connected();
}

bool turnout2Bool(String strState)
{
    if (strState == "THROWN")
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool light2Bool(String strState)
{
    if (strState == "ON")
    {
        return true;
    }
    else
    {
        return false;
    }
}

String bool2Sensor(bool inState)
{
    if (inState == true)
    {
        return "ACTIVE";
    }
    else
    {
        return "INACTIVE";
    }
}

void callback(char* topic, byte* payload, unsigned int length)
{
    String tmpTopic = topic;
    char tmpStr[length+1];
    for (int x=0; x<length; x++)
    {
        tmpStr[x] = (char)payload[x]; // payload is a stream, so we need to chop it by length.
    }
    tmpStr[length] = 0x00; // terminate the char string with a null

    if (tmpTopic.startsWith("trains/track/light/"))
    {
        int tmpLight = 0;
        tmpTopic.remove(0,19);
        tmpLight = tmpTopic.toInt();

    }
    else if (tmpTopic.startsWith("trains/track/turnout/"))
    {
        int tmpTurnout = 0;
        tmpTopic.remove(0,21);
        tmpTurnout = tmpTopic.toInt();

    }

}



void setup()
{
    client.setServer(server, 1883);
    client.setCallback(callback);

    modbus.begin(MODBUS_BAUD); // baud rate, config (optional)

// check for the WiFi module:
    if (WiFi.status() == WL_NO_SHIELD)
    {
        //Serial.println("Communication with WiFi module failed!");
        // don't continue
        while (true);
    }

    // attempt to connect to Wifi network:
    while (status != WL_CONNECTED)
    {
        //Serial.print("Attempting to connect to SSID: ");
        //Serial.println(ssid);
        // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
        status = WiFi.begin(ssid, pass);

        // wait 3 seconds for connection:
        delay(3000);
    }
    delay(1500);
    lastReconnectAttempt = 0;

    for (int i = 22; i <= 34; i++)
    {
        pinMode(i, OUTPUT);
        digitalWrite(i,HIGH);
    }


}

void loop()
{
    if (!client.connected())
    {
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect())
            {
                lastReconnectAttempt = 0;
            }
        }
    }
    else
    {
        // Client connected

        client.loop();
    }

    if (millis() - lastTimer >= 500)
    {
        /** \brief setup current values
         *
         *
         */
        lastTimer = millis();
        modbus.readDiscreteInputs(1, 0, discreteInputs, 3);


    }
}
