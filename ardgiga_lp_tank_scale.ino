#include <PubSubClient.h>
#include <WiFi.h>
#include "arduino_secrets.h"
#include <Wire.h>

#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU8702

NAU7802 myScale; //Create instance of the NAU7802 class

//myMem locations to store 4-byte variables
#define myMem_SIZE 100 //Allocate 100 bytes of myMem
#define LOCATION_CALIBRATION_FACTOR 0 //Float, requires 4 bytes of myMem
#define LOCATION_ZERO_OFFSET 10 //Must be more than 4 away from previous spot. int32_t, requires 4 bytes of myMem

bool settingsDetected = false; //Used to prompt user to calibrate their scale

//Create an array to take average of weights. This helps smooth out jitter.
#define AVG_SIZE 4
float avgWeights[AVG_SIZE];
byte avgWeightSpot = 0;

ExternalEEPROM myMem;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = WSSID;        // your network SSID (name)
char pass[] = WPSWD;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Update these with values suitable for your hardware/network.
IPAddress server(192, 168, 1, 100);

WiFiClient wClient;
PubSubClient client(wClient);

long lastReconnectAttempt = 0;

unsigned long lastTimer = 0UL;

boolean reconnect()
{
    if (client.connect("ArduinoScale1"))
    {
        client.subscribe("genscale/#");

    }
    return client.connected();
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

    if (tmpTopic.startsWith("genscale/"))
    {
        String tmpGenScale = "";
        tmpTopic.remove(0,9);
        tmpGenScale = tmpTopic;

    }

}

//Gives user the ability to set a known weight on the scale and calculate a calibration factor
void calibrateScale(void)
{
  Serial.println();
  Serial.println();
  Serial.println(F("Scale calibration"));

  Serial.println(F("Setup scale with no weight on it. Press a key when ready."));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  myScale.calculateZeroOffset(64); //Zero or Tare the scale. Average over 64 readings.
  Serial.print(F("New zero offset: "));
  Serial.println(myScale.getZeroOffset());

  Serial.println(F("Place known weight on scale. Press a key when weight is in place and stable."));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  Serial.print(F("Please enter the weight, without units, currently sitting on the scale (for example '4.25'): "));
  while (Serial.available()) Serial.read(); //Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); //Wait for user to press key

  //Read user input
  float weightOnScale = Serial.parseFloat();
  Serial.println();

  myScale.calculateCalibrationFactor(weightOnScale, 64); //Tell the library how much weight is currently on it
  Serial.print(F("New cal factor: "));
  Serial.println(myScale.getCalibrationFactor(), 2);

  Serial.print(F("New Scale Reading: "));
  Serial.println(myScale.getWeight(), 2);

  recordSystemSettings(); //Commit these values to myMem

  settingsDetected = true;
}

//Record the current system settings to myMem
void recordSystemSettings(void)
{
  //Get various values from the library and commit them to NVM
  myMem.put(LOCATION_CALIBRATION_FACTOR, myScale.getCalibrationFactor());
  myMem.put(LOCATION_ZERO_OFFSET, myScale.getZeroOffset());

  //myMem.commit(); //Some platforms need this. Comment this line if needed
}

//Reads the current system settings from myMem
//If anything looks weird, reset setting to default value
void readSystemSettings(void)
{
  float settingCalibrationFactor; //Value used to convert the load cell reading to lbs or kg
  int32_t settingZeroOffset; //Zero value that is found when scale is tared

  //Look up the calibration factor
  myMem.get(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  if (settingCalibrationFactor == 0xFFFFFFFF)
  {
    settingCalibrationFactor = 1.0; //Default to 1.0
    myMem.put(LOCATION_CALIBRATION_FACTOR, settingCalibrationFactor);
  }

  //Look up the zero tare point
  myMem.get(LOCATION_ZERO_OFFSET, settingZeroOffset);
  if (settingZeroOffset == 0xFFFFFFFF)
  {
    settingZeroOffset = 0; //Default to 0 - i.e. no offset
    myMem.put(LOCATION_ZERO_OFFSET, settingZeroOffset);
  }

  //Pass these values to the library
  myScale.setCalibrationFactor(settingCalibrationFactor);
  myScale.setZeroOffset(settingZeroOffset);

  settingsDetected = true; //Assume for the moment that there are good cal values
  if (settingCalibrationFactor == 1.0 || settingZeroOffset == 0)
    settingsDetected = false; //Defaults detected. Prompt user to cal scale.
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    client.setServer(server, 1883);
    client.setCallback(callback);
    
    myMem.setMemoryType(512);
    if (myMem.begin() == false)
    {
        Serial.println("No memory detected. Freezing.");
        while (true)
        ;
    }
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
    if (myScale.begin() == false)
    {
        Serial.println("Scale not detected. Please check wiring. Freezing...");
        while (1);
    }
    Serial.println("Scale detected!");
    readSystemSettings(); //Load zeroOffset and calibrationFactor from myMem

    myScale.setSampleRate(NAU7802_SPS_320); //Increase to max sample rate
    myScale.calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channel 

    Serial.print("Zero offset: ");
    Serial.println(myScale.getZeroOffset());
    Serial.print("Calibration factor: ");
    Serial.println(myScale.getCalibrationFactor());

    Serial.println("\r\nPress 't' to Tare or Zero the scale.");
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
    if (myScale.available() == true)
  {
    int32_t currentReading = myScale.getReading();
    float currentWeight = myScale.getWeight();

    Serial.print("Reading: ");
    Serial.print(currentReading);
    Serial.print("\tWeight: ");
    Serial.print(currentWeight, 2); //Print 2 decimal places

    avgWeights[avgWeightSpot++] = currentWeight;
    if(avgWeightSpot == AVG_SIZE) avgWeightSpot = 0;

    float avgWeight = 0;
    for (int x = 0 ; x < AVG_SIZE ; x++)
      avgWeight += avgWeights[x];
    avgWeight /= AVG_SIZE;

    Serial.print("\tAvgWeight: ");
    Serial.print(avgWeight, 2); //Print 2 decimal places

    if(settingsDetected == false)
    {
      Serial.print("\tScale not calibrated. Press 'c'.");
    }

    Serial.println();
}

if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 't') //Tare the scale
      myScale.calculateZeroOffset();
    else if (incoming == 'c') //Calibrate
    {
      calibrateScale();
    }
  }
}

