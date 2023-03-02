/*
Base on 
https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/examples/ZED-F9P/Example15_NTRIPClient/Example15_NTRIPClient.ino
*/

#include <Arduino.h>

#include <WiFi.h>
#include <ESPmDNS.h>

#include "secrets.h"
#include "constants.h"
#include "Rover.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h" //Built-in ESP32 library
#else
#include <Base64.h> //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long ntripLastReceivedRTCM_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
const unsigned long ntripMaxTimeBeforeHangup_ms = 10000UL; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
const unsigned ntripTimeBeforeReconnect_ms = 5000UL; //If we drop connection, try reconnect in 5s

bool transmitLocation = false;        //By default we will transmit the unit's location via GGA sentence.

WiFiClient ntripClient; // The WiFi connection to the NTRIP server. This is global so pushGGA can see if we are connected.
long ntripCxnDropped_ms = 0;

long mqttLastReconnectAttempt_ms = 0;

WiFiClient wc;
PubSubClient mqttClient(wc);

Rover rover(1,2,3,4,5,6);


void mqttConnect(){
  if (mqttClient.connect(MQTT_CLIENT_NAME)) {
    // Once connected, publish an announcement...
    mqttClient.publish(MQTT_TOPIC_STATUS,"connected");
    // ... and resubscribe
    mqttClient.subscribe(MQTT_TOPIC_CONTROL);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (topic==MQTT_TOPIC_CONTROL){
    DynamicJsonDocument json(1024);
    deserializeJson(json,payload);
    rover.updateSpeed(json["speedL"], json["speedR"])
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callbacks: Wifi connection

void connected_to_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) {
  Serial.println("[+] Connected to the WiFi network");
  Serial.println("[+] Setting up mDNS");
  if (!MDNS.begin(MQTT_CLIENT_NAME)) {
      Serial.println("Error setting up MDNS responder!");
      while(1){
          delay(1000);
      }
  }
  Serial.println("[+] ....done!");

  Serial.println("[+] Setting up MQTT client & connection");

  mqttClient.setServer(mqttBroker,mqttPort);
  mqttConnect();
  mqttClient.setCallback(mqttCallback);
}

void disconnected_from_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) {
  Serial.println("[-] Disconnected from the WiFi AP");
  WiFi.begin(ssid, password);
}

void got_ip_from_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info) { 
  Serial.print("[+] Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}




//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: publishGPGGA will be called when new GPGGA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_GGA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGPGGAcallback
//        /               _____  This _must_ be NMEA_GGA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void pushGPGGA(NMEA_GGA_data_t *nmeaData)
{

  Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
  
    //Provide the caster with our current position as needed
  if ((ntripClient.connected() == true) && (transmitLocation == true))
  {
    Serial.print(F("[->] Pushing GGA to server"));
  
    //Push our current GGA sentence to caster
    ntripClient.print((const char *)nmeaData->nmea);
  }
}



//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: publishPVTdataMqtt will be called when new NAV PVT data arrives.
//
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void publishPVTdataMqtt(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  DynamicJsonDocument doc(1024);
  double latitude = (ubxDataStruct->lat)/10000000.0; // Print the latitude
  double longitude = ubxDataStruct->lon/ 10000000.0; // Print the longitude
  double altitude = (ubxDataStruct->hMSL)/ 1000.0  ; // Print the height above mean sea level
  uint32_t time = ubxDataStruct->iTOW;

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate in mm

  doc["time"] = time;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["altitude"] = altitude;
  doc["hAcc"] = hAcc;
  doc["fixType"] = fixType;
  
  String jsonStr;
  serializeJson(doc, jsonStr);
  int strLen = jsonStr.length() + 1; 

  char json[strLen];
  jsonStr.toCharArray(json,strLen);
  if (mqttClient.connected() == true){
      mqttClient.publish(MQTT_TOPIC_NMEA, json);
      Serial.print(F("[->] Publishing PVT as json to Mqtt"));

  }
}

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 3)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();    
}


 void callbackPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct){
    publishPVTdataMqtt(ubxDataStruct);
    printPVTdata(ubxDataStruct);
 }
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Connect to NTRIP Caster. Return true is connection is successful.
bool beginClient()
{
  Serial.print(F("Opening socket to "));
  Serial.println(casterHost);

  if (ntripClient.connect(casterHost, casterPort) == false) //Attempt connection
  {
    Serial.println(F("Connection to caster failed"));
    return (false);
  }
  else
  {
    Serial.print(F("Connected to "));
    Serial.print(casterHost);
    Serial.print(F(" : "));
    Serial.println(casterPort);

    Serial.print(F("Requesting NTRIP Data from mount point "));
    Serial.println(mountPoint);

    // Set up the server request (GET)
    const int SERVER_BUFFER_SIZE = 512;
    char serverRequest[SERVER_BUFFER_SIZE];
    snprintf(serverRequest,
             SERVER_BUFFER_SIZE,
             "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun u-blox Client v1.0\r\n",
             mountPoint);

    // Set up the credentials
    char credentials[512];
    if (strlen(casterUser) == 0)
    {
      strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
    }
    else
    {
      //Pass base64 encoded user:pw
      char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; //The ':' takes up a spot
      snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

      Serial.print(F("Sending credentials: "));
      Serial.println(userCredentials);

#if defined(ARDUINO_ARCH_ESP32)
      //Encode with ESP32 built-in library
      base64 b;
      String strEncodedCredentials = b.encode(userCredentials);
      char encodedCredentials[strEncodedCredentials.length() + 1];
      strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); //Convert String to char array
#else
      //Encode with nfriendly library
      int encodedLen = base64_enc_len(strlen(userCredentials));
      char encodedCredentials[encodedLen];                                         //Create array large enough to house encoded data
      base64_encode(encodedCredentials, userCredentials, strlen(userCredentials)); //Note: Input array is consumed
#endif

      snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
    }

    // Add the encoded credentials to the server request
    strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
    strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

    Serial.print(F("serverRequest size: "));
    Serial.print(strlen(serverRequest));
    Serial.print(F(" of "));
    Serial.print(sizeof(serverRequest));
    Serial.println(F(" bytes available"));

    // Send the server request
    Serial.println(F("Sending server request: "));
    Serial.println(serverRequest);
    ntripClient.write(serverRequest, strlen(serverRequest));

    //Wait up to 5 seconds for response
    unsigned long startTime = millis();
    while (ntripClient.available() == 0)
    {
      if (millis() > (startTime + 5000))
      {
        Serial.println(F("Caster timed out!"));
        ntripClient.stop();
        return (false);
      }
      delay(10);
    }

    //Check reply
    int connectionResult = 0;
    char response[512];
    size_t responseSpot = 0;
    while (ntripClient.available()) // Read bytes from the caster and store them
    {
      if (responseSpot == sizeof(response) - 1) // Exit the loop if we get too much data
        break;

      response[responseSpot++] = ntripClient.read();

      if (connectionResult == 0) // Only print success/fail once
      {
        if (strstr(response, "200") != NULL) //Look for '200 OK'
        {
          connectionResult = 200;
        }
        if (strstr(response, "401") != NULL) //Look for '401 Unauthorized'
        {
          Serial.println(F("Hey - your credentials look bad! Check your caster username and password."));
          connectionResult = 401;
        }
      }
    }
    response[responseSpot] = '\0'; // NULL-terminate the response

    //Serial.print(F("Caster responded with: ")); Serial.println(response); // Uncomment this line to see the full response

    if (connectionResult != 200)
    {
      Serial.print(F("Failed to connect to "));
      Serial.println(casterHost);
      return (false);
    }
    else
    {
      Serial.print(F("Connected to: "));
      Serial.println(casterHost);
      ntripLastReceivedRTCM_ms = millis(); //Reset timeout
    }
  } //End attempt to connect

  return (true);
} // /beginClient

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Check for the arrival of any correction data. Push it to the GNSS.
//Return false if: the connection has dropped, or if we receive no data for maxTimeBeforeHangup_ms
bool processConnection()
{
  if (ntripClient.connected() == true) // Check that the connection is still open
  {
    uint8_t rtcmData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
    size_t rtcmCount = 0;
 
    //Collect any available RTCM data
    while (ntripClient.available())
    {
      //Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data!
      rtcmData[rtcmCount++] = ntripClient.read();
      if (rtcmCount == sizeof(rtcmData))
        break;
    }

    if (rtcmCount > 0)
    {
      ntripLastReceivedRTCM_ms = millis();

      //Push RTCM to GNSS module over I2C
      myGNSS.pushRawData(rtcmData, rtcmCount);
      
      Serial.print(F("Pushed "));
      Serial.print(rtcmCount);
      Serial.println(F(" RTCM bytes to ZED"));
    }
  }
  else
  {
    Serial.println(F("Connection dropped!"));
    return (false); // Connection has dropped - return false
  }  
  
  //Timeout if we don't have new data for maxTimeBeforeHangup_ms
  if ((millis() - ntripLastReceivedRTCM_ms) > ntripMaxTimeBeforeHangup_ms)
  {
    Serial.println(F("RTCM timeout!"));
    return (false); // Connection has timed out - return false
  }

  return (true);
} // /processConnection

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void closeConnection()
{
  if (ntripClient.connected() == true)
  {
    ntripClient.stop();
  }
  Serial.println(F("Disconnected!"));
}  

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void setup() {
  Serial.begin(115200);
  Serial.println(F("NTRIP testing"));

  Wire.begin(); //Start I2C

  if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("u-blox module connected"));

  // Set up the GNSS

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);                                //Set the I2C port to output both NMEA and UBX messages
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

  myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
  myGNSS.setNavigationFrequency(1); //Set output in Hz.

    // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  myGNSS.setNMEAGPGGAcallbackPtr(&pushGPGGA); // Set up the callback for GPGGA
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C, 10); // Tell the module to output GGA every 10 seconds
  myGNSS.setAutoPVTcallbackPtr(&callbackPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed
  //myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM

  Serial.print(F("Connecting to local WiFi"));

  WiFi.mode(WIFI_STA); //Optional
  WiFi.onEvent(connected_to_ap, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(got_ip_from_ap, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(disconnected_from_ap, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  while (Serial.available()) // Empty the serial buffer
    Serial.read();
}



//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.

  enum states // Use a 'state machine' to open and close the connection
  {
    open_connection,
    push_data,
    close_connection,
    connection_dropped
  };
  static states state = open_connection;

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  switch (state)
  {
    case open_connection:
      Serial.println(F("Connecting to the NTRIP caster..."));
      if (beginClient()) // Try to open the connection to the caster
      {
        Serial.println(F("Connected to the NTRIP caster!"));
        state = push_data; // Move on
      }
      else
      {
        Serial.print(F("Could not connect to the caster. Trying again in 5 seconds."));
        for (int i = 0; i < 5; i++)
        {
          delay(1000);
          Serial.print(F("."));
        }
        Serial.println();
      }
      break;

    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case push_data:
      // If the connection has dropped or timed out
      if (processConnection() == false)
      {
        state = close_connection; // Move on
      }
      break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case close_connection:
      Serial.println(F("Closing the connection to the NTRIP caster..."));
      ntripCxnDropped_ms = millis();
      closeConnection();
      state = connection_dropped; // Move on
      break;
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

    case connection_dropped:
      // If the connection has dropped or timed out, wait for period and try reconnect
      if (millis() - ntripCxnDropped_ms >= ntripTimeBeforeReconnect_ms)
        state = open_connection; // Move on
      break; 
  }


  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  
}
