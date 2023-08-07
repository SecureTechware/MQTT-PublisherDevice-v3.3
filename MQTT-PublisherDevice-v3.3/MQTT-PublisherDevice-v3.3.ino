// MCP2515 Connection with ESP32
/*
 * MCP2515  =     ESP32
 * VCC      =       5V
 * GND      =      GND
 * CS       =       P5
 * SO       =      P19
 * SI       =      P23
 * SCK      =      P18
 * INT      =      P22
 */
#define TINY_GSM_MODEM_SIM7000

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false


// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "";
const char wifiPass[] = "";

long unsigned int rxId; // for CAN Packet ID
unsigned char len = 0; // Length of CAN Packet
unsigned char rxBuf[8]; // Default Buffer Size


                        

//MQTT Server Connection
#define mqtt_server "broker.hivemq.com"
#define owner "ArsalanSaleemSecureTechwarePublisher_3_3"
#define mqtt_user "ArsalanSaleemSecureTechwarePublisher_3_3"
#define mqtt_password "ArsalanSaleemSecureTechware_3_3"
#define status_topic "SecureTechware_MCP2515_Status_3_3"
#define gps_data_topic "SecureTechware_GPS_Data_3_3"
#define data_topic "SecureTechware_MCP2515_Data_3_3"
#define mqtt_port 1883

#include <TinyGsmClient.h> //Include TinyGSM Library
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <Ticker.h>
#include <SPI.h>
#include <mcp_can.h>  // https://github.com/coryjfowler/MCP_CAN_lib


#define CS 5  // Define CS PIN
MCP_CAN CAN0(CS);  // Initialize MCP_CAN      


// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);



Ticker tick;



#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD   9600
#define PIN_DTR     25
#define PIN_TX      27
#define PIN_RX      26
#define PWR_PIN     4

#define LED_PIN             12


uint32_t lastReconnectAttempt = 0;
String SendData;
void enableGPS(void)
{
    // Set Modem GPS Power Control Pin to HIGH ,turn on GPS power
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+CGPIO=0,48,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG("Set GPS Power HIGH Failed");
    }
    modem.enableGPS();
}

boolean mqttConnect()
{
    SerialMon.print("Connecting to ");
    SerialMon.print(mqtt_server);

    // Connect to MQTT Broker
    //boolean status = mqtt.connect("GsmClientTest");

    // Or, if you want to authenticate MQTT:
     boolean status = mqtt.connect(owner,mqtt_user, mqtt_password);

    if (status == false) {
        SerialMon.println(" fail");
        return false;
    }
    SerialMon.println(" success");
    //mqtt.publish(topicInit, "GsmClientTest started");
    mqtt.subscribe(status_topic);
    return mqtt.connected();
}


void setup()
{
    // Set console baud rate
    Serial.begin(115200);
    delay(10);

    pinMode(CS, OUTPUT);  
    // Initialize MCP2515 running at MCP_8MHZ or MCP_16MHZ with a baudrate of 500kb/s and the masks and filters disabled.
    if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
    else Serial.println("Error Initializing MCP2515");

    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

    pinMode(22, INPUT);                            // Setting pin 22 for /INT input

    Serial.println("ESP32 --> MCP2515 --> MQTT Initialization. Complete Publisher Successed.");


    Serial.println("\nWait...");

    delay(1000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    if (!modem.restart()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting");
    }



    String name = modem.getModemName();
    DBG("Modem Name:", name);

    String modemInfo = modem.getModemInfo();
    DBG("Modem Info:", modemInfo);

#if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if (GSM_PIN && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
    }
#endif

#if TINY_GSM_USE_WIFI
    // Wifi connection parameters must be set before waiting for the network
    SerialMon.print(F("Setting SSID/password..."));
    if (!modem.networkConnect(wifiSSID, wifiPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
    // The XBee must run the gprsConnect function BEFORE waiting for network!
    modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork()) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

#if TINY_GSM_USE_GPRS
    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected()) {
        SerialMon.println("GPRS connected");
    }
#endif

    // MQTT Broker setup
    mqtt.setServer(mqtt_server, 1883);
   

}

void loop()
{


    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
        SerialMon.println("Network disconnected");
        if (!modem.waitForNetwork(180000L, true)) {
            SerialMon.println(" fail");
            delay(10000);
            return;
        }
        if (modem.isNetworkConnected()) {
            SerialMon.println("Network re-connected");
        }

#if TINY_GSM_USE_GPRS
        // and make sure GPRS/EPS is still connected
        if (!modem.isGprsConnected()) {
            SerialMon.println("GPRS disconnected!");
            SerialMon.print(F("Connecting to "));
            SerialMon.print(apn);
            if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
                SerialMon.println(" fail");
                delay(10000);
                return;
            }
            if (modem.isGprsConnected()) {
                SerialMon.println("GPRS reconnected");
            }
        }
#endif
    }

    if (!mqtt.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            lastReconnectAttempt = t;
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        delay(100);
        return;
    }
String DATENSATZ;  //Temp Variable
    
    if(!digitalRead(22))                        // If pin 22 is low, read receive buffer
    {
      CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
     
      DATENSATZ+=String(rxId,HEX).c_str(); //Set ID
      DATENSATZ+=String("#").c_str();

      for(int i=0;i<len;i++){
        DATENSATZ+=String(rxBuf[i],HEX).c_str(); //Set CAN Frame
        DATENSATZ+=" ";
      }

      if(SendData!=DATENSATZ){
            SendData=DATENSATZ;
            Serial.println(SendData); 
            mqtt.publish(data_topic, String(SendData).c_str(), true); // MQTT-Server send
      }
    }

  enableGPS();

    /*GPS data*/  
  String Location="";
  float lat, lon;
    while (1)
    {
      if (modem.getGPS(&lat, &lon))
      {
        Location= "Lat: "+ String(lat) + " "+ "Long: "+ String(lon) ;
        break;
       }else{
          Location="GPS Data Not Available";
       }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    }
  



    mqtt.loop();

}
