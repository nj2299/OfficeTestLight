/*Top Comment Lines
 * 
 * 
 * Test light
 * 
 * 
 * Published March 2020
 * Author: NJ
 */
/*************************Headers***************************/
#include <Time.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>    //Wifi header
#include <ESP8266WiFiMulti.h> 
#include <WiFiUdp.h>      //Used to maintain time (sync to NTP server)
#include <ESP8266HTTPClient.h>  //create web clients
#include <ESP8266httpUpdate.h>  //over the air updates
#include <PubSubClient.h>     //MQTT library
//#include <Adafruit_NeoPixel.h>  //Neopixel light control
#include <NeoPixelBus.h>    //Neopixel library that uses DMA (Direct Memory Access)
#include <WiFiConnect.h>
#include <Esp.h>
/*************************Constants***************************/
#define PixelCount 24
//#define PixelCount 58
//#define LED_PIN D2    //control pin from ESP
#define TIMER_MS 5000
#define MQTT_KEEPALIVE 120
#define MQTT_MAX_PACKET_SIZE 512

/*************************global Variables***************************/
const char* mqttServer = "192.168.1.13";
const int mqttPort = 1883;
const char* clientName = "";  //these three variables used for setting the client name to the Macaddress
String topicString;
char topicChar[18];
int segmultiplier = PixelCount/12;

const char* topic_sub_roomupdate = "OfficeTestLight/roomupdate";  //listen to this topic
const char* topic_sub_Heartbeat = "OfficeTestLight/Hearbeat"; 
const char* topic_pub = "OfficeTestLight/status";
const char* topic_sub_firmware = "OfficeTestLight/commands/firmware";  //listen for firmware update
bool command = 0;           //is there a current meeting
bool prevcommand = 0;
bool statechange = 1;
bool Heartbeat = 0;
bool Firmware =0;
bool colorchange = 0;
int Rcolor = 0; 
int Gcolor = 0; 
int Bcolor = 0; 
int Wcolor = 0; 
unsigned long connect_time;
unsigned long heartbeat_reset;

WiFiClient espClient;         //wifi client
PubSubClient client(espClient); //MQTT client requires wifi client
ESP8266WiFiMulti wifiMulti;     //creates instance of wifi multi class
/*************************Setup time server******************************************/
//setup time server
WiFiUDP UDP;   //creates instance of UDP class to send and recieve 
IPAddress timeServerIP;
const char* NTPServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;  // NTP time stamp is in the first 48 bytes of the message
byte NTPBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
unsigned long intervalNTP = 60000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;
uint32_t actualTime=0;
unsigned long prevActualTime = 0;

/************************setup light strip*****************************************/
// For Esp8266, the Pin is omitted and it uses GPIO3 due to DMA hardware use.  
NeoPixelBus<NeoGrbwFeature, NeoWs2813Method> strip(PixelCount); // 3 pixel LED strip (rgb)
//NeoPixelBus<NeoRgbwFeature, NeoWs2813Method> strip(PixelCount);  //rgbw pixel strip
#define colorSaturation 255

RgbwColor red(colorSaturation, 0, 0,0);
RgbwColor green(0, colorSaturation, 0,0);
RgbwColor blue(0, 0, colorSaturation,0);
RgbwColor white(0,0,0,colorSaturation);
RgbwColor purple(128,0,128,0);
RgbwColor black(0);

/****************setup wifi************************************/
void setup_wifi() {
  unsigned long currentMillis=0;
  unsigned long startTimer = millis();
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
//  WiFiMulti.addAP(ssid, password);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
          currentMillis = millis();
      if ((currentMillis - startTimer) > 60000) {                                     //frozen for 10 minutes, restart
      Serial.println("More than 10 minutes since last NTP response. Rebooting.");
      Serial.flush();
      ESP.restart();
  }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  long rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);

  
  topicString = WiFi.macAddress();          //sets up the unique clientName using the MacAddress
  topicString.toCharArray(topicChar, 18);
  clientName = topicChar;
  Serial.println(topicChar);
}

/*****************Firmware UPdate**********************************/

void updateFirmware(){
  
 // t_httpUpdate_return ret = ESPhttpUpdate.update("http://99.231.13.167/update");
 //   t_httpUpdate_return ret = ESPhttpUpdate.update("http://nj2299.duckdns.org:1881/BasementTVLightUpdate");
 t_httpUpdate_return ret = ESPhttpUpdate.update("http://nodered:J1Gal00@192.168.1.13:1880/endpoint/OfficeTestLightUpdate");

      Serial.println(ret);
        switch(ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;
                
            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;

            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("NO UPDATES");
                break;

           default:
              Serial.println("Something else...");
              break;
        }
}


/*****************Connect to MQTT Broker**********************************/
void ConnectBroker(PubSubClient client, const char* clientName)
{
    while (!client.connected())
    {
        Serial.print("Connecting to MQTT: ");
        Serial.println(clientName);
        if(client.connect(clientName,"mqtt","J1g@l00"))      //command to connect to MQTT broker with the unique client name
        {
          Serial.println("Connected");
        }
        else
        {
          Serial.print("Failed with state ");
          Serial.println(client.state());
          delay(200);
        }
    }
} 
/*****************reconnect to MQTT Broker if it goes down**********************************/
void reconnect() {
  // Loop until we're reconnected
//  delay(10);
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//
//
//  Serial.println("");
//  Serial.println("WiFi connected");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());
//  long rssi = WiFi.RSSI();
//  Serial.print("RSSI:");
//  Serial.println(rssi);
//  
  
  unsigned long currentMillis=0;
  unsigned long startTimer = millis();
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect, just a name to identify the client
    if (client.connect(clientName)) {
      Serial.println("connected");
      
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      currentMillis = millis();
      if ((currentMillis - startTimer) > 60000) {                                     //frozen for 10 minutes, restart
      Serial.println("More than 10 minutes since last NTP response. Rebooting.");
      Serial.flush();
      ESP.restart();
  }
    }
  }
}


/************************setup lights***********************************/
void setup_lights(){
  strip.Begin();
  strip.Show(); // Initialize all pixels to 'off'
}

/*****************MQTT Listener******************************************************/
void callback(char* topic, byte* payload, unsigned int length2){
  //topicString = WiFi.macAddress();
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
   
  Serial.print("Message: ");
  for(int i = 0; i<length2;i++){
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-------------");
  payload[length2] = 0;
  StaticJsonBuffer<500> JSONbuffer; 
  String inData = String((char*)payload);
  JsonObject& root = JSONbuffer.parseObject(inData);

  String ID = root["ID"];
  if(ID == "All" || ID == topicString){
    command = root["command"];  //binary 0 or 1
    Heartbeat = root["Heartbeat"];
    Firmware = root["Firmware"];
    Rcolor = root["Red"];
    Gcolor = root["Green"];
    Bcolor = root ["Blue"];
    Wcolor = root ["White"];
    colorchange = root["colorchange"];
     
    if (Heartbeat !=1){
      if (command != prevcommand){
      statechange = 1;
      prevcommand = command;
      Serial.println(command);
      Serial.println(statechange);
    }
   }

     if (Heartbeat == 1){
      sendStartupMessage();
      Heartbeat = 0;
      heartbeat_reset = millis();
     }
  
    if(Firmware == 1){
      updateFirmware();
      Firmware = 0;
    }
    messageRecieved(); 
  }
  Serial.flush();
}

/***************message recieved******************************************************/
//To universal Topic
void messageRecieved(){
  
  StaticJsonBuffer<100> JSONbuffer;            //Creates JSON message
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["id"] = clientName;
  JSONencoder["message"] = "recieved";
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  client.publish(topic_pub, JSONmessageBuffer, false);
  
}

/***************Startup Message - sent on bootup connection******************************************************/
//To universal Topic
void sendStartupMessage(){
  
  StaticJsonBuffer<100> JSONbuffer;            //Creates JSON message
  JsonObject& JSONencoder = JSONbuffer.createObject();
  JSONencoder["id"] = clientName;
  JSONencoder["Time"] = actualTime;
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  client.publish(topic_pub, JSONmessageBuffer, false);
  
}


/***************Effect Control*****************************************************/
  void effect_control (){
   RgbwColor color(Rcolor,Gcolor,Bcolor,Wcolor);
   colorchange = 0;
//if (command==1 && statechange==1)
    if (command==1){
     LightOutMiddle (color);
     //full_on();
     //half_on();
      //colorWipe(white,50);
      Serial.println("lights on");
      statechange = 0;
      digitalWrite(LED_BUILTIN, LOW);
      Serial.flush();
    }

    if (command==0){
      clear_strip();
      //LightOutMiddle (black);
      Serial.println("lights off");
      statechange = 0;
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.flush();
    }


  }

/************************Time Functions***********************************/
void startUDP() {
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages on port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  Serial.println();
}

uint32_t getTime() {
  if (UDP.parsePacket() == 0) { // If there's no response (yet)
    return 0;
  }
  UDP.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
  // Combine the 4 timestamp bytes into one 32-bit number
  uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
  // Convert NTP time to a UNIX timestamp:
  // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t UNIXTime = NTPTime - seventyYears;
  return UNIXTime;
  Serial.flush();
}

void sendNTPpacket(IPAddress& address) {
  memset(NTPBuffer, 0, NTP_PACKET_SIZE);  // set all bytes in the buffer to 0
  // Initialize values needed to form NTP request
  NTPBuffer[0] = 0b11100011;   // LI, Version, Mode
  // send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); // NTP requests are to port 123
  UDP.write(NTPBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

/************************LIGHT EFFECTS***********************************/

// start in the middle and fill outwards
void LightOutMiddle(RgbwColor c) {
  int LED = PixelCount/2;
  int LED2 = LED-1;
  if(LED%2==0){                       //checks if LEDS are even or odd
    strip.SetPixelColor(LED, c);
    strip.SetPixelColor(LED2, c);
   delay(1);
    strip.Show();
    stall(4);
    
  }

  else {
    strip.SetPixelColor(LED, c);
    delay(1);
    strip.Show();
    stall(4);
  }
  for(uint16_t i=0; i<(PixelCount/2)+1; i++) {
    strip.SetPixelColor(LED+i,c);
    if(LED%2==0){
      strip.SetPixelColor(LED2-i,c);
    }
    else{
      strip.SetPixelColor(LED-i,c);
    }
    delay(1);
    strip.Show();
    stall(4);
    
    }

}

// Fill the dots one after the other with a color
void colorWipe(RgbwColor c, uint8_t wait) {
  for(uint16_t i=0; i<PixelCount; i++) {
    strip.SetPixelColor(i, c);
    delay(1);
    strip.Show();
    delay(wait);
    }
  
//   stall(40); 
//   clear_strip();
}

void colorWipeReverse (RgbwColor c, uint8_t wait) {
  for(uint16_t i= 0 ; i<PixelCount+1; i++) {
    strip.SetPixelColor(PixelCount-i,c);
    strip.Show();
    delay(wait);
    }
   stall(40); 
   clear_strip();
}

void stall(uint16_t s){
  unsigned long currentMillis=0;
  unsigned long startTimer = millis();
  for (uint8_t i=0; i<=s; i++){
     while(startTimer - currentMillis < 810){
     currentMillis = millis();
     client.loop(); 
    }
  }
}


void clear_strip(){
  for (uint16_t i=0; i<PixelCount; i++){
    strip.SetPixelColor(i,black);
    
  }
  delay(1);
  //strip.Show();
}

void full_on(){
  for (uint16_t i=0; i<PixelCount; i++){
    strip.SetPixelColor(i,white);
  }
  delay(1);
 // strip.Show();
}

void half_on(){
  for (uint16_t i=0; i<PixelCount; i++){
    if(i%2==0){
      strip.SetPixelColor(i,white);
    }
    
  }
  delay(1);
  strip.Show();
}


/************************SETUP***********************************/
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer,mqttPort);
  ConnectBroker(client, clientName);    //connect to MQTT borker
  client.setCallback(callback);
  client.subscribe(topic_sub_roomupdate); 
  client.subscribe(topic_sub_firmware);
  client.subscribe (topic_sub_Heartbeat);
  setup_lights();  
  startUDP();
  
  if(!WiFi.hostByName(NTPServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }
  Serial.print("Time server IP:\t");
  Serial.println(timeServerIP);
  
  Serial.println("\r\nSending NTP request ...");
  sendNTPpacket(timeServerIP);  

 sendStartupMessage();
 pinMode(LED_BUILTIN, OUTPUT);
 digitalWrite(LED_BUILTIN, HIGH);
  

}

/************************LOOP***********************************/
void loop() {
//now = millis();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  strip.Show();

  unsigned long currentMillis = millis();

  if (currentMillis - prevNTP > intervalNTP) { // If a minute has passed since last NTP request
    prevNTP = currentMillis;
    Serial.println("\r\nSending NTP request ...");
    sendNTPpacket(timeServerIP);               // Send an NTP request
    Serial.flush();
  }

  uint32_t time = getTime();                   // Check if an NTP response has arrived and get the (UNIX) time
  if (time) {                                  // If a new timestamp has been received
    timeUNIX = time;
    Serial.print("NTP response:\t");
    Serial.println(timeUNIX);
    lastNTPResponse = currentMillis;
  } else if ((currentMillis - lastNTPResponse) > 3600000) {
    Serial.println("More than 1 hour since last NTP response. Rebooting.");
    Serial.flush();
    ESP.restart();
  }

  if(currentMillis - heartbeat_reset > 900000){
    Serial.println("three heartbeats missed");
    Serial.flush();
    ESP.restart();
  }

  actualTime = timeUNIX + (currentMillis - lastNTPResponse)/1000;
  if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
    prevActualTime = actualTime;
    if (colorchange == 1){
      effect_control();
    }
  }
}
