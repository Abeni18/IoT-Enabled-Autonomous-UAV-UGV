/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <dht.h>

// Update these with values suitable for your network.

const char* ssid = "NETGEAR28";
const char* password = "excitedocean856";

//const char* mqtt_server = "broker.mqtt-dashboard.com";

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char* mqtt_server = "192.168.1.19";
//const char* mqtt_server = "10.0.0.9";


//Sensor Setup 
dht DHT;//create a variable type of dht
const int DHT11_PIN= 4;//Humiture sensor attach to pin7
float temperature = 0;
float humidity = 0;



// Initializes the espClient
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

// connect LED and Sound to pin D3 and D4 respectively
const int ledPin = 0;  // D3
const int soundPin = 2; //D4

void setup_wifi() {

  delay(20);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  delay(20);
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "UAV_2/UGV_3/LED_Red") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }


  if (String(topic) == "UAV_2/UGV_3/Sound") {
    Serial.print("Sound output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(soundPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(soundPin, LOW);
    }
  }

  if (String(topic) == "UAV_2/UGV_3/Control") {
    Serial.print("command output ");
    //Serial.println(messageTemp);
    if(messageTemp == "8"){
      Serial.println(8);
      delay(10);
    }
    else if (messageTemp = "5"){
      Serial.println(5);
      delay(10);
    }
    delay(10);
     }
  
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    /*
     YOU  NEED TO CHANGE THIS NEXT LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
     To change the ESP device ID, you will have to give a unique name to the ESP8266.
     Here's how it looks like now:
       if (client.connect("ESP8266Client")) {
     If you want more devices connected to the MQTT broker, you can do it like this:
       if (client.connect("ESPOffice")) {
     Then, for the other ESP:
       if (client.connect("ESPGarage")) {
      That should solve your MQTT multiple connections problem

     THE SECTION IN loop() function should match your device name
    */
    if (client.connect("ESP8266Client_3")) {
      Serial.println("connected");
//////////////////////////////      // Subscribe               /////////////////////////
      client.subscribe("UAV_2/UGV_3/LED_Red");
      client.subscribe("UAV_2/UGV_3/Sound");
      client.subscribe("UAV_2/UGV_3/Control");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(50);
    }
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(soundPin, OUTPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  //client.loop();
  
  
 
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    
    // Temperature in Celsius   
    int chk = DHT.read11(DHT11_PIN);//read the value returned from sensor
    switch (chk)
    { case DHTLIB_ERROR_CHECKSUM:
      temperature = DHT.temperature ; 
      humidity = DHT.humidity ;      
      delay(10); //wait a while 
      break;
      }
   
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
//    Serial.print("Temperature: ");
//    Serial.println(tempString);
    client.publish("UAV_2/UGV_3/temperature", tempString);

    // Convert the value to a char array
    char humString[8];
    dtostrf(humidity, 1, 2, humString);
//    Serial.print("Humidity: ");
//    Serial.println(humString);
    client.publish("UAV_2/UGV_3/humidity", humString);
  }
  if(!client.loop())
  client.connect("ESP8266Client_3");
}
