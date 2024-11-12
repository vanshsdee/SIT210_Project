#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

// WiFi and MQTT setup
const char* ssid = "Vansh"; // Replace with your WiFi network SSID
const char* password = "12345678"; // Replace with your WiFi network password
const char* mqtt_server = "broker.hivemq.com"; // MQTT broker address
const int mqtt_port = 1883; // MQTT port

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// I2C LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2); // Address 0x27, 16 columns, 2 rows

// LED pin
const int ledPin = 2; // LED connected to GPIO 2


void setup() {
  Serial.begin(115200);
  setup_wifi();
  // Initialize the I2C LCD
  lcd.init();
  lcd.backlight();
  // Set up the LED pin
  pinMode(ledPin, OUTPUT);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  
}

//run lcd awa led
void lcd_led()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Object detected!!");

  for (int i = 0; i < 10; i++) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }

  lcd.clear();

}


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Process the object detection status
  if (messageTemp == "Object detected") 
  {
    Serial.println("Object detected!");

    //starts lcd and led at same time
    lcd_led();
  } 

  //when object is detected and it is out of frame within 10seconds so, led blinking and lcd message has to stopped
  if (messageTemp == "Object not detected") 
  {
    Serial.println("Object not detected!");

    lcd.clear();
    digitalWrite(ledPin, LOW);
  } 


}

void reconnect() 
{
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("NanoClient")) 
    {
      Serial.println("connected");
      // Subscribe to the time topic
      mqttClient.subscribe("object_detection/topic");
    } 
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println("trying again in 5 seconds");
      delay(5000);
    }
  }
}


 void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
}