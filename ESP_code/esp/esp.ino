#include <WiFi.h>

// WiFi credentials
const char* ssid = "Shashwat’s iPhone";     // Enter your WiFi hotspot SSID
const char* password = "pink@123";      // Enter your WiFi hotspot password
const uint16_t port = 8002;
const char* host = "172.20.10.4";          // Enter the IP address of your laptop after connecting it to the WiFi hotspot

// External peripherals 
int buzzerPin = 15;
int redLed = 2;

char incomingPacket[80];
WiFiClient client;

String msg = "0";
int counter = 0;

void setup() {
  Serial.begin(9600);                    // Serial to print data on Serial Monitor

  // Output Pins
  pinMode(buzzerPin, OUTPUT);                      
  pinMode(redLed, OUTPUT);
  // Initially off
  digitalWrite(buzzerPin, HIGH);           // Negative logic Buzzer       
  digitalWrite(redLed, LOW);

  // Connecting to WiFi
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
 
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (!client.connect(host, port)) {
    Serial.println("Connection to host failed");
    digitalWrite(buzzerPin, HIGH);         
    digitalWrite(redLed, LOW); 
    delay(200);
    return;
  }

  while (1) {
    msg = client.readStringUntil('\n');   // Read the message through the socket until a newline char('\n')
    client.print("Hello from ESP32!");    // Send an acknowledgement to the host (laptop)
    counter = msg.toInt();
    Serial.println(counter);              // Print data on Serial monitor
    msg.trim();
//    Serial.println(msg);  // Assuming ESP32 is connected to Arduino Uno via Serial1
    digitalWrite(redLed, HIGH);
    if (msg == "forward"){
      Serial.println("forward");
    }
    if (msg == "right"){
      Serial.println("right");
    }
    if (msg == "left"){
      Serial.println("left");
    }
    if (msg == "stop"){
      Serial.println("stop");
    }
    if (msg == "Sleft"){
      Serial.println("Sleft");
    }
    if (msg == "Sright"){
      Serial.println("Sright");
    }
    
    digitalWrite(redLed, LOW);
  }
}
