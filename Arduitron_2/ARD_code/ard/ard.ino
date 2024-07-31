//PWM Pin for Controlling the speed

int led = 13;
//motor controlling pin
int IN1 = 8;
int IN2 = 9;
int IN3 = 10;
int IN4 = 11;

void setup() {
  Serial.begin(9600); // Initialize serial communication
  //output pin declare

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(led, OUTPUT);

  //Intial State of Car
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming message
    String message = Serial.readStringUntil('\n');
    
    // Print the received message to the Serial Monitor
    Serial.print("Received message: ");
    Serial.println(message);
    digitalWrite(led, HIGH);
    // Add your logic here to process the received message
    message.trim();
    if (message == "forward"){
      Serial.println("executing forward");
      forward();
      delay(100);
      stop();
    }
    if (message == "left"){
      Serial.println("executing left");
      left();
      delay(100);
      stop();
    }
    if (message == "right"){
      Serial.println("executing right");
      right();
      delay(100);
      stop();
    }
    if (message == "stop"){
      Serial.println("executing stop");
      stop();
    }
    if (message == "Sleft"){
      Serial.println("executing Sleft");
      left();
      delay(200);
      stop();
    }
    if (message == "Sright"){
      Serial.println("executing Sright");
      right();
      delay(200);
      stop();
    }
    digitalWrite(led, LOW);
  }
}
//function for control motor
void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  }

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}