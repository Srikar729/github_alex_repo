const int relay1Pin = 26;
const int relay2Pin = 27;

void setup() {
  Serial.begin(115200);
  Serial.println("Relay Control");

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);

  digitalWrite(relay1Pin, HIGH);
  digitalWrite(relay2Pin, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "r11") {
      digitalWrite(relay1Pin, LOW);
      Serial.println("ACK: Relay 1 ON");
    } else if (command == "r10") {
      digitalWrite(relay1Pin, HIGH);
      Serial.println("ACK: Relay 1 OFF");
    } else if (command == "r21") {
      digitalWrite(relay2Pin, LOW);
      Serial.println("ACK: Relay 2 ON");
    } else if (command == "r20") {
      digitalWrite(relay2Pin, HIGH);
      Serial.println("ACK: Relay 2 OFF");
    } else {
      Serial.println("Invalid command: "+command);
    }
  }
}
