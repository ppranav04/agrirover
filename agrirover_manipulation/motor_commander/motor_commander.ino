#include <Servo.h>

Servo servos[5];                // Array for 5 servos
const int servoPins[4] = {3, 6, 9, 10}; // PWM pins where servos are connected

const byte numServos = 5;
const byte maxLength = 64;
char inputString[maxLength];    // Buffer for incoming serial data
byte inputPos = 0;

void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }
  Serial.println("Ready to receive servo angles");
}

void loop() { 
  while (Serial.available() > 0) {
    char inChar = Serial.read();

    if (inChar == '\n') {  // End of command
      inputString[inputPos] = '\0';  // Null-terminate string
      processCommand(inputString);
      inputPos = 0;  // Reset buffer position
    } 
    else if (inputPos < maxLength - 1) {
      inputString[inputPos++] = inChar;
    }
  }
}

void processCommand(char* command) {
  int values[numServos];
  byte index = 0; 

  char* token = strtok(command, ",");
  while (token != NULL && index < numServos) {
    values[index++] = atoi(token);  // Convert string to int
    token = strtok(NULL, ",");
  }

  if (index == numServos) {
    // Move servos and print complementary angles
    Serial.print("Servo angles: ");
    for (byte i = 0; i < numServos; i++) {
      int angle = constrain(values[i], 0, 180);
      servos[i].write(angle);
      Serial.print(angle);
      if (i < numServos - 1) Serial.print(", ");
    }
    Serial.println();
  } else {
    Serial.println("Error: Invalid number of angles received");
  }
}
