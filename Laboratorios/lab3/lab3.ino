#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 400

#define MOTOR_RIGHT_PWM 5
#define MOTOR_RIGHT_DIR 6
#define MOTOR_LEFT_PWM  9
#define MOTOR_LEFT_DIR  10

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int distance = 0;

double setpoint = 30.0; // Setpoint en centímetros

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
}

void loop() {
  distance = sonar.ping_cm();
  
  // Envía la distancia medida a través del puerto serial
  Serial.print("Distance: ");
  Serial.println(distance);
  
  // Recibe el valor del error y el PID desde Python
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    double error = input.substring(0, commaIndex).toDouble();
    double output = input.substring(commaIndex + 1).toDouble();

    // Si el error está dentro del 20% del setpoint, detén el robot
    if (abs(error) <= setpoint * 0.2) {
      analogWrite(MOTOR_RIGHT_PWM, 0);
      analogWrite(MOTOR_LEFT_PWM, 0);
    } else {
      // Ajuste de umbrales basado en valores típicos del PID
      int motorSpeed;
      if (abs(output) > 20) { // Valor alto de PID, robot muy lejos
        motorSpeed = 255; // Velocidad máxima
      } else if (abs(output) < 15) { // Valor mediano de PID
        motorSpeed = 128; // Velocidad media
      } else if (abs(output) <= 10) { // Valor bajo de PID
        motorSpeed = 40; // Velocidad baja
      }

      // Ajusta la dirección basada en la salida del PID.
      if (output < 0) {
        digitalWrite(MOTOR_RIGHT_DIR, HIGH);
        digitalWrite(MOTOR_LEFT_DIR, HIGH);
      } else {
        digitalWrite(MOTOR_RIGHT_DIR, LOW);
        digitalWrite(MOTOR_LEFT_DIR, LOW);
      }

      analogWrite(MOTOR_RIGHT_PWM, motorSpeed);
      analogWrite(MOTOR_LEFT_PWM, motorSpeed);
    }
  }

  delay(50);
}
