#include <NewPing.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 200

#define MOTOR_RIGHT_PWM 5
#define MOTOR_RIGHT_DIR 6
#define MOTOR_LEFT_PWM  9
#define MOTOR_LEFT_DIR  10

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int distance = 0;

double setpoint = 80.0; // Setpoint en centímetros
double kp = 0.5, ki = 0.05, kd = 0.1;
double integral = 0, lastError = 0;
unsigned long lastTime = 0;

double PID(double error, double timeChange);

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
}

void loop() {
  unsigned long currentTime = millis();
  double timeChange = (double)(currentTime - lastTime) / 1000.0;

  if (timeChange >= 0.05) { // Actualizado a 50 ms
    double error, output;
    distance = sonar.ping_cm();
    if (distance == 0 || distance > MAX_DISTANCE) {
      distance = MAX_DISTANCE; // Manejo de error del sensor
    }

    // Lectura del setpoint desde el puerto serial
    if (Serial.available() > 0) {
      setpoint = Serial.parseFloat();
      // Limpia el buffer del puerto serial
      while (Serial.available() > 0) {
        Serial.read();
      }
    }

    error = setpoint - distance;
    
    // Si la distancia está dentro del 15% del setpoint, detener el robot completamente
    if (abs(error) <= setpoint * 0.15) {
      analogWrite(MOTOR_RIGHT_PWM, 0);
      analogWrite(MOTOR_LEFT_PWM, 0);
    } else {
      output = PID
    (error, timeChange);

      // Ajuste de umbrales basado en valores típicos del PID
      int motorSpeed;
      if (abs(output) > 30) { // Valor alto de PID, robot muy lejos
        motorSpeed = 255; // Velocidad máxima
      } else if (abs(output) < 20) { // Valor mediano de PID
        motorSpeed = 128; // Velocidad media
      } else if (abs(output) <= 10){ // Valor bajo de PID
        motorSpeed = 40; // Velocidad baja
      }

      // Ajusta la dirección basada en la salida del PID.
      if (output > 0) {
        digitalWrite(MOTOR_RIGHT_DIR, HIGH);
        digitalWrite(MOTOR_LEFT_DIR, HIGH);
      } else {
        digitalWrite(MOTOR_RIGHT_DIR, LOW);
        digitalWrite(MOTOR_LEFT_DIR, LOW);
      }

      analogWrite(MOTOR_RIGHT_PWM, motorSpeed);
      analogWrite(MOTOR_LEFT_PWM, motorSpeed);
    }

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" cm, Output: ");
    Serial.println(output);

    lastTime = currentTime;
  }
}

double PID(double error, double timeChange) {
  double derivative, output;
  
  // Variables locales para los parámetros del PID
  double localKp = kp, localKi = ki, localKd = kd;

  // Cálculo del término integral
  integral += error * timeChange;
  integral = constrain(integral, -50, 50); // Limita el término integral para evitar oscilaciones excesivas

  // Cálculo del término derivativo
  derivative = (error - lastError) / timeChange;

  // Cálculo de la salida del controlador PID
  output = localKp * error + localKi * integral + localKd * derivative;

  // Actualización del error anterior para la próxima iteración
  lastError = error;

  return output;
}
