#include <Servo.h>

Servo miServo;  // Crea un objeto servo para controlar el SG90
const int trigger = 10;
const int echo = 11;
const int fotoresistor = A0;
const int servo = 12;

void setup() 
{
  miServo.attach(servo);  // Asigna el pin al servo
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  miServo.write(0);  // Posición inicial del servo
}

void loop() 
{
  long duracion;
  long distancia;

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  duracion = pulseIn(echo, HIGH);
  distancia = (duracion / 2) / 29.1;

  // Se lee la luz del entorno
  int luz = analogRead(fotoresistor);

  // Si la distancia es mayor  o igual a 80 y la luz es alta
  if (distancia >= 80 && luz > 900) 
  {
    miServo.write(180);
  }
  // Asumiendo "poca luz" como <= 900 y l
  else if (distancia <= 30 && luz <= 900) 
  {
    miServo.write(60);
  }
  // Si la lectura es 0 no hay luz y la distancia es 2
  else if (distancia == 2 && luz == 0) 
  {
    miServo.write(0);
  } 
  // En el caso de que no se cumpla nada se moverá 10 grados
  else 
  {
    miServo.write(10);
  }

  // Retraso para dar tiempo al servo a moverse y estabilizarse
  delay(1000);
}
