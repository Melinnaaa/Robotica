#include <Servo.h>

Servo miServo;  // Crea un objeto servo para controlar el SG90
const int trigger = 10;
const int echo = 11;
const int fotoresistor = A0;
const int servo = 9;

void setup() 
{
  miServo.attach(servo);  // Asigna el pin al servo
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  miServo.write(0);  // Posición inicial del servo
  Serial.begin(9600);
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
  Serial.println(luz);
  Serial.println(distancia);
  

  // Si la distancia es mayor  o igual a 80 y la luz es alta
  if (distancia >= 80 && luz > 250) 
  {
    miServo.write(180);
  }
  // Asumiendo "poca luz" como <= 300 y l
  if (distancia <= 30 &&  luz <= 250) 
  {
    miServo.write(60);
  }
  // Si la lectura es 0 no hay luz y la distancia es 2
  if (distancia <= 2 && luz < 50) 
  {
    miServo.write(0);
  }

  // Retraso para dar tiempo al servo a moverse y estabilizarse
  delay(100);
}
