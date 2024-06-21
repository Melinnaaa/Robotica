#include <NewPing.h>
#include <ArduinoJson.h>

#define TRIGGER_PIN  12
#define ECHO_PIN     11
#define MAX_DISTANCE 400

#define MOTOR_RIGHT_IN1 5  // IN1 (PWM)
#define MOTOR_RIGHT_IN2 6  // IN2 (PWM)
#define MOTOR_LEFT_IN1  9  // IN3 (PWM)
#define MOTOR_LEFT_IN2  10 // IN4 (PWM)

#define IR_PIN 2  // Conectado al pin digital 2

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int distance = 0;

unsigned long lastTime = 0;
unsigned long lastCommandTime = 0;
bool isRemoteControl = false; // Variable para modo control remoto
const unsigned long remoteControlTimeout = 3000; // 3 segundos de timeout
bool turning = false; // Variable para saber si está girando
bool turnDirection; // true = left, false = right

// Variables para el envío de datos cada 1 segundo
unsigned long lastSensorSendTime = 0;
const unsigned long sensorSendInterval = 1000; // 1 segundo

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(IR_PIN, INPUT);
  randomSeed(analogRead(0)); // Inicializar el generador de números aleatorios
}

void loop() {
  unsigned long currentTime = millis();
  int irValue = digitalRead(IR_PIN); // Declarar irValue aquí

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Si estamos en modo detectar mascotas
    if (command.startsWith("pet:")) {
      handlePetCommand(command);
    }
    // Señal que envía para mantener el control remoto activo y no realizar ningún movimiento.
    else if (command == "KEEP_ALIVE") {
      lastCommandTime = currentTime;
    }
    // Si se está controlando de forma remota el robot 
    else if (command == "FORWARD" || command == "BACKWARD" || command == "LEFT" || command == "RIGHT" ||
            command == "FORWARD_RIGHT" || command == "FORWARD_LEFT" || command == "BACKWARD_RIGHT" || command == "BACKWARD_LEFT" || command == "STOP") {
      isRemoteControl = true;
      handleRemoteControl(command);
      lastCommandTime = currentTime;
    }
    // Para buscar la mascota de no ser detectada.
    else if (command.startsWith("move:")) {
      handleMoveCommand(command.substring(5));
      lastCommandTime = currentTime;
    }
    // Si no se recibe nada el robot se mueve de forma arbitraria evitando obstaculos.
    else {
      isRemoteControl = false;
    }

    // Se lee el puerto serial
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
  // Si se deja de realizar el control remoto.
  if ((currentTime - lastCommandTime) > remoteControlTimeout) {
    isRemoteControl = false;
  }

  // En caso de no estar haciendo nada desde la aplicación
  if (!isRemoteControl && (currentTime - lastTime) >= 200) {
    lastTime = currentTime;
    distance = sonar.ping_cm();
    if (distance == 0 || distance > MAX_DISTANCE) {
      distance = MAX_DISTANCE;
    }

    if (irValue == LOW || distance < 10) { // Obstáculo detectado por IR o distancia menor a 10 cm
      if (!turning) {
        turnDirection = random(0, 2); // Elegir aleatoriamente entre 0 (derecha) y 1 (izquierda)
        turning = true;
      }
      if (turnDirection) {
        turnLeft(75); // Gira a la izquierda con velocidad ligeramente reducida
      } else {
        turnRight(75); // Gira a la derecha con velocidad ligeramente reducida
      }
      delay(1000); // Espera un poco antes de verificar de nuevo
    } else {
      turning = false;
      int motorSpeed = map(distance, 10, MAX_DISTANCE, 45, 100); // Ajusta la velocidad en función de la distancia
      moveForward(motorSpeed);
      delay(500); // Delay para el movimiento
    }
  }

  // Enviar datos de sensores cada 1 segundo
  if ((currentTime - lastSensorSendTime) >= sensorSendInterval) {
    lastSensorSendTime = currentTime;

    // Enviar datos de sensores a la Raspberry Pi
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["distance"] = distance;
    jsonDoc["irValue"] = irValue;
    jsonDoc["isRemoteControl"] = isRemoteControl;

    char outputJson[200];
    serializeJson(jsonDoc, outputJson);
    Serial.println(outputJson);
  }
}

// Comandos para el control remoto mediante la aplicación
void handleRemoteControl(String command) {
  int speed = 75;  
  if (command == "FORWARD") {
    moveForward(speed);
  } else if (command == "BACKWARD") {
    moveBackward(speed);
  } else if (command == "LEFT") {
    turnLeft(speed);
  } else if (command == "RIGHT") {
    turnRight(speed);
  } else if (command == "FORWARD_RIGHT") {
    moveForwardRight(speed);
  } else if (command == "FORWARD_LEFT") {
    moveForwardLeft(speed);
  } else if (command == "BACKWARD_RIGHT") {
    moveBackwardRight(speed);
  } else if (command == "BACKWARD_LEFT") {
    moveBackwardLeft(speed);
  } else if (command == "STOP") {
    stopMotors();
  }
}

// Sigue los comandos que manda la raspberry respecto a la posición de la mascota
void handlePetCommand(String command) {
  int speed = 75;  
  String position = command.substring(4);
  if (position == "forward") {
    moveForward(speed);
  } else if (position == "left") {
    turnLeft(speed);
  } else if (position == "right") {
    turnRight(speed);
  }
}


void handleMoveCommand(String direction) {
  int speed = 75;  
  if (direction == "forward") {
    moveForward(speed);
  } else if (direction == "left") {
    turnLeft(speed);
  } else if (direction == "right") {
    turnRight(speed);
  }
}

// Funciones de movimiento del robot

void moveForward(int speed) {
  analogWrite(MOTOR_RIGHT_IN1, speed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  analogWrite(MOTOR_LEFT_IN1, speed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, speed);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, speed);
}

void turnLeft(int speed) {
  analogWrite(MOTOR_RIGHT_IN1, speed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, speed);
  analogWrite(MOTOR_LEFT_IN1, speed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}

void moveForwardRight(int speed) {
  analogWrite(MOTOR_RIGHT_IN1, speed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, speed * 0.5);
}

void moveForwardLeft(int speed) {
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, speed * 0.5);
  analogWrite(MOTOR_LEFT_IN1, speed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}

void moveBackwardRight(int speed) {
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  analogWrite(MOTOR_RIGHT_IN2, speed * 0.5);
  analogWrite(MOTOR_LEFT_IN1, speed);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}

void moveBackwardLeft(int speed) {
  analogWrite(MOTOR_RIGHT_IN1, speed);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  analogWrite(MOTOR_LEFT_IN2, speed * 0.5);
}

void stopMotors() {
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
}
