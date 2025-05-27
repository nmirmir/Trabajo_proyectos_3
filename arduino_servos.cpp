#include <Servo.h>

// Declaración de los objetos Servo
Servo servo1;
Servo servo2;

// Pines donde están conectados los servos
const int pinServo1 = 9;
const int pinServo2 = 10;

void setup() {
  // Inicia los servos en los pines correspondientes
  servo1.attach(pinServo1);
  servo2.attach(pinServo2);
  
  // Inicia la comunicación serie
  Serial.begin(9600);
  
  // Inicializa los servos en la posición "Cerrado"
  servo1.write(0);    // Posición "Cerrado" de servo1
  servo2.write(0);    // Posición "Cerrado" de servo2
}

void loop() {
  // Verifica si hay datos disponibles en el puerto serie
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Elimina espacios o caracteres invisibles
    
    // Verifica el comando recibido
    if (command == "Abierto") {
      // Si el comando es "Abierto", mueve los servos a una posición abierta
      servo1.write(90);  // Posición "Abierto" de servo1
      servo2.write(90);  // Posición "Abierto" de servo2
      Serial.println("Servos: Abierto");
    } 
    else if (command == "Cerrado") {
      // Si el comando es "Cerrado", mueve los servos a la posición cerrada
      servo1.write(0);   // Posición "Cerrado" de servo1
      servo2.write(0);   // Posición "Cerrado" de servo2
      Serial.println("Servos: Cerrado");
    }
    else {
      // Si el comando no es reconocido
      Serial.println("Comando no reconocido. Usa 'Abierto' o 'Cerrado'.");
    }
    
    // Pequeño retraso para permitir que el buffer se vacíe
    delay(50);
  }
}
