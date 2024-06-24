#include <Wire.h>
#include <PID_v2.h>
#include <MPU6050.h>

MPU6050 mpu;

// Definición de pines para el servo (ajusta según tu configuración)
const int servoPin = 9;

// Parámetros del PID (ajusta según sea necesario)
double Setpoint = 0;  // Ángulo deseado
double Input, Output;

// Valores de los parámetros PID iniciales (ajusta según sea necesario)
double Kp = 1.0;
double Ki = 0.0;
double Kd = 0.0;

// Crea el objeto PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  myPID.SetMode(AUTOMATIC); // Configura el PID en modo automático
  myPID.SetOutputLimits(-255, 255); // Límites de salida del PID

  // Configura el pin del servo
  pinMode(servoPin, OUTPUT);
}

void loop() {
  // Lee los valores del sensor MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calcula el ángulo utilizando los valores del giroscopio
  double gyroAngle = gx / 131.0; // Factor de escala del giroscopio

  // Filtra el ruido del giroscopio con un filtro complementario
  double alpha = 0.98; // Factor de filtrado
  Input = alpha * (Input + gyroAngle) + (1 - alpha) * ax;

  // Calcula la salida del PID
  myPID.Compute();

  // Aplica la salida al servo
  int pwmValue = map(Output, -255, 255, 0, 180); // Mapea la salida del PID al rango del servo
  analogWrite(servoPin, pwmValue);

  // Lee los comandos desde el puerto serie para ajustar dinámicamente los parámetros PID
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'P') {
      Kp += 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'p') {
      Kp -= 0.1;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'I') {
      Ki += 0.01;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'i') {
      Ki -= 0.01;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'D') {
      Kd += 1.0;
      myPID.SetTunings(Kp, Ki, Kd);
    } else if (command == 'd') {
      Kd -= 1.0;
      myPID.SetTunings(Kp, Ki, Kd);
    }
  }
}
