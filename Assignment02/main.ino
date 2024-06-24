#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

const int motorPWM = 9;      // Pin PWM para el motor DC (control de velocidad)
const int motorDir1 = 8;     // Pin para controlar la dirección del motor
const int motorDir2 = 7;     // Pin para controlar la dirección del motor
const float desiredAngle = 0.0;  // Ángulo deseado del brazo

// Parámetros del PID
float kp, ki, kd;
float q0, q1, q2;
volatile float e = 0.0, e_1 = 0.0, e_2 = 0.0;
float k = 1.04, tau = 160, theta = 10 + 8 / 2; // Parámetros del Modelo del sistema

void loop() {
  PID();
  // Puedes agregar otras acciones aquí según sea necesario
}

void PID(void) {
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  float currentAngle = calcularAnguloConIMU(gx, gy, gz);

  e = (desiredAngle - currentAngle);

  // Control PID
  float u = kp * e + ki * e_1 + kd * e_2; // Ley del controlador PID discreto

  if (u >= 255.0) // Saturar la acción de control en un tope máximo y mínimo
    u = 255.0;

  if (u <= 0.0)
    u = 0.0;

  // Retorno a los valores reales
  e_2 = e_1;
  e_1 = e;

  // La acción calculada se transforma en PWM para controlar el motor DC
  ajustarMotorDC(u);

  // Visualización en la consola serial
  Serial.print("Ángulo IMU: ");
  Serial.print(currentAngle);
  Serial.print(" | Error: ");
  Serial.print(e);
  Serial.print(" | Salida PID: ");
  Serial.println(u);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);

  // Inicializar el control PID
  float factorSintonia = 0.6;
  kp = 0.6 * factorSintonia; 
  ki = 1.2 * factorSintonia;
  kd = 0.3 * factorSintonia;
  q0 = kp * (1 + theta / 2) + ki * (theta / 2) + kd * (theta / 2); 
  q1 = -(kp * (1 - theta / 2) + ki * (theta / 2) * 2 + kd * theta / 2); 
  q2 = kd * theta / 2;

  Serial.println("Sistema inicializado");
}


void ajustarMotorDC(float pwmValue) {
  if (pwmValue > 0) {
    digitalWrite(motorDir1, HIGH);
    digitalWrite(motorDir2, LOW);
  } else {
    digitalWrite(motorDir1, LOW);
    digitalWrite(motorDir2, HIGH);
  }

  float mult = pwmValue * 25 ;
  analogWrite(motorPWM, abs(mult));
}

/*float calcularAnguloConIMU(int16_t gx, int16_t gy, int16_t gz) {
  float roll = atan2(-gy, gz) * 180.0 / M_PI;
  return roll;
}*/

double calcularAnguloConIMU(int16_t gx, int16_t gy, int16_t gz) {
  mpu.getAcceleration(&gx, &gy, &gz);
  double roll = atan2(-gy, gz) * 180.0 / M_PI;
  return roll;
}
