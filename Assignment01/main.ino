int salida = A0;
int analogPin = A0;        // Potenciometro
int valor;
double duty;
int sensor;               
int sp;                   // SetPoint
int ang;                  // Angulo leido del potenciometro
double ei;                // Error
double e[20];             // Arreglo para error
double m[20];             // Arreglo para manipulacion
byte u; 


///////////////////////
float kp = 1.321;
float ki = 1.065;         // Cuando el PID T = 0.05seg.
float kd = 0.386;
///////////////////////

void setup()
{
  pinMode(salida, OUTPUT);
  Serial.begin(9600); 
}

void loop()
{
  if(Serial.available()>0)    // Checar si hay datos recibidos. Se mete el Setpoint
  {
    delay(10);
    sp = Serial.parseInt();
    serialFlush();
    if(sp > 110)
    {
      sp = 110;
    }
    if(sp < 0)
    {
      sp = 0;
    }
    Serial.print("Sp = ");
    Serial.print(sp);
  }
  else{}

  // POTENCIOMETRO 
  sensor = analogRead(A0);
    // Serial.println(sensor);
    // Mapear angulo
  ang = map(sensor, 385,869,0,120);
    // Serial.print("Angulo = ");
  Serial.print(ang);

  ei = sp - ang;            // Calculo de error
  Serial.print("Error = ");
  Serial.print(ei);

  // Arreglo de corrimiento para valores anteriores en ecuacion de diferencias

    e[3] = e[2];
    e[2] = e[1];
    e[1] = e[0];
    e[0] = ei;

    m[6] = m[5];
    m[5] = m[4];
    m[4] = m[3];
    m[3] = m[2];
    m[2] = m[1];
    m[1] = m[0];

  
  // Controlador P
  m[0] = kp * e[0];

  // Controlador PI
  m[2] = m[1] + (kp * e[0]) + (ki * e[1]);

  // Controlador PID
  m[4] = m[3] + (kp * e[0]) + (ki * e[1]) + (kd * e[2]);
  
  // m[0] = m[2]+((kp+0.025*ki+40*kd)*e[0])+((0.05*ki-80*kd)*e[1])+((-0.025*ki-kp+40*kd)*e[2]);

  if(m[0] > 100)
  {
    m[0] = 100;
  }
  if(m[0] < 0)
  {
    m[0] = 0;
  }

  u = m[0]*2.55;

  if(u > 255)
  {
    u = 255;
  }
  if(u < 0)
  {
    u = 0;
  }

  analogWrite(salida, u);
  delay(50);

}

void serialFlush()
{
  while(Serial.available() > 0)
  {
   char t = Serial.read(); 
  }
}
