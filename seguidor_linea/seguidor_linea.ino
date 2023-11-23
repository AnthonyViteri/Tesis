/*
-----UNIVERSIDAD DE LAS FUERZAS ARMADAS ESPE-----
Tema: Seguidor de Linea para proyecto de Tesis
Autores: Banda Jonathan y Viteri Anthony
*/

//-----Libreria QTR-----
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8; //Definimos el numero de sensores
uint16_t sensorValues[SensorCount]; //Creamos un arreglo

//Driver MotorDC
//Motor A
#define rightMotor1 7 //pin driver1
#define rightMotor2 6 //pin driver2
#define rightMotorPWM 3 //pin pwmA

//Motor B 
#define leftMotor1 9 //pin driver3
#define leftMotor2 8 //pin driver4
#define leftMotorPWM 11 //pin pwmB

#define BaseSpeed 255
#define MaxSpeed 255

//-----Variables para el control PID-----
float Kp,Ki,Kd,Ts,sp,pv;
float e,e_1,e_2;
int u,u_1;

void setup() {

  //Configuramos el sensor
  qtr.setTypeAnalog(); // Definimos los valores como analogicos
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);//Definimos los pines
  qtr.setEmitterPin(2); //Definimos el pin el emisor
  
  //Configuramos los pines de los motores
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT); //Pin predeterminado el Arduino led integrado en placa pin 13
  digitalWrite(LED_BUILTIN, HIGH); // Prendemos el LED para indicar que estamos en modo calibracion
  /* analogRead() tarda aproximadamente 0.1 ms en un AVR.
  0,1 ms por sensor * 4 muestras por sensor leído (por defecto) * 8 sensores
   * 10 lecturas por llamada a calibrate() = ~24 ms por llamada a calibrate().
   Llame a calibrate() 400 veces para que la calibración dure unos 10 segundos.
  */
  for (uint16_t i = 0; i <= 60; i++)
  {
    if (i <= 10){motorLeft();}
    else if (i >= 11 && i<=30){motorRight();}
    else if (i >= 31 && i <= 50){motorLeft();}
    else if (i >= 51 && i <= 60){motorRight();}
    qtr.calibrate();
    delay(0.5);
  }
  motorStop();
  digitalWrite(LED_BUILTIN, LOW); // Apagamos el LED para indicar que salimos de modo calibracion
  //Configuramos el Serial
  Serial.begin(9600);
  // imprimimos los valores mínimos de calibración medidos cuando los emisores estaban encendidos
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  // imprimimos los valores máximos de calibración medidos cuando los emisores estaban encendidos
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  delay(2000);
  sp = 3500;
  Kp=2;
  Ki=0;
  Kd=0;
  Ts = 0.01;
  e = 0;
  e_1 = 0;
  e_2 = 0;
  u_1 = 0;
}

void loop() {
  /* 
    leemos los valores calibrados de los sensores y obtenemos una medida de la posición de la línea
    de 0 a 7000 (para una línea blanca, utilice readLineWhite() en su lugar)
  */ 
  uint16_t position = qtr.readLineBlack(sensorValues);

  e = position - 3500; //Sacamos el error

  //Aplicamos el control con el ecuacion a diferencias
  u = u_1 + (Kp*(e-e_1)) + (Ki*e*Ts) + (Kd*((e-(2*e_1)+e_2)/Ts));
  if (u > 255) {u = 255;}
  else if (u < 0) {u = 0;}
  else{u = u;}

  int rightMotorSpeed = BaseSpeed + u;
  int leftMotorSpeed = BaseSpeed - u;

  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  {
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
  e_1 = e;
  e_2 = e_1;
  u_1 = u;
}


