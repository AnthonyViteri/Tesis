/*
-----UNIVERSIDAD DE LAS FUERZAS ARMADAS ESPE-----
Tema: Seguidor de Linea para proyecto de Tesis
Autores: Banda Jonathan y Viteri Anthony
*/

//-----Libreria QTR-----
#include <QTRSensors.h>

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

//-----Variables para el control PID-----
float Kp,,Ki,Kd,Ts,sv,pv;
float e,e_1,e_2;
int u,u_1;

void setup() {
  //set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  TCCR2B = TCCR2B & B11111000 | B00000111;   
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
  #define BaseSpeed 150
  #define MaxSpedd 255
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT); //Pin predeterminado el Arduino led integrado en placa pin 13
  digitalWrite(LED_BUILTIN, HIGH); // Prendemos el LED para indicar que estamos en modo calibracion
  /* analogRead() tarda aproximadamente 0.1 ms en un AVR.
  0,1 ms por sensor * 4 muestras por sensor leído (por defecto) * 8 sensores
   * 10 lecturas por llamada a calibrate() = ~24 ms por llamada a calibrate().
   Llame a calibrate() 400 veces para que la calibración dure unos 10 segundos.
  */
  for (uint16_t i = 0; i < 200; i++) //Calibracion durante 4.8s
  {
    if (i <= 10){motorLeft();}
    else if (i >= 11 && i<=30){motorRight();}
    else if (i >= 31 && i <= 50){motorLeft();}
    else if (i >= 51 && i <= 70){motorRight();}
    else if (i >= 71 && i<=90){motorRight();}
    else if (i >= 91 && i <= 110){motorLeft();}
    else if (i >= 111 && i <= 130){motorRight();}
    else if (i >= 131 && i<=150){motorRight();}
    else if (i >= 151 && i <= 170){motorLeft();}
    else if (i >= 171 && i <= 190){motorRight();}
    else if (i >= 191 && i<=200){motorRight();}
    qtr.calibrate();
    delay(1);
  }
  motorStop();
  digitalWrite(LED_BUILTIN, LOW); // Apagamos el LED para indicar que salimos de modo calibracion
  //Configuramos el Serial
  Serial.begin(115200);
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
  sv = 3500;
  Kp=5;
  Ki=2;
  Kd=0.0001;
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

  //Aplicamos el control con el ecuacion a diferencias
  u = u_1 + (Kp*(e-e_1)) + (Ki*e*Ts) + (Kd*((e-(2*e_1)+e_2)/Ts));
  int rightMotorSpeed = BaseSpeed + ut;
  int leftMotorSpeed = BaseSpeed - ut;

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
  e_1 = et;
  e_2 = et_1;
  u_1 = ut;
} 



// Funciones para movimiento de los motores para la calibracion
void motorStop(){
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2,LOW);
  analogWrite(rightMotorPWM,0);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2,LOW);
  analogWrite(leftMotorPWM,0);
}
void motorRight(){
  digitalWrite(rightMotor1,HIGH);
  digitalWrite(rightMotor2,LOW);
  analogWrite(rightMotorPWM,150);
  digitalWrite(leftMotor1,LOW);
  digitalWrite(leftMotor2,HIGH);
  analogWrite(leftMotorPWM,150); 
}
void motorLeft(){
  digitalWrite(rightMotor1,LOW);
  digitalWrite(rightMotor2,HIGH);
  analogWrite(rightMotorPWM,70);
  digitalWrite(leftMotor1,HIGH);
  digitalWrite(leftMotor2,LOW);
  analogWrite(leftMotorPWM,150); 
}