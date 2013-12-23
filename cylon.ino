#include <Servo.h> 
#include <NewPing.h>
#include "sounds.c"
#include <SoftwareSerial.h>

//Bluetooth
SoftwareSerial blue(0,1);
char rec;

//Ultrasonidos
#define TRIGGER_PIN  2   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50   // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define UMBRAL 40         // Distancia que considera cercano (en cm)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

//Servo head
#define PIN_SERVO 3       // Pin de cotrol del servo
#define SERVO_CENTER 80   // Posicion del servo en centro
#define SERVO_LEFT 120    // Posicion del servo a la izquierda
#define SERVO_RIGHT 40    // Posicion del servo a la derecha
Servo servoHead;          // Objeto servo
int pos;                  // Posición del servo
unsigned int distance;    // Distancia minima actual a obstáculo
boolean goingRight;       // Indicador de hacia que lado est el servo con respecto al centro

//Buzzer
#define PIN_BUZZER 7

//RGB
#define PIN_RGB_R 4
#define PIN_RGB_G A1
#define PIN_RGB_B A0

//Acelerometro
#define PIN_X A3
//#define PIN_Y A4
//#define PIN_Z A5

//Potenciometro
#define PIN_POT A2

//Tiempos
#define LOOP_TIME 30
#define SERVO_TIME 150
#define BT_TIME 150

unsigned long loopTimer = 0, buzzerTimer=0, buzzerTime = 0,  noteDurationTimer=0, noteDurationTime=0, servoTimer=0, btTimer=0;
boolean buzzerOn;

//Siguelineas
#define SL_RIGHT A4
#define SL_LEFT A5
int desc = 10;

//Motores
#define ENA 5             // Potencia motor A
#define IN1 11            // Control 1 motor A
#define IN2 10            // Control 2 motor A
#define ENB 6             // Potencia motor B
#define IN3 9             // Control 1 motor B
#define IN4 8             // Control 2 motor A
 
int speed = 220;

//Modes
#define MODE_STOP 0
#define MODE_DISCOVERY 1
#define MODE_SIGUELINEAS 2
int mode = MODE_STOP;


void setup(){
  blue.begin(9600);
  
  //Ultrasonidos
  distance = 0;
  
  //RGB
  pinMode(PIN_RGB_R, OUTPUT);  
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  turnRGB(1,0,0);
  
  //Servo head
  servoHead.attach(PIN_SERVO);
  pos=SERVO_CENTER;
  goingRight=true;
  servoHead.write(pos);
  
  //Motores
  pinMode(ENA, OUTPUT);     
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);     
  pinMode(ENB, OUTPUT);     
  pinMode(IN3, OUTPUT);     
  pinMode(IN4, OUTPUT);
  
  speed = analogRead(PIN_POT)/4;
  
  //Potenciometro
  pinMode(PIN_POT, INPUT);

  //Acelerometro
  pinMode(PIN_X, INPUT); 
  //pinMode(PIN_Y, INPUT); 
  //pinMode(PIN_Z, INPUT); 
 
  pinMode(SL_RIGHT, INPUT);     
  pinMode(SL_LEFT, INPUT); 
  
  initialMelody(PIN_BUZZER);
} 

void loop() 
{ 
  if(blue.available()){
    rec=blue.read();
    switch(rec){
      case '0':
        modoStop();
        mode = MODE_STOP;
        break;
      case '1':
        modoStop();
        mode = MODE_DISCOVERY;
        break;
      case '2':
        mode = MODE_SIGUELINEAS;
        break; 
      default:
        blue.print(rec);
        blue.println(" no es una orden valida. Introduzca 0, 1 o 2");
    }
  }
  speed = analogRead(PIN_POT)/4;
  switch(mode){
      case MODE_DISCOVERY:
        modoDiscovery();
        break;
      case MODE_SIGUELINEAS:
        modoSiguelineas();
        break;
      case MODE_STOP:
        modoStop();
        break;
  }
}

void modoStop(){
  motorStop();
  turnRGB(1,0,0);
  pos=SERVO_CENTER;
  servoHead.write(pos);
}
 
void modoDiscovery() 
{   
  loopTimer = millis();
  
  if ((millis()-btTimer) > BT_TIME){
     btTimer=millis();
     //sendBluetooth();
  }
  
  if ((millis()-servoTimer) > SERVO_TIME){
    servoTimer=millis();
    moverServo();
  }
   
  scan();
  
  if ((millis()-noteDurationTimer) > noteDurationTime && buzzerOn){
    noTone(PIN_BUZZER);
    buzzerOn=false;
  }
  if ((millis()-buzzerTimer) > buzzerTime){
    buzzerTimer=millis();
    nearSound(PIN_BUZZER,distance);
  }
  if (distance < UMBRAL){
     turnRGB(0,0,1);
     if (pos == SERVO_RIGHT){
       motorLeft(170);
     }
     else if (pos == SERVO_LEFT){
       motorRight(170);
     }
     else{
       if (random(1)){
         motorRight(220);
       }
       else{
         motorLeft(220);
       } 
     }
     delay(100); 
  }
  else {
    turnRGB(0,1,0); 
    motorForward(speed); 
  }
   
  while ((millis()-loopTimer) < LOOP_TIME){delay(1);} //Solo ejecuta bucle principal una vez cada LOOP_TIME  
}

void modoSiguelineas(){
  int right = analogRead(SL_RIGHT);
  int left = analogRead(SL_LEFT);
  if (right<190){
     motorRight(speed);
     turnRGB(0,1,1);
  }
  else if (left<80){
    motorLeft(speed);
    turnRGB(0,1,1);
  }
  else{
   motorForward(speed);
   turnRGB(1,0,1);  
  }
}

void scan(){
  unsigned int uS = sonar.ping();
  distance = sonar.convert_cm(uS);
  if (distance==0){distance=MAX_DISTANCE;}
}

void motorForward(int speed){
   analogWrite(ENA, speed+desc);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, HIGH);   
   analogWrite(ENB, speed);   
   digitalWrite(IN3, LOW);   
   digitalWrite(IN4, HIGH);  
}

void motorLeft(int speed){
   analogWrite(ENA, speed+desc);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, HIGH);   
   analogWrite(ENB, speed);   
   digitalWrite(IN3, HIGH);   
   digitalWrite(IN4, LOW);  
}

void motorRight(int speed){
   analogWrite(ENA, speed+desc);
   digitalWrite(IN1, HIGH);   
   digitalWrite(IN2, LOW);   
   analogWrite(ENB, speed);   
   digitalWrite(IN3, LOW);   
   digitalWrite(IN4, HIGH);  
}

void motorStop(){
   analogWrite(ENA, 0);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, LOW);   
   analogWrite(ENB, 0);   
   digitalWrite(IN3, LOW);   
   digitalWrite(IN4, LOW);  
}

void nearSound(int pin, int distance){
    tone(pin, NOTE_C4,1000/8);
    noteDurationTime=1000/8;
    buzzerTime = distance>40?1600:distance*distance;
    noteDurationTimer=millis();
    buzzerOn=true;
}

void turnRGB(int R, int G, int B){
  digitalWrite(PIN_RGB_R, R);
  digitalWrite(PIN_RGB_G, G);
  digitalWrite(PIN_RGB_B, B);
}

void moverServo(){
  if (pos==SERVO_LEFT){
    goingRight=true;
    pos=SERVO_CENTER;
  }
  else if(pos==SERVO_RIGHT){
    goingRight=false;
    pos=SERVO_CENTER;
  }
  else if (goingRight){
    pos=SERVO_RIGHT;
  }
  else{
    pos=SERVO_LEFT;
  }
  servoHead.write(pos);
}

void sendBluetooth(){
  blue.print("Speed=");
  blue.print(speed);
  blue.print(" X=");
  blue.print(analogRead(PIN_X));
//  blue.print(" Y=");
//  blue.print(analogRead(PIN_Y));
//  blue.print(" Z=");
//  blue.println(analogRead(PIN_Z));
}

