#include <Servo.h> 
#include <NewPing.h>
#include "sounds.c"


//Ultrasonidos
#define TRIGGER_PIN  13   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13   // Arduino pin tied to echo pin on the ultrasonic sensor.
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
#define PIN_RGB_R A2
#define PIN_RGB_G A1
#define PIN_RGB_B A0

//Potenciometro
#define PIN_POT A3

//Tiempos
#define LOOP_TIME 30
#define SERVO_TIME 150

unsigned long loopTimer = 0, buzzerTimer=0, buzzerTime = 0,  noteDurationTimer=0, noteDurationTime=0, servoTimer=0;
boolean buzzerOn;

//Motores
#define ENA 5             // Potencia motor A
#define IN1 11            // Control 1 motor A
#define IN2 10            // Control 2 motor A
#define ENB 6             // Potencia motor B
#define IN3 9             // Control 1 motor B
#define IN4 8             // Control 2 motor A
 
void setup() 
{ 
  Serial.begin(115200);
  
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
  
  //Potenciometro
  pinMode(PIN_POT, OUTPUT);  
  
  initialMelody(PIN_BUZZER);
} 
 
 
void loop() 
{   
  loopTimer = millis();
  
  //Serial.println(analogRead(PIN_POT));
  
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
    motorForward(180); 
  }
   
   while ((millis()-loopTimer) < LOOP_TIME){delay(1);} //Solo ejecuta bucle principal una vez cada LOOP_TIME
} 

void scan(){
  unsigned int uS = sonar.ping();
  distance = sonar.convert_cm(uS);
  if (distance==0){distance=MAX_DISTANCE;}
  //Serial.print("Ping: ");
  //Serial.print(distance);
  //Serial.println("cm");
}

void motorForward(int speed){
   analogWrite(ENA, speed);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, HIGH);   
   analogWrite(ENB, speed-15);   
   digitalWrite(IN3, LOW);   
   digitalWrite(IN4, HIGH);  
}

void motorLeft(int speed){
   analogWrite(ENA, speed);
   digitalWrite(IN1, LOW);   
   digitalWrite(IN2, HIGH);   
   analogWrite(ENB, speed-15);   
   digitalWrite(IN3, HIGH);   
   digitalWrite(IN4, LOW);  
}

void motorRight(int speed){
   analogWrite(ENA, speed);
   digitalWrite(IN1, HIGH);   
   digitalWrite(IN2, LOW);   
   analogWrite(ENB, speed-15);   
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

