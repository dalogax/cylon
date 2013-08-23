#include <Servo.h> 
#include <NewPing.h>
#include "sounds.c"


//Ultrasonidos
#define TRIGGER_PIN  13   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 50  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define UMBRAL 30          // Distancia que considera cercano (en cm)

//Servo head
#define PIN_SERVO 3       // Pin de cotrol del servo
Servo servoHead;          // Objeto servo
int pos;             // Posición del servo
unsigned int distance;    // Distancia minima actual a obstáculo

//Buzzer
#define PIN_BUZZER 7

//Tiempos
#define LOOP_TIME 30

unsigned long loopTimer = 0, buzzerTimer=0, buzzerTime = 0,  noteDurationTimer=0, noteDurationTime=0;
boolean buzzerOn;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 
Servo servoHead;
 
int pos = 30; // variable to store the servo position 
unsigned int distance = 0;

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
  
  //Servo head
  servoHead.attach(PIN_SERVO);
  servoHead.write(80);
  pos=30;
  
  //Motores
  pinMode(ENA, OUTPUT);     
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);     
  pinMode(ENB, OUTPUT);     
  pinMode(IN3, OUTPUT);     
  pinMode(IN4, OUTPUT);
  
  initialMelody(PIN_BUZZER);
  
  /*pinMode(2, OUTPUT);  
  digitalWrite(5, HIGHT);
  pinMode(1, OUTPUT);  
  digitalWrite(4, HIGHT);
  pinMode(0, OUTPUT);  
  digitalWrite(3, HIGHT);*/
} 
 
 
void loop() 
{   
  loopTimer = millis();
   
  motorForward(200);
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
     motorLeft(200);
     delay(100); 
  }
  else {
      motorForward(200); 
  }
   
   while ((millis()-loopTimer) < LOOP_TIME){delay(20);} //Solo ejecuta bucle principal una vez cada LOOP_TIME
} 

void scan(){
  unsigned int uS = sonar.ping();
  distance = sonar.convert_cm(uS);
  if (distance==0){distance=MAX_DISTANCE;}
  Serial.print("Ping: ");
  Serial.print(distance);
  Serial.println("cm");
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

