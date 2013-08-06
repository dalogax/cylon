#include <Servo.h> 
#include <NewPing.h>

#define TRIGGER_PIN  13   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     13   // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define UMBRAL 30          // Distancia que considera cercano (en cm)

//Servo head
#define PIN_SERVO 3

//Pines motores
#define ENA 5
#define IN1 11
#define IN2 10
#define ENB 6
#define IN3 9
#define IN4 8

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance. 
Servo servoHead;
 
int pos = 30; // variable to store the servo position 
unsigned int distance = 0;
 
void setup() 
{ 
  Serial.begin(115200);
  servoHead.attach(PIN_SERVO);  // attaches the servo on pin PIN_SERVO to the servo object 
  servoHead.write(80);
  
  pinMode(ENA, OUTPUT);     
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);     
  pinMode(ENB, OUTPUT);     
  pinMode(IN3, OUTPUT);     
  pinMode(IN4, OUTPUT);
  
} 
 
 
void loop() 
{    
    motorForward(200);
    scan();
    if (distance < UMBRAL){
        motorLeft(200);
       delay(100); 
    }
    else {
        motorForward(200); 
    }
    

} 

void scan(){
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  distance = sonar.convert_cm(uS);
  if (distance==0){distance=MAX_DISTANCE;}
  Serial.print("Ping: ");
  Serial.print(distance); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
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
