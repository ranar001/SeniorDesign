//LEFT:
const int trigPin1=11;
const int echoPin1=10;
//RIGHT:
const int echoPin2=9;
const int trigPin2=8;
//FRONT
const int trigPin3=2;
const int echoPin3=3;
//REAR
const int trigPin4=13;
const int echoPin4=12;

//distance variables
float duration1, distance1; 
float duration2, distance2;
float duration3, distance3;
float duration4, distance4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);//LEFT
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);//RIGHT
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);//FRONT
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);//REAR
  pinMode(echoPin4, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//LEFT Sensor
 digitalWrite(trigPin1, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trigPin1, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(trigPin1, LOW); 
 
//LEFT Calculation
 duration1 = pulseIn(echoPin1, HIGH); 
 distance1 = (duration1*.0343)/2;

  Serial.print("LEFT Distance: "); Serial.println(distance1); //Serial.print(" cm.");
  delay(250); 

//RIGHT Sensor:
 digitalWrite(trigPin2, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trigPin2, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(trigPin2, LOW);

//RIGHT Calculation:
  duration2 = pulseIn(echoPin2, HIGH); 
  distance2 = (duration2*.0343)/2;
 
 Serial.print("RIGHT Distance: "); Serial.println(distance2); //Serial.print(" cm."); 
 delay(250); 

//FRONT Sensor:
 digitalWrite(trigPin3, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trigPin3, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(trigPin3, LOW);

//FRONT Calculation:
  duration3 = pulseIn(echoPin3, HIGH); 
  distance3 = (duration3*.0343)/2;
 
 Serial.print("FRONT Distance: "); Serial.println(distance3); //Serial.print(" cm."); 
 delay(250); 

 //REAR Sensor:
 digitalWrite(trigPin4, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trigPin4, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(trigPin4, LOW);

//REAR Calculation:
  duration4 = pulseIn(echoPin4, HIGH); 
  distance4 = (duration4*.0343)/2;
 
 Serial.print("REAR Distance: "); Serial.println(distance4); //Serial.print(" cm."); 
 delay(250); 
 
}  
