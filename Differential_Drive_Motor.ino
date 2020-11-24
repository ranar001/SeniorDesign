// Left Motor(Back Wheel) 
int enA = 10;             //enable A gets pin 10
int in1 = 9;              //input 1 gets pin 9
int in2 = 8;              //input 2 gets pin 8
// Right Motor(Back Wheel)
int enB = 5;              //enable B gets pin 5
int in3 = 7;              //input 3 gets pin 7
int in4 = 6;              //input 4 gets pin 6

//
void setup()
{
  // Declare motor control pins to be in output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // A 2s delay before starting main loop
  // The LED at pin 13 will turn on for ~2s to indicate delay
  digitalWrite(13, HIGH);
  delay(1900);            //Delay for 1.9 sec
  digitalWrite(13, LOW);
  delay(100);             //Delay for 0.1 sec
}

/*  Move forward function
 *  Dir (Boolean) { true: Forward,
 *        false: Backward }
 *  Spd (Int) { 0 <-> 255 }
 *  Dur (Int) { Duration (in ms) }
 */

void moveBot(bool dir, int spd, int dur){
  // Back Left Motor
  digitalWrite(in1, dir);
  digitalWrite(in2, !dir);  //Inverts boolean value.
  // Back Right Motor
  digitalWrite(in3, dir);
  digitalWrite(in4, !dir);
  // Set motor speed to spd
  analogWrite(enA, spd);
  analogWrite(enB, spd);
  //Motion Duration
  delay(dur);
}

/*  Rotate function
 *  Dir (Boolean) { true: Clockwise,
 *        false: Counter-clockwise }
 *  Spd (Int) { 0 <-> 255 }
 *  Dur (Int) { Duration (in ms) }
 */

void rotateBot(bool dir, int spd, int dur){
  // Back Left Motor
  digitalWrite(in1, dir);
  digitalWrite(in2, !dir);  //Inverts boolean value.
  // Back Right Motor
  digitalWrite(in3, !dir);
  digitalWrite(in4, dir);
  // Set motor speed to spd
  analogWrite(enA, spd);    
  analogWrite(enB, spd);
  //Rotation Duration
  delay(dur);
}

//Turn off both motors
void stopMotors(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW); 
}

//Alway run this code
void loop()
{
  // Move forward for 2s @ speed 200
  moveBot(true, 200, 2000);
  // Rotate bot for 1s clockwise @ speed 150
  rotateBot(true, 200, 1000);
  // Move backward for 2s @ speed 200
  moveBot(false, 200, 2000);
  // Rotate bot for 1s counter-clockwise @ speed 150
  rotateBot(false, 200, 1000);
  // Stop motors for 1s @ speed 200
  stopMotors(); //fxn. call to stop both motors after doing loop fxn.
  delay(1000);
}
