/*
 * For use with:
 * PRODUCTION TEST_210618_AUX_CONTROL.approjx
 * 
 * Uses the AUX Control output on the Audio Precision hardware to prompt the system on 
 * what channel is being tested.
 * 
 * The system then runs the appropriate positioning program for each type of bar.
 * 
 * Program Outline:
 * 
 * -Channel test based on input from Audio Precision AUX Control:
 * 
 *     -Center: (APX: 04)
 *          Yellow: Goes Left
 *          White: Goes Right
 *          (Bar needs to be positioned manually using the controls.)
 * 
 *     -Left: (APX: 05)
 *          Yellow: SPL3
 *          Green: SPL5Q/SPL6Q/SPL8Q
 *          White: BEQ tweeter
 * 
 *     -Right: (APX: 06)
 *          Yellow: SPL3
 *          Green: SPL5Q/SPL6Q/SPL8Q
 *          White: BEQ tweeter
 * 
 *     -CS Left:  (APX: 07)
 *          Yellow: SPL3
 *          Green: SPL5Q
 *          White: QX3
 *      
 */
#include <Tic.h>

TicI2C tic3(16); //tic3 on I2C address 16

//Global parameters (can be adjusted as needed)
const float SPL3CS = 3;//in
const float SPL5QCS = 5.625;//in
const float QX3CS = 5;//in

const float SPL3Limit = 5;//in
const float SPL5Limit = 3;//in
const float SPLBELimit = 1;//in

const long VLR = 100000000;//target speed for manual positioning


//Ultrasonic Sensor L
const int trigPinL = 30;
const int echoPinL = 31;

//Ultrasonic Sensor R
const int trigPinR = 47;
const int echoPinR = 48;

//define digital input pins
const int greenButton = 33;
const int whiteButton = 34;
const int yellowButton = 35;

//define digital output pins
const int greenLight = 38;
const int whiteLight = 39;
const int yellowLight = 40;


//define pins used for APX communication
const int apxPin1 = 22;
const int apxPin2 = 24;
const int apxPin3 = 26;
const int apxPin4 = 28;
const int apxPin5 = 23;
const int apxPin6 = 25;
const int apxPin7 = 27;
const int apxPin8 = 29;


//define global variables for the signal on each APX pin and set each to LOW
int apx1 = LOW;
int apx2 = LOW;
int apx3 = LOW;
int apx4 = LOW;
int apx5 = LOW;
int apx6 = LOW;
int apx7 = LOW;
int apx8 = LOW;


//define other global variables
byte apxCode = 0;
float LRLimit = 0;

//---------------------------------------------setup
void setup() {
  Serial.begin(9600);//Begin serial communication with PC

  //Setup digital inputs
  pinMode(greenButton, INPUT_PULLUP);
  pinMode(yellowButton, INPUT_PULLUP);
  pinMode(whiteButton, INPUT_PULLUP);
  
  //Ultrasonic Distance Sensor Left
  pinMode(trigPinL,OUTPUT);
  pinMode(echoPinL,INPUT);
  
  //Ultrasonic Distance Sensor Right
  pinMode(trigPinR,OUTPUT);
  pinMode(echoPinR,INPUT);

  //Setup Indicator Lights
  pinMode(greenLight, OUTPUT);
  pinMode(yellowLight, OUTPUT);
  pinMode(whiteLight, OUTPUT);

  //Setup apx communication pins
  pinMode(apxPin1, INPUT);
  pinMode(apxPin2, INPUT);
  pinMode(apxPin3, INPUT);
  pinMode(apxPin4, INPUT);
  pinMode(apxPin5, INPUT);
  pinMode(apxPin6, INPUT);
  pinMode(apxPin7, INPUT);
  pinMode(apxPin8, INPUT);

  ticConnect(); //connection to stepper driver established

  tic3.setProduct(TicProduct::Tic36v4); //sets stepper driver type
  
  tic3.deenergize(); //ensures that motor will not start energized on connection

  exitSafeStart(); //exit safe start command given

}

//---------------------------------------------loop
void loop() {
  testLoop();//goes to test loop function
}

//---------------------------------------------apxRead
void apxRead() {
  //reads the HIGH/LOW signals on each of the APX Aux Control pins
  apx1 = digitalRead(apxPin1);
  apx2 = digitalRead(apxPin2);
  apx3 = digitalRead(apxPin3);
  apx4 = digitalRead(apxPin4);
  apx5 = digitalRead(apxPin5);
  apx6 = digitalRead(apxPin6);
  apx7 = digitalRead(apxPin7);
  apx8 = digitalRead(apxPin8);


  //updates each digit of the apxCode variable 1/0
  bitWrite(apxCode, 0, apx1);
  bitWrite(apxCode, 1, apx2);
  bitWrite(apxCode, 2, apx3);
  bitWrite(apxCode, 3, apx4);
  bitWrite(apxCode, 4, apx5);
  bitWrite(apxCode, 5, apx6);
  bitWrite(apxCode, 6, apx7);
  bitWrite(apxCode, 7, apx8);
}

//---------------------------------------------emergencyStop
void emergencyStop() {
  tic3.deenergize();
  tic3.enterSafeStart();
  Serial.println("EMERGENCY STOP");
  do {
    digitalWrite(yellowLight, HIGH);
    delay(100);
    digitalWrite(yellowLight, LOW);
    delay(100);
    timeoutReset();
  } while (digitalRead(yellowButton) == HIGH || digitalRead(whiteButton) == HIGH);
}

//---------------------------------------------ticConnect
void ticConnect(){
  //establishes connection to the stepper driver
  Wire.begin();
  delay(25);
}

//---------------------------------------------exitSafeStart
void exitSafeStart(){
  //shortened exitSafeStart command
  tic3.exitSafeStart();
}

//---------------------------------------------testLoop
void testLoop()
{
    apxRead();
    if (apxCode == 04) { //-----------------------------Center
      Serial.println("Center");
      
      while (apxCode == 04){//while the APX Aux Control does not give the phase complete signal
        apxRead();
        Serial.println("apxRead");

        //allows the user to control Left/Right motion with the White/Yellow buttons

        //Left motion on white button press
        if (digitalRead(whiteButton) == LOW){
          Serial.println("whiteButtonPress");
          digitalWrite(yellowLight, LOW);
          digitalWrite(whiteLight, LOW);
          digitalWrite(greenLight,HIGH);
          Serial.println("greenLightOn");
          exitSafeStart();
          tic3.energize();
          Serial.println("energize");
          tic3.setTargetVelocity(VLR);
          Serial.println("targetVelocity:VLR");
        }

        //Right motion on yellow button press
        else if (digitalRead(yellowButton) == LOW){
          Serial.println("yellowButtonPress");
          digitalWrite(yellowLight, LOW);
          digitalWrite(whiteLight, LOW);
          digitalWrite(greenLight,HIGH);
          Serial.println("greenLightOn");
          exitSafeStart();
          tic3.energize();
          Serial.println("energize");
          tic3.setTargetVelocity(-VLR);
          Serial.println("targeVelocity:-VLR");
        }

        //deenergizes when no button is pressed
        else{
          Serial.println("noInput");
          digitalWrite(yellowLight, HIGH);
          digitalWrite(whiteLight, HIGH);
          digitalWrite(greenLight,LOW);
          Serial.println("greenLightOff");
          tic3.deenergize();
          Serial.println("deenergize");
        }
      }
      Serial.println("Center Done");
    }

    else if (apxCode == 05) { //-------------------------Left
      Serial.println("Left");
      timeoutReset();

      //turns lights on and waits for user input
      digitalWrite(yellowLight, HIGH);
      digitalWrite(greenLight, HIGH);
      digitalWrite(whiteLight, HIGH);

      Serial.print("Input: ");
      for(;;){
        apxRead();
        
        //SPL3 limit distance selected with yellow button
        if (digitalRead(yellowButton) == LOW){
          Serial.println("SPL3");
          LRLimit = SPL3Limit;
          break;
        }

        //SPL5Q/SPL6Q/SPL8Q limit distance selected with green button
        else if (digitalRead(greenButton) == LOW){
          Serial.println("SPL5Q");
          LRLimit = SPL5Limit;
          break;
        }

        //BEQ Tweeter limit distance selected with white button
        else if (digitalRead(whiteButton) == LOW){
          Serial.println("BEQ");
          LRLimit = SPLBELimit;
          break;
        }

        //When the user does not press a button and moves on to the rest of the test
        else if(apxCode != 05){
          digitalWrite(yellowLight, LOW);
          digitalWrite(greenLight, LOW);
          digitalWrite(whiteLight, LOW);
          goto leftEnd;
        }
        
        else{
          timeoutReset();
        }
      }

      //turns off all indicator lights
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight, LOW);
      digitalWrite(whiteLight, LOW);

      //turns on the yellow light to indicate that the system is in motion
      timeoutDelay(500);
      digitalWrite(yellowLight, HIGH);

      //halts motor before energizing
      tic3.haltAndSetPosition(0);
      tic3.energize();
      tic3.haltAndSetPosition(0);
      Serial.println("Halt");
      exitSafeStart();
      Serial.print("Initial Position: ");
      Serial.println(tic3.getCurrentPosition());

      //while the bar is > 10in away from the distance sensor, it will move left
      while (ultrasonicL() > 10) {
        moveLeft();
      }

      //while once the bar is < 10in away, moves at constant speed
      while(ultrasonicL() > LRLimit){
        if (digitalRead(yellowButton) == LOW || digitalRead(greenButton) == LOW || digitalRead(whiteButton) == LOW){
          emergencyStop();
        }        
        tic3.setTargetVelocity(-10000000);
        Serial.println("Slow");
      }

      //holds in position briefly once in position
      tic3.haltAndHold();
      delay(500);

      //deenergizes then turns on green light to indicate it is in position.
      tic3.deenergize();
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight, HIGH);
      Serial.println("In Position");

      //waits for phase complete indication from APX
      while(apxCode == 05){
        apxRead();
        timeoutReset();
      }
      digitalWrite(greenLight, LOW);
      
      leftEnd:
      Serial.println("Left Done");
    }

    else if (apxCode == 06) { //--------------------------Right
      Serial.println("Right");
      timeoutReset();

      //turns on lights and waits for user input
      digitalWrite(yellowLight, HIGH);
      digitalWrite(greenLight, HIGH);
      digitalWrite(whiteLight, HIGH);

      Serial.print("Input: ");
      for(;;){
        apxRead();
        
        //selects SPL3 limit distance on yellow button press
        if (digitalRead(yellowButton) == LOW){
          Serial.println("SPL3");
          LRLimit = SPL3Limit;
          break;
        }

        //selects SPL5Q/SPL6Q/SPL8Q limit distance on green button press
        else if (digitalRead(greenButton) == LOW){
          Serial.println("SPL5Q");
          LRLimit = SPL5Limit;
          break;
        }

        //selects BEQ limit distance on white button press
        else if (digitalRead(whiteButton) == LOW){
          Serial.println("BEQ");
          LRLimit = SPLBELimit;
          break;
        }

        //When the user does not press a button and moves on to the rest of the test
        else if (apxCode != 06){
          digitalWrite(yellowLight, LOW);
          digitalWrite(greenLight, LOW);
          digitalWrite(whiteLight, LOW);
          goto rightEnd;
        }
        
        else{
          timeoutReset();
        }
      }

      //turns off indicator lights
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight,  LOW);
      digitalWrite(whiteLight,  LOW);

      //turns on yellow light to indicate the system is in motion
      timeoutDelay(500);
      digitalWrite(yellowLight, HIGH);

      //halts motor before energizing
      tic3.haltAndSetPosition(0);
      tic3.energize();
      tic3.haltAndSetPosition(0);
      Serial.println("Halt");
      exitSafeStart();
      Serial.print("Initial Position: ");
      Serial.println(tic3.getCurrentPosition());

      //while the bar is < 10in from the end move right
      while (ultrasonicR() > 10) {
        moveRight();
      }

      //moves the bar towards the rigt at constant speed until the limit position is reached
      while(ultrasonicR() > LRLimit){
        if (digitalRead(yellowButton) == LOW || digitalRead(greenButton) == LOW || digitalRead(whiteButton) == LOW){
          emergencyStop();
        }        
        tic3.setTargetVelocity(10000000);
        Serial.println("Slow");
      }

      //stops and briefly holds position once limit is reached
      tic3.haltAndHold();
      delay(500);

      //deenergizes and turns on green light to indicate the bar is in position
      tic3.deenergize();
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight, HIGH);
      Serial.println("In Position");

      //waits for phase done signal from apx
      while (apxCode == 06){
        apxRead();
        timeoutReset();
      }
      digitalWrite(greenLight,LOW);

      rightEnd:
      Serial.println("Right Done");
    }

    else if (apxCode == 07) { //-------------------------CSLeft
      Serial.println("CSLeft");
      int CSStep = 0;

      //turns on all lights and waits for input
      digitalWrite(yellowLight, HIGH);
      digitalWrite(greenLight, HIGH);
      digitalWrite(whiteLight, HIGH);
      
      for(;;){
        apxRead();

        //SPL3CS position on yellow button input
        if(digitalRead(yellowButton) == LOW){
          Serial.println("SPL3CS");
          CSStep = stepConvert(SPL3CS);
          break;
        }

        //SPL5QCS position on green button input
        else if(digitalRead(greenButton) == LOW){
          Serial.println("SPL5QCS");
          CSStep = stepConvert(SPL5QCS);
          break;
        }

        //QX3CS position on white button input
        else if(digitalRead(whiteButton) == LOW){
          Serial.println("QX3CS");
          CSStep = stepConvert(QX3CS);
          break;
        }

        //When the user does not press a button and moves on to the rest of the test
        else if(apxCode != 07){
          digitalWrite(yellowLight, LOW);
          digitalWrite(greenLight, LOW);
          digitalWrite(whiteLight, LOW);
          goto csEnd;
        }
        
        else{
          timeoutReset();
        }
      }

      //turns off all indicator lights 
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight, LOW);
      digitalWrite(whiteLight, LOW);

      //turns on yellow light to indicate the system is in motion
      timeoutDelay(500);
      digitalWrite(yellowLight, HIGH);

      //halts motor before energizing
      tic3.haltAndSetPosition(0);
      exitSafeStart();
      tic3.energize();

      //sets target position to the position selected earlier
      //Change CStep to -CSStep to change direction
      tic3.setTargetPosition(CSStep);
      
      //resets the command timeout on the stepper driver while waiting for it to reach its position
      // change CSStep to -CSStep to change direction
      while(tic3.getCurrentPosition() != CSStep){
        if (digitalRead(yellowButton) == LOW || digitalRead(greenButton) == LOW || digitalRead(whiteButton) == LOW){
          emergencyStop();
        }
        timeoutReset();
      }
      delay(500);

      //deenergizes the motor and turns on the green light to indicate the bar is in position
      tic3.deenergize();
      digitalWrite(yellowLight, LOW);
      digitalWrite(greenLight, HIGH);

      //waits for phase done indication from apx
      while (apxCode == 07){
        apxRead();
        timeoutReset();
      }
      digitalWrite(greenLight, LOW);

      csEnd:
      Serial.println("CS Left Done");
    }
    
    else{//-----------------------------------else
      Serial.println("Default");
        apxRead();
        Serial.println("apxRead");

        //allows the user to control Left/Right motion with the White/Yellow buttons

        //Left motion on white button press
        if (digitalRead(whiteButton) == LOW){
          Serial.println("whiteButtonPress");
          digitalWrite(yellowLight, LOW);
          digitalWrite(whiteLight, LOW);
          digitalWrite(greenLight,HIGH);
          Serial.println("greenLightOn");
          exitSafeStart();
          tic3.energize();
          Serial.println("energize");
          tic3.setTargetVelocity(VLR);
          Serial.println("targetVelocity:VLR");
        }

        //Right motion on yellow button press
        else if (digitalRead(yellowButton) == LOW){
          Serial.println("yellowButtonPress");
          digitalWrite(yellowLight, LOW);
          digitalWrite(whiteLight, LOW);
          digitalWrite(greenLight,HIGH);
          Serial.println("greenLightOn");
          exitSafeStart();
          tic3.energize();
          Serial.println("energize");
          tic3.setTargetVelocity(-VLR);
          Serial.println("targeVelocity:-VLR");
        }

        //deenergizes when no button is pressed
        else{
          Serial.println("noInput");
          digitalWrite(yellowLight, HIGH);
          digitalWrite(whiteLight, HIGH);
          digitalWrite(greenLight,LOW);
          Serial.println("greenLightOff");
          tic3.deenergize();
          Serial.println("deenergize");
        }
    }
}

//---------------------------------------------timeoutReset
void timeoutReset() {
  //resets the stepper driver command timeout
  tic3.resetCommandTimeout();
}

//---------------------------------------------timeoutDelay
void timeoutDelay(int ms)
// adds a delay while keeping the stepper drivers from timing out
//FOR ROUGH TIMING ONLY resolution limitations and other commands may extend the time
//resolution of 100ms (will round up by 100ms if a higher resolution is inputed?)
{
  int timeA = ms / 100;
  for (int i = 0; i <= timeA; i++)
  {
    tic3.resetCommandTimeout();
    delay(100);
  }
}

//---------------------------------------------stepConvert
int stepConvert(int inputInches)
{
  /*
   * Takes a distance in inches as an input and outputs
   * the number of steps needed to travel that distance
   */
  long stepOut = 0;
  
  //checks the microstepping mode to use for calculations
  TicStepMode stepMode = tic3.getStepMode();
  
  //whole step
  if (stepMode == TicStepMode::Microstep1) {
    stepOut = inputInches * 100;
  }

  //half step
  else if (stepMode == TicStepMode::Microstep2) {
    stepOut = inputInches * 200;
  }
  
  //1/4 step
  else if (stepMode == TicStepMode::Microstep4) {
    stepOut = inputInches * 400;
  }
  
  //1/8 step
  else if (stepMode == TicStepMode::Microstep8) {
    stepOut = inputInches * 800;
  }
  
  //1/16 step
  else if (stepMode == TicStepMode::Microstep16) {
    stepOut = inputInches * 1600;
  }
  
  //1/32 step
  else if (stepMode == TicStepMode::Microstep32) {
    stepOut = inputInches * 3200;
  }
  
  //1/64 step
  else if (stepMode == TicStepMode::Microstep64) {
    stepOut = inputInches * 6400;
  }
  
  //1/128 step
  else if (stepMode == TicStepMode::Microstep128) {
    stepOut = inputInches * 12800;
  }
  
  //1/256 step
  else if (stepMode == TicStepMode::Microstep256) {
    stepOut = inputInches * 25600;
  }
  
  else {
    stepOut = inputInches * 100;
  }
  return stepOut;
}


//---------------------------------------------ultrasonicL
float ultrasonicL() {
  /*
   * takes measurement with left ultrasonic distance sensor
   * and outputs calculated distance in inches
   */
  //ensures the initial signal on the trig pin is LOW
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);

  //drives the trig pin HIGH, then LOW to cause ultrasonic ping
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  //measures the time it takes for the signal on the echo pin to go HIGH
  //this indicates the time it takes for the sound to be reflected back
  float TimeLA = pulseIn(echoPinL, HIGH, 250000);

  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  float TimeLB = pulseIn(echoPinL, HIGH, 250000);

  //calculates the distace based on the time measured and the speed of sound
  float distanceL = (((TimeLA + TimeLB)/2) * 0.0135039) / 2;

  //outputs distance
  Serial.println(distanceL);
  return distanceL;
}

//---------------------------------------------ultrasonicR
float ultrasonicR() {
  /*
   * takes measurement with right ultrasonic distance sensor
   * and outputs calculated distance in inches
   */
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  
  float TimeRA = pulseIn(echoPinR, HIGH);

  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);

  float TimeRB = pulseIn(echoPinR, HIGH);
  
  float distanceR = (((TimeRA + TimeRB)/2) * 0.0135039) / 2;
  Serial.println(distanceR);
  return distanceR;
}

//---------------------------------------------moveLeft
void moveLeft() {
  /*
   * sets the target velocity of the motor based on the distace
   * that the bar is from the left ultrasonic distance sensor
   */
  tic3.energize();
  float distL = ultrasonicL();
  if (digitalRead(yellowButton) == LOW || digitalRead(greenButton) == LOW || digitalRead(whiteButton) == LOW){
    emergencyStop();
  }
  timeoutReset();
  long velocityL = (distL-10) * 2000000 + 10000000;
  // change to velocityL to change direction of motor
  tic3.setTargetVelocity(-velocityL);
}

//---------------------------------------------moveRight
void moveRight() {
  /*
   * sets the target velocity of the motor based on the distance
   * that the bar is from the right ultrasonic distance sensor
   */
  tic3.energize();
  float distR = ultrasonicR();
  if (digitalRead(yellowButton) == LOW || digitalRead(greenButton) == LOW || digitalRead(whiteButton) == LOW){
    emergencyStop();
  }
  timeoutReset();
  long velocityR = (distR-10) * 2000000 + 10000000;
  //change to -velocityR to change direction of motor
  tic3.setTargetVelocity(velocityR);
}
