// Include the AccelStepper library:
#include <AccelStepper.h>
#include <stdio.h>

// Include servo library
#include <Servo.h>

// setting pushbutton pins
#define startHomingButton 6
#define startButton 7
#define weftInserted 8
#define stopButton 2
#define LED 26

// start button global variable 
int startHomingFlag = 0;
int startFlag = 0;

//////////////////////////////PATTERN//////////////////////////////
// initialise pattern matrix
const int colNum = 38;
const int rowNum = 34;
int myPattern[rowNum][colNum]=
{{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},
 {0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1},
 {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0},
 {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

// global variable for selection process
int rowCounter = 1;

// positions of pins
int pinPos;
int pinPosPrev;

//////////////////////////////STEPPERS//////////////////////////////
// Define the AccelStepper interface type (DRV8825 & TB6600)
#define MotorInterfaceType1 1

// Horizontal linear actuator stepper motor //
#define horDirPin 15
#define horStepPin 16

// Create a new instance of the AccelStepper class:
AccelStepper horStepper = AccelStepper(MotorInterfaceType1, horStepPin, horDirPin);

// horizontal stepper homing
#define homeSwitchHor 24

long horInitialHoming=1;
int moveToFirstPin = 1165;
int pinDist = 1116;
long horStepperPos = 0;

// Vertical linear actuator stepper motor //
#define vertDirPin 48
#define vertStepPin 46

// Create a new instance of the AccelStepper class:
AccelStepper vertStepper = AccelStepper(MotorInterfaceType1, vertStepPin, vertDirPin);

// vertical stepper homing 
#define homeSwitchVert 22

long vertInitialHoming=-1;
int moveToHeddleGuide = 11050;
int shed = 6000;
long vertStepperPos = 0;

// Cloth roll warp beam stepper motor // 
#define beamDirPin 52
#define beamStepPin 50

// Create a new instance of the AccelStepper class:
AccelStepper beamStepper = AccelStepper(MotorInterfaceType1, beamStepPin, beamDirPin);

long beamPos = 0;
int takeUpLetOff = 1000;

//////////////////////////////SERVOS//////////////////////////////
// servo setup 
#define servoMin   500
#define servoMax   2500

#define selectorServoPin  3  
#define beaterServo1Pin   4
#define beaterServo2Pin   5

Servo selectorServo; 
Servo beaterServo1;
Servo beaterServo2;

// selector servo motor positions in degrees
int pushIn = 130;
int pushOut = 60;

// beater positions in degrees
int retract = 135;
int extend = 45;

long correction = 225; //correction factor added to adjust for misalignments 

//////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // start serial monitor
  Serial.begin(9600);
  
  // setup pushbuttons as inputs
  pinMode(startButton, INPUT);
  pinMode(startHomingButton, INPUT);
  pinMode(weftInserted, INPUT);
  pinMode(stopButton, INPUT);
  
  // setup homing limit switches
  pinMode(homeSwitchHor, INPUT_PULLUP);
  pinMode(homeSwitchVert, INPUT_PULLUP);
    
  // setup LED for testing
  pinMode(LED, OUTPUT);

  // interrupt for stopButton 
  attachInterrupt(digitalPinToInterrupt(stopButton), emergencyStop, FALLING);

  // attach servo motors 
  selectorServo.attach(selectorServoPin, servoMin, servoMax);
  beaterServo1.attach(beaterServo1Pin, servoMin, servoMax);
  beaterServo2.attach(beaterServo2Pin, servoMin, servoMax);

  //position servos at start positions 
  selectorServo.write(pushOut);
  delay(1000);
  beaterServo1.write(retract);
  beaterServo2.write(retract);
  
  Serial.println("start homing?");
  //wait till startHoming button is pressed
  startHoming();
  
  // home horizontal stepper
  homeHorStepper();
  
  delay(1000);
  // home vertical stepper
  homeVertStepper();

  delay(1000);

  // cloth roll and warp beam stepper motor setup
  beamStepper.setCurrentPosition(0);
  beamStepper.setMaxSpeed(2000.00);
  beamStepper.setAcceleration(2000.00);
  delay(1000);
  
}

void loop() {
  //wait till start button is pressed 
  start();
  digitalWrite(53, HIGH);
  // checks if the pattern is completed or not  
  Serial.print("row:");
  Serial.println(rowCounter);
  if(rowCounter<rowNum){
    // takes up and lets off cloth and homes vertical and horizontal stepper motors every 5 lines of weaving
    if(rowCounter%5==0){
      Serial.println("Move cloth and warp");
      moveClothWarp();
      Serial.println("Homing");
      homeHorStepper();
      delay(1000);
      homeVertStepper();
      delay(1000);
      //checks which side of the loom the selector servo should be and moves the selector if necessary for the pattern to continue as normal
      if(rowCounter%2==0){
      }
      else{
        horStepperPos = horStepperPos - pinDist*37 + correction;
      }
    }
    else{
    }
    servoSelect();
    delay(4000);
    Serial.println();
    Serial.println("Move plates up");
    movePlatesUp();
    delay(4000);
    Serial.println("Retract beater");
    retractBeater();
    delay(4000);
    Serial.println("Wait for weft insertion");
    waitForWeft();
    delay(1000);
    Serial.println("Extend beater");
    extendBeater();
    delay(4000);
    if ((rowCounter)%5!=0){
      Serial.println("Move plates down");
      movePlatesDown();
    }
    else{
    }
    delay(4000);
  }

  else{
    Serial.println("Fabric is complete!");
    patternComplete();
  }
  
}

//////////////////////////////FUNCTIONS//////////////////////////////

/* Function Name: startHoming
 * Description: Waits till the button press for the homing of the stepper motors
 */
void startHoming(void){
 if (startHomingFlag == 0){
   while(digitalRead(startHomingButton)== LOW){  
  }
  startHomingFlag = 1;
 }
 else{
 }
}

/* Function Name: start
 * Description: Waits till the button press for the main loop to begin
 */
void start(void){
 if (startFlag == 0){
   Serial.println("start?");
   while(digitalRead(startButton)== LOW){  
  }
  startFlag = 1;
 }
 else{
 }
}

/* Function Name: waitForWeft
 * Description: Waits for the user to click the button to say the weft has been inserted
 */
void waitForWeft(void){
 while(digitalRead(weftInserted) == LOW){
 }
}

/* Function name: homeHorStepper
 * Function descritption: Homes the horizonatl stepper motor 
 */
void homeHorStepper(void){
  horStepper.setCurrentPosition(0);
  horStepperPos = 0;
  horInitialHoming = 1;
  
  // Set the maximum speed in steps per second
  horStepper.setMaxSpeed(1000.00);
  horStepper.setAcceleration(1000.00);

  // move stepper motor until the limit switch is hit
  while(digitalRead(homeSwitchHor)== HIGH){
    horStepper.moveTo(horInitialHoming);
    horInitialHoming++;
    horStepper.run();
  }

  delay(100);
  horStepper.setCurrentPosition(0);
  delay(100);
  
  // reset the maximum speed in steps per second (setCurrentPosition sets these values to zero)
  horStepper.setMaxSpeed(1000.00);
  horStepper.setAcceleration(1000.00);
  horInitialHoming=-1;

  // moves stepper motor away from limit switch
  while(digitalRead(homeSwitchHor) == LOW){
    horStepper.moveTo(horInitialHoming);
    horInitialHoming--;
    horStepper.run();
  }
  delay(1000);

  // sets a new maximum speed and acceleration 
  horStepper.setMaxSpeed(1500.00);
  horStepper.setAcceleration(1500.00);

  // moves turns stepper motor until selector servo is at the first pin
  horStepperPos = horStepperPos - moveToFirstPin;
  horStepperMove(horStepperPos);

  delay(1000);
}

/* Function name: homeVertStepper
 * Function descritption: Homes the vertical stepper motor 
 */
void homeVertStepper(void){
  vertStepper.setCurrentPosition(0);
  vertStepperPos = 0;
  vertInitialHoming=1;
  
  // Set the maximum speed in steps per second
  vertStepper.setMaxSpeed(400.00);
  vertStepper.setAcceleration(400.00);
 
  // move stepper motor until the limit switch is hit
  while(digitalRead(homeSwitchVert)== HIGH){
    vertStepper.moveTo(vertInitialHoming);
    vertInitialHoming++;
    vertStepper.run();
  }

  delay(100);
  vertStepper.setCurrentPosition(0);
  delay(100);
  
  // reset the maximum speed in steps per second (setCurrentPosition sets these values to zero)
  vertStepper.setMaxSpeed(400.00);
  vertStepper.setAcceleration(400.00);
  vertInitialHoming=-1;
  
  // moves stepper motor away from limit switch
  while(digitalRead(homeSwitchVert) == LOW){
    vertStepper.moveTo(vertInitialHoming);
    vertInitialHoming--;
    vertStepper.run();
  }
  delay(1000);

  // sets a new maximum speed and acceleration 
  vertStepper.setMaxSpeed(400.00);
  vertStepper.setAcceleration(400.00);

  // turns stepper motor until the frame that holds the pen click mechanisms is at the heddle plate guide
  vertStepperPos = vertStepperPos - moveToHeddleGuide;
  vertStepperMove(vertStepperPos);

  delay(1000);
}

/* Function name: servoSelect
 * Function descritption: changes the state of the pins
 */
void servoSelect(void){
  // checks if the row number is even or odd 
  Serial.println();
  if (rowCounter%2>0){
    // counts up the colNum-1 from zero if the row number is odd
    for (int col=0; col<colNum; col=col+1){
      pinPos = myPattern[rowCounter][col];
      pinPosPrev = myPattern[rowCounter-1][col];
      Serial.print(myPattern[rowCounter][col]);
      moveSelectorServo(pinPos, pinPosPrev);
      delay(500);
      // if the selector is at the end of the row don't do anything
      if(col==colNum-1){
        horStepperPos = horStepperPos + correction; //correction added to compensate for misalignment
        horStepperMove(horStepperPos);
        delay(500);
      }
      // if the selector is not at the end of the row move the stepper motor
      else{
        horStepperPos = horStepperPos - pinDist;
        horStepperMove(horStepperPos);
        delay(500);  
      }
    }
  }
  else{
    // counts down to zero from colNum - 1
    for (int colBack = colNum - 1; colBack>=0; colBack=colBack-1){
      pinPos = myPattern[rowCounter][colBack];
      pinPosPrev = myPattern[rowCounter-1][colBack];
      Serial.print(myPattern[rowCounter][colBack]);
      moveSelectorServo(pinPos, pinPosPrev);
      delay(500); 
      // if the selector is at the end of the row don't do anything
      if(colBack==0){
        horStepperPos = horStepperPos - correction; //correction added to compensate for misalignment
        horStepperMove(horStepperPos);
        delay(500);
      }
      // if the selector is not at the end of the row move the stepper motor
      else{
        horStepperPos = horStepperPos + pinDist;
        horStepperMove(horStepperPos);
        delay(500); 
      }     
    }
  }
  // add 1 to the global rowCounter variable
  rowCounter=rowCounter+1;
}

/* Function name: moveSelectorServo
 * Description: controls movement of selector servo
 */
void moveSelectorServo(int pinPos, int pinPosPrev){
  // determines if the current position of the pin is different from what it was previously 
  // current state required (in) equal previous state (in)
  if (pinPos == 0 & pinPosPrev == 0){ 
    delay(900);
  }
  // current state required (in) not equal previous state (out)
  else if (pinPos == 0 & pinPosPrev == 1){
    selectorServo.write(pushOut);
    delay(300);
    selectorServo.write(pushIn);
    delay(300);
    selectorServo.write(pushOut);
    delay(300);
  }
  // current state required (out) equal previous state (out)
  else if (pinPos == 1 & pinPosPrev == 1){
    delay(900);
  }
  // current state required (out) not equal previous state (in)
  else{
    selectorServo.write(pushOut);
    delay(300);
    selectorServo.write(pushIn);
    delay(300);
    selectorServo.write(pushOut);
    delay(300);
  }
}

/* Function name: horStepperMove
 * Description: moves selector
 */
void horStepperMove(long pos){
  horStepper.moveTo(pos);
  horStepper.runToPosition();
}

/* Function name: vertStepperMove
 * Description: moves frame
 */
void vertStepperMove(long pos){
  vertStepper.moveTo(pos);
  vertStepper.runToPosition();
}

/* Function name: retractBeater
 * Description: moves beater away from fell
 */
 void retractBeater(void){
  beaterServo1.write(retract);
  beaterServo2.write(retract);
  
 }

 /* Function name: extendBeater
 * Description: moves beater away from fell
 */
 void extendBeater(void){
  beaterServo1.write(extend);
  beaterServo2.write(extend);
 }

/* Function name: movePlatesUp
 * Description: Moves the frame that holds the pen click mechanisms down by rotating the vertical stepper motor 
 */
 void movePlatesUp(void){
  vertStepperPos = vertStepperPos + shed;
  vertStepperMove(vertStepperPos);
 }

/* Function name: movePlatesUp
 * Description: Moves the frame that holds the pen click mechanisms up by rotating the vertical stepper motor 
 */
 void movePlatesDown(void){
  vertStepperPos = vertStepperPos - shed;
  vertStepperMove(vertStepperPos);
 }

/* Function name: moveClothWarp
 * Description: Rotates the motor that controls the warp beam and cloth roll
 */
 void moveClothWarp(void){
    beamPos = beamPos + takeUpLetOff;
    beamStepper.moveTo(beamPos);
    beamStepper.runToPosition();
 }
 
/* Function name: emergencyStop
 * Description: stops the program if button stopButton is pressed and waits for the user to press the start button again before the program continues
 */
 void emergencyStop(void){
  digitalWrite(LED,HIGH);
  while(digitalRead(startButton)==LOW);
  digitalWrite(LED,LOW);
  delay(4000);
}

/* Function name: patternComplete
 * Description: Puts system in an infinite loop 
 */
 void patternComplete(void){
  while(1);
}
  
