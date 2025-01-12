
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab

bool doLogs = false;


// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR1 = 3575.0855;
const float GR1 = 297.924;
const float CPR1 = 3;

const float PPR2 = 601.1077;
const float GR2 = 50.092;
const float CPR2 = 3;
// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1 = 0;
float demandPositionInDegrees_m2 = 0;
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains
float Kp_m1 = 1.8, Kd_m1 = 0.8, Ki_m1 = 0;
float Kp_m2 = 2, Kd_m2 = 0, Ki_m2 = 0;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

void setup() {
  Serial.begin(9600);

  // Task 1: Initialize the pins using pinMode and attachInterrupt functions
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  
  pinMode(encoderPinB_m1, INPUT_PULLUP); 
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);
  
  pinMode(encoderPinB_m2, INPUT_PULLUP); 

  // Initialize motor control pins
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);
  
  delay(3000); // 等待一段时间
  previousTime = micros(); // 初始化时间
}

 
  
 
void loop() {
  
  if (doLogs == true) {
    Serial.print("beginning");
  }

  if (readyToSend == false) {
    if (Serial.available() > 0) 
    {       // is there anything received?
      c = Serial.read();                // read characters
      matlabStr = matlabStr + c;        // append characters to string as these are received
      
      if (matlabStr.indexOf(";") != -1)
      { // have we received a semicolon (indicates end of command from MATLAB)?
        readyToSend = true;             // then set flag to true since we have received the full command

        // parse incoming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
        val1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
        int posEnd = matlabStr.indexOf(";");                        // position of last character
        val2 = matlabStr.substring(posComma1 + 1, posEnd).toFloat(); // float from substring from comma+1 to end-1
        demandPositionInDegrees_m1=val1;
        demandPositionInDegrees_m2=val2;
      }
    }
  }
  
  
  // Task 2: Compute the current position in degrees and bound it to [-360,360]
   currentPositionInDegrees_m1 = ((counter_m1 * 360) / (CPR1 * GR1 * 2));
   currentPositionInDegrees_m2 = ((counter_m2 * 360) / (CPR2 * GR2 * 2));

   if (currentPositionInDegrees_m1 >= 360.0 || currentPositionInDegrees_m1 <= -360.0) {
    counter_m1 -= ((GR1 * CPR1 * 2) * ((int)(currentPositionInDegrees_m1 / 360)));
   }

   if (currentPositionInDegrees_m2 >= 360.0 || currentPositionInDegrees_m2 <= -360.0) {
    counter_m2 -= ((GR2 * CPR2 * 2) * ((int)(currentPositionInDegrees_m2 / 360)));
   }

  // Task 3: Compute elapsed time (deltaT) and control the frequency of printing
   currentTime = micros();
   deltaT = currentTime - previousTime;
   previousTime = currentTime;

   if (doLogs==true) {
    Serial.println(deltaT);
   }

   if (deltaT > 400) {
    // Task 4: Compute error (P,I,D)
     float errorPositionInDegrees_m1 = currentPositionInDegrees_m1 - demandPositionInDegrees_m1;
     float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
     errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
     errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;

     float errorPositionInDegrees_m2 = currentPositionInDegrees_m2 - demandPositionInDegrees_m2;
     float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
     errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
     errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    // Task 5: Compute the PID output
     float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;
      controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);
    
     float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;
     controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);


    // Check the counter to stop motor
     volatile long expectedcounter2 = demandPositionInDegrees_m2*(CPR2 * GR2 * 2)/360;
     if (abs(counter_m2) > expectedcounter2) {
       controllerOutput_m2 = 0; 
     }
     volatile long expectedcounter1 = demandPositionInDegrees_m1*(CPR1 * GR1 * 2)/360;
     if (abs(counter_m1) > expectedcounter1) {
       controllerOutput_m1 = 0; 
     }
 

    // Task 6: Send voltage to motors
     if (controllerOutput_m1 > 0) {
       digitalWrite(motorPin1_m1, HIGH);
       digitalWrite(motorPin2_m1, LOW);
       analogWrite(enablePin_m1, controllerOutput_m1);
     } else {
       digitalWrite(motorPin1_m1, LOW);
       digitalWrite(motorPin2_m1, HIGH);
       analogWrite(enablePin_m1, -controllerOutput_m1);
     }

     if (controllerOutput_m2 > 0) {
       digitalWrite(motorPin1_m2, HIGH);
       digitalWrite(motorPin2_m2, LOW);
       analogWrite(enablePin_m2, controllerOutput_m2);
     } else {
       digitalWrite(motorPin1_m2, LOW);
       digitalWrite(motorPin2_m2, HIGH);
       analogWrite(enablePin_m2, -controllerOutput_m2);
     }


    // updateEncoder_m1();
    // updateEncoder_m2();

  

    // Task 7: Print the current position and demand for plotting
    if (doLogs == true)
    {
    Serial.println("Loop is running...");
    Serial.print("Current Position m1: ");
    Serial.println(currentPositionInDegrees_m1);

    Serial.print("Demand Position m1: ");
    Serial.println(demandPositionInDegrees_m1);

    Serial.print("Current Position m2: ");
    Serial.println(currentPositionInDegrees_m2);

    Serial.print("Demand Position m2: ");
    Serial.println(demandPositionInDegrees_m2);
    }
  
}

  // Send data back to MATLAB if ready
   if (readyToSend) {
     Serial.print("c");                    // command
     Serial.print(currentPositionInDegrees_m1, 6);                      
     Serial.print(",");                    // delimiter
     Serial.print(currentPositionInDegrees_m2, 6); 
     Serial.write(13);                     // carriage return (CR)
     Serial.write(10);                     // new line (NL)
     //readyToSend = false;
     i += 1;
   }
}

// Interrupt functions for tracking the encoder positions
void updateEncoder_m1() {
  int aState = digitalRead(encoderPinA_m1);
  int bState = digitalRead(encoderPinB_m1);

  if (aState != aLastState_m1) {
    if (bState != aState) {
      counter_m1++;
    } else {
      counter_m1--;
    }
    aLastState_m1 = aState;
  }
  
}
 

void updateEncoder_m2() {
  int aState = digitalRead(encoderPinA_m2);
  int bState = digitalRead(encoderPinB_m2);

  if (aState != aLastState_m2) {
    if (bState != aState) {
      counter_m2++;
    } else {
      counter_m2--;
    }
    aLastState_m2 = aState;
  }
  
}

