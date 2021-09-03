#include <AccelStepper.h>

#define M1_EN_PIN 7
#define M1_STEP_PIN 8
#define M1_DIR_PIN 9

#define M2_EN_PIN 4
#define M2_STEP_PIN 5
#define M2_DIR_PIN 6

int homeSwitchPin[] = {2, 3};

const int numSteppers = 2;
AccelStepper stepper1 = AccelStepper(stepper1.DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper stepper2 = AccelStepper(stepper2.DRIVER, M2_STEP_PIN, M2_DIR_PIN);
AccelStepper* steppers[numSteppers] = {&stepper1, &stepper2}; 
//for multiple steppers see https://forum.arduino.cc/t/solved-accelstepper-library-and-multiple-stepper-motors/379416/2

// each element corresponds to a stepper motor
float maxMotorSpeed[] = {0.5, 0.5}; //units of rev per sec
float maxMotorAccel[] = {1.0, 1.0}; //units of rev per sec per sec
float stepsPerRev[] = {1600.0, 1600.0}; // 1/8 microstepping
float distPerRev[] = {8.0, 8.0}; // 2 mm pitch w/ 4 start lead screw = 8 mm lead (or 8 mm traveled per lead screw revolution)
float homingBuffer[] = {5.0, 5.0}; //units: mm. how much to move forward after triggering homing/limit switch.
float minPosition[] = {0.0, 0.0};
float maxPosition[] = {80.0, 80.0};
bool isForwardCCW[] = {true, true}; //determines which direction is forward or positive
bool hasHomed[] = {false, false};
bool isHoming[] = {false, false};
bool hasReported[] = {true, true};
bool disableWhenStill[] = {true, true};

float stepsPerDist[numSteppers];

// see Serial Input Basics by Robin2: https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
const int maxChars = 32; //max number of chars when reading from serial into cstring. anything larger rewrites last char of cstring (?)
const int maxTokens = 10; //max number of "words" or "tokens" separated by spaces (e.g. "mv samx 1" is 3 words or 3 tokens). print error if exceeded
char incomingChars[maxChars]; //incoming chars until newline terminator
char incomingParams[maxTokens][maxChars]; //incoming chars parsed into cstrings
char tempChars[maxChars];
bool newSerialData = false; // whether there is new data parsed from serial that needs to be processed
int numTokens = 0; //number of tokens or params
//strtoX endptr https://stackoverflow.com/questions/27973759/return-value-of-strtod-if-string-equals-to-zero

char charACK[] = "ACK"; //"\x06";
char charNAK[] = "NAK";
char charSUCC[] = "SUCC";
char charFAIL[] = "FAIL";
char charWARN[] = "WARN";

bool isDebugging = false;

//----------SETUP----------
void setup() 
{
    //initialize stepper params
    //tmc2209 silentstepstick stepper driver in standalone/legacy mode
    steppers[0]->setEnablePin(M1_EN_PIN);
    steppers[1]->setEnablePin(M2_EN_PIN);
    
    for (int i = 0; i < numSteppers; ++i)
    {
        stepsPerDist[i] = stepsPerRev[i] / distPerRev[i];
        
        pinMode(homeSwitchPin[i], INPUT);
  
        steppers[i]->setPinsInverted(isForwardCCW[i], false, true); 
        //dir, step, enable. if dir not inverted, then +'ve step produces lead screw rotation clockwise (looking from motor to end) and carriage moves towards the motor
        
        steppers[i]->enableOutputs();
        steppers[i]->setAcceleration(stepsPerRev[i] * maxMotorAccel[i]);
        steppers[i]->setMaxSpeed(stepsPerRev[i] * maxMotorSpeed[i]);
        steppers[i]->setSpeed(stepsPerRev[i] * maxMotorSpeed[i]);
        steppers[i]->setCurrentPosition(0);

        if (disableWhenStill[i])
        {
            steppers[i]->disableOutputs();
        }
    }

    Serial.begin(115200);
}

void loop() 
{
    readNextSerial();

    //Process new parsed commands
    if (newSerialData == true)
    {
        if (isDebugging)
        {
            Serial.print(">>");
            Serial.println(incomingChars);
        }

        strcpy(tempChars, incomingChars);
        parseTokens();
        goToCommand();
    }

    //run stepper motors
    for (int i = 0; i < numSteppers; ++i)
    {
      // if currently homing and the limit switch is depressed
      if (isHoming[i] && !digitalRead(homeSwitchPin[i]))
      {
            steppers[i]->enableOutputs();
            steppers[i]->move(homingBuffer[i] * stepsPerDist[i]);
            hasReported[i] = false;   
      }
      
      if (steppers[i]->distanceToGo() != 0)
      {
          // can this cause intervals to skip? because not EXACTLY a multiple of 100 on evaluation
          // might depend on how slow/fast the main loop is, if too fast then it will repeat, if to slow it may skip
          // not too critical since only used for printing 
          if (millis() % 100 == 0 && isDebugging)
          {
              Serial.print("Moving, distance to go for stepper ");
              Serial.print(i+1);
              Serial.print(" is: ");
              Serial.print(steppers[i]->distanceToGo() / stepsPerDist[i]);
              Serial.print(" at ");
              Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);
          }
          steppers[i]->run();
      }
      else if (!hasReported[i])
      {
          if (isHoming[i])
          {
            steppers[i]->setCurrentPosition(0);
            isHoming[i] = false;
            hasHomed[i] = true;

            //Homing completion message
            printSUCC();
            Serial.print("Stepper ");
            Serial.print(i+1);
            Serial.println(" has been homed. Current position set to zero.");
            
          }
          else
          {
            //Regular move completion message
            printSUCC();
            Serial.print("Move complete for stepper  ");
            Serial.print(i+1);
            Serial.print(" at ");
            Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);
          }
           if (disableWhenStill[i])
          {
              steppers[i]->disableOutputs();
          }
          hasReported[i] = true;    
      }
      else if(hasReported[i] && isHoming[i])
      {
          isHoming[i] = false;
          hasHomed[i] = false;
          
          printFAIL();
          Serial.print(F("Did not hit limit switch when homing stepper "));
          Serial.print(i+1);
          Serial.println(F(". Stepper has not been homed. Inspect system and try again."));
      }
    }
   
    //reset new data flag
    newSerialData = false;
    //reset the parsed tokens
    for (int i = 0; i < numTokens; ++i)
    {
        incomingParams[i][0] = '\0';
    }
    //reset the token counter
    numTokens = 0;
}

//----------SERIAL AND UTILITY FUNCTIONS----------
void readNextSerial()
{
    //read serial buffer until newline, read into incomingChars
    static bool hasReadHeader = false;
    static int ndx = 0;
    char nextChar;
    char footer = '\n';
    char header = '>';

    while (Serial.available() > 0 && newSerialData == false)
    {
        nextChar = Serial.read();

        if (hasReadHeader == true)
        {
            //keep adding into cstring until newline
            if (nextChar != footer)
            {
                incomingChars[ndx] = nextChar;
                ++ndx;
                if (ndx >= maxChars)
                {
                    ndx = maxChars - 1;
                }
            }
            else
            {
                //if reading newline then terminate cstring and indicate there is new data to process
                incomingChars[ndx] = '\0';
                ndx = 0;
                newSerialData = true;
                hasReadHeader = false;
            }
        }
        else if (nextChar == header)
        {
            hasReadHeader = true;
        }
    }
}

void parseTokens()
{
    char* token;
    numTokens = 0;

    token = strtok(tempChars, " ");

    while (token != NULL)
    {
        //save into cstring array and read next portion
        strcpy(incomingParams[numTokens], token);
        ++numTokens;

        if (numTokens >= maxTokens)
        {
            numTokens = maxTokens - 1;
        }
        token = strtok(NULL, " ");
    }
}

bool isValidMotorNum(char* str, long& num)
{
    char* endptr;
    num = strtol(str, &endptr, 10);
    if (endptr == str || *endptr != '\0')
    {
        printNAK();
        Serial.print("Parameter ");
        Serial.print(str);
        Serial.println(" is not an integer");
        return false;
    }
    else
    {
        if (num <= 0 || num > numSteppers)
        {
            printNAK();
            Serial.println("Stepper number is invalid");
            return false;
        }
        else
        {
            return true;
        }
    }
}

bool isValidTargetPos(char* str, long motorNum, double& pos)
{
    char* endptr;
    pos = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        printNAK();
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");
        return false;
    }
    else
    {
        if (pos < minPosition[motorNum-1] || pos > maxPosition[motorNum-1])
        {
            printNAK();
            Serial.print("Position ");
            Serial.print(pos);
            Serial.println(" is out of range");
            return false;
        }
        else
        {
          return true;
        }
    }
}

bool isValidRelativeDist(char* str, long motorNum, double& dist, double& pos)
{
    char* endptr;
    dist = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        printNAK();
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");
        return false;
    }
    else
    {
        pos = steppers[motorNum-1]->currentPosition() / stepsPerDist[motorNum-1] + dist;
        if (pos < minPosition[motorNum-1] || pos > maxPosition[motorNum-1])
        {
            printNAK();
            Serial.print("Position ");
            Serial.print(pos);
            Serial.println(" is out of range");
            return false;
        }
        else
        {
          return true;
        }
    }
}

//---------COMMAND FUNCTIONS------------
void homeMotor()
{
    if (numTokens == 1)
    {
        printlnACK();
        for (int i = 0; i < numSteppers; ++i)
        {
            double distance = maxPosition[i] - minPosition[i] + homingBuffer[i] + 2.0;
            steppers[i]->enableOutputs();
            steppers[i]->move(-1.0*distance * stepsPerDist[i]);
            isHoming[i] = true;
        }
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();
        double distance = maxPosition[motorNum-1] - minPosition[motorNum-1] + homingBuffer[motorNum-1] + 2.0;
        steppers[motorNum-1]->enableOutputs();
        steppers[motorNum-1]->move(-1.0*distance * stepsPerDist[motorNum-1]);
        isHoming[motorNum-1] = true;
    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");
    }
}

void unhomeMotor()
{
    if (numTokens == 1)
    {
        printlnACK();
        for (int i = 0; i < numSteppers; ++i)
        {
            steppers[i]->stop();
            steppers[i]->disableOutputs();
            isHoming[i] = false;
            hasHomed[i] = false;
        }
        printSUCC();
        Serial.println("Flagging all motors as not homed");
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();

        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->disableOutputs();
        isHoming[motorNum-1] = false;
        hasHomed[motorNum-1] = false;

        printSUCC();
        Serial.print("Flagging stepper ");
        Serial.print(motorNum);
        Serial.println(" as not homed");
    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");
    }
}

void moveAbsolute()
{
    //Check the number of tokens
    if (numTokens != 3)
    {
        printNAK();
        Serial.println("Invalid number of parameters");
        return;
    }

    //Check that the first number is an integer and is within the range of valid stepper numbers
    long motorNum;
    if (!isValidMotorNum(incomingParams[1], motorNum))
    {
        return;
    }
    
    //Check that the second number is an actual number and is within the range of valid positons
    double targetPos;
    if (!isValidTargetPos(incomingParams[2], motorNum, targetPos))
    {
        return;
    }

    printlnACK();

    if (!hasHomed[motorNum-1])
    {
        printFAIL();
        Serial.print("You must home stepper ");
        Serial.print(motorNum);
        Serial.println(" before you can move it");
        return;
    }

    //Successful 
    if (isDebugging)
    {
        Serial.print("Moving stepper ");
        Serial.print(motorNum);
        Serial.print(" to ");
        Serial.println(targetPos);
    }
    //enable and disable outputs when not moving in future
    steppers[motorNum-1]->enableOutputs();
    steppers[motorNum-1]->moveTo(targetPos * stepsPerDist[motorNum-1]);
    hasReported[motorNum-1] = false;
    
}

void moveRelative()
{
    //Check the number of tokens
    if (numTokens != 3)
    {
        printNAK();
        Serial.println("Invalid number of parameters");
        return;
    }

    //Check that the first number is an integer and is within the range of valid stepper numbers
    long motorNum;
    if (!isValidMotorNum(incomingParams[1], motorNum))
    {
        return;
    }

    //Check that the second number is an actual number and is within the range of valid positons
    double distance;
    double targetPos;
    if (!isValidRelativeDist(incomingParams[2], motorNum, distance, targetPos))
    {
        return;
    }  

    printlnACK();

    if (!hasHomed[motorNum-1])
    {
        printFAIL();
        Serial.print("You must home stepper ");
        Serial.print(motorNum);
        Serial.println(" before you can move it");
        return;
    }

    //Successful 
    if (isDebugging)
    {
        Serial.print("Moving stepper ");
        Serial.print(motorNum);
        Serial.print(" by ");
        Serial.print(distance);
        Serial.print(" to ");
        Serial.println(targetPos);
    }

    //enable and disable outputs when not moving in future
    steppers[motorNum-1]->enableOutputs();
    steppers[motorNum-1]->moveTo(targetPos * stepsPerDist[motorNum-1]);
    //steppers[motorNum-1]->move(distance * stepsPerDist[motorNum-1]); //same as above
    hasReported[motorNum-1] = false;
}

//void setPosition()
//{
//    //dont make available to the user.
//    Serial.println("Reset position command");
//}

void stopMotor()
{
    //WARNING stop will send ACK/NAK and SUCC if already stopped (main loop will send SUCC if moving then stopping)
    //BUT if the motor is already moving a SUCC/FAIL is expected for that command and calling stop will mess up the expected flow control by 
    //interjecting an additional ACK/NAK before reporting completion/stop with SUCC/FAIL
    //In other words, this is fine for manual control or emergency stops but not for automation when specific protocol is expected from the host
    if (numTokens == 1)
    {
        printlnACK();
        if (isDebugging)
        {
            Serial.println(F("Stopping all stepper motors"));
        }
        for (int i = 0; i < numSteppers; ++i)
        {
            steppers[i]->stop();

            if (steppers[i]->distanceToGo() == 0)
            {
                printSUCC();
                Serial.print("Stepper ");
                Serial.print(i+1);
                Serial.println(" is already stopped");
            }
            //if stepper is moving then SUCC will be sent when reporting completion
        }
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();

        if (isDebugging)
        {
            Serial.print("Stopping stepper ");
            Serial.println(motorNum);
        }
        
        steppers[motorNum-1]->stop();

        if (steppers[motorNum-1]->distanceToGo() == 0)
        {
            printSUCC();
            Serial.print("Stepper ");
            Serial.print(motorNum);
            Serial.println(" is already stopped");
        }
        //if stepper is moving then SUCC will be sent when reporting completion
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }
}

void whereMotor()
{
    if (numTokens == 1)
    {
      printlnACK();
      for (int i = 0; i < numSteppers; ++i)
      {
         printSUCC();
         Serial.print("Stepper ");
         Serial.print(i+1);
         Serial.print(" is at = ");
         Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);
      }
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();
        printSUCC();
        Serial.print("Stepper ");
        Serial.print(motorNum);
        Serial.print(" is at = ");
        Serial.println(steppers[motorNum-1]->currentPosition() / stepsPerDist[motorNum-1]);
    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");
    }
}

void enableMotor()
{
    if (numTokens == 1)
    {
        printlnACK();
        if (isDebugging)
        {
            Serial.println(F("Enabling all stepper motors"));
        }
        for (int i = 0; i < numSteppers; ++i)
        {
            steppers[i]->stop();
            steppers[i]->enableOutputs();
        }
        printSUCC();
        Serial.println(F("Steppers have been enabled"));
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();

        if (isDebugging)
        {
            Serial.print("Enabling stepper ");
            Serial.println(motorNum);
        }

        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->enableOutputs();

        printSUCC();
        Serial.print("Stepper ");
        Serial.print(motorNum);
        Serial.println(" has been enabled");
    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");
    }
}

void disableMotor()
{
    if (numTokens == 1)
    {
        printlnACK();

        if (isDebugging)
        {
            Serial.println(F("Disabling all stepper motors"));
        }
        for (int i = 0; i < numSteppers; ++i)
        {
            steppers[i]->stop();
            steppers[i]->disableOutputs();
        }
        printSUCC();
        Serial.println(F("Steppers have been disabled"));
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printlnACK();

        if (isDebugging)
        {
            Serial.print("Disabling stepper ");
            Serial.println(motorNum);
        }

        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->disableOutputs();

        printSUCC();
        Serial.print("Stepper ");
        Serial.print(motorNum);
        Serial.println(" has been disabled");
    }
    else
    {
        printNAK();
        Serial.println("Invalid number of parameters");
    }
}

void setDebugOn()
{
    if (numTokens == 1)
    {
        isDebugging = true;
        Serial.println(F("Debug messages turned ON"));
    }
    else
    {
        if (isDebugging)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

void setDebugOff()
{
    if (numTokens == 1)
    {
        isDebugging = false;
        //should not be able to print below if false
        //Serial.println("Debug messages turned OFF");
    }
    else
    {
        if (isDebugging)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

void goToCommand()
{
    //Run function based on first parsed token
    //(cannot use switch/case for cstring comparisons, it is actually better with the if-else ladder in this particular situation)
    if (strcmp(incomingParams[0], "hm") == 0)
    {   
        homeMotor();
    }
    else if (strcmp(incomingParams[0], "unhm") == 0)
    {   
        unhomeMotor();
    }
    else if (strcmp(incomingParams[0], "mv") == 0)
    {   
        moveAbsolute();
    }
    else if (strcmp(incomingParams[0], "mvr") == 0)
    {
        moveRelative();
    }
    else if (strcmp(incomingParams[0], "set") == 0)
    {
        Serial.println(F("Resetting vars is not allowed for the user"));
        //setPosition();
    }
    else if (strcmp(incomingParams[0], "stop") == 0 || strcmp(incomingParams[0], "s") == 0)
    {
        stopMotor();
    }
    else if (strcmp(incomingParams[0], "wm") == 0)
    {
        whereMotor();
    }
    else if (strcmp(incomingParams[0], "enm") == 0)
    {
        enableMotor();
    }
    else if (strcmp(incomingParams[0], "dism") == 0)
    {
        disableMotor();
    }
    else if (strcmp(incomingParams[0], "debugon") == 0)
    {
        setDebugOn();
    }
    else if (strcmp(incomingParams[0], "debugoff") == 0)
    {
        setDebugOff();
    }
    else if (strcmp(incomingParams[0], "whoru") == 0)
    {
        printWhoAmI();
    }
    else if (strcmp(incomingParams[0], "help") == 0)
    {
        printHelp();
    }
    else if (strcmp(incomingParams[0], "info") == 0)
    {
        printMotorInfoCmd();
    }
    else if (strcmp(incomingParams[0], "") == 0)
    {
        //Serial.println("Blank token");
        //this happens if only header and footer are sent
    }
    else
    {
        //no command exists
        printNAK();
        Serial.print("No command \"");
        Serial.print(incomingParams[0]);
        Serial.println("\" exists. Type \"help\' for more info.");
    }
}

void printWhoAmI()
{
    Serial.println(F("This is an Arduino controller for stepper motors"));
}

void printHelp()
{
    Serial.println(F("=================================== List of commands ==================================="));
    Serial.println(F("A = motor number from 1 to total number of motors"));
    Serial.println(F("B = distance in units of mm"));
    Serial.println(F("info A          // Prints information about stepper motor A. Leave A blank for info on all motors."));
    Serial.println(F("hm A            // Homes motor A and sets zero position. REQUIRED BEFORE MOVABLE. Leave A blank to home all motors."));
    Serial.println(F("unhm A          // Flags motor A as being not homed. Leave A blank to un-home all motors."));
    Serial.println(F("mv A B          // Moves motor A to absolute position B (mm)."));
    Serial.println(F("mvr A B         // Moves motor A by distance B (mm)."));
    Serial.println(F("stop A (OR s A) // Stops motor A. Leave A blank to stop all motors."));
    Serial.println(F("wm A            // Prints where is motor A position (mm). Leave A blank for position of all motors."));
    Serial.println(F("enm A           // Enables motor A's stepper driver. Stepper holds position when not moving. Leave A blank to enable all motors."));
    Serial.println(F("dism A          // Disables motor A's stepper driver. Stepper may be rotated by external force when not moving. Leave A blank to disable all motors."));
    Serial.println(F("whoru           // Prints what this Arduino does (useful for COM port indentification)."));
}

void printMotorInfoCmd()
{
    if (numTokens != 1 && numTokens != 2)
    {
        Serial.println("Invalid number of parameters");
        return;
    }
    else if (numTokens == 1)
    {   
        Serial.print("Total number of steppers: ");
        Serial.println(numSteppers);
        for (int i = 0; i < numSteppers; ++i)
        {
            printMotorInfo(i);
        }
    }
    else
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printMotorInfo(motorNum-1);
    }
}

void printMotorInfo(int i)
{
    Serial.print("\n");
    Serial.print(F("===== Stepper "));
    Serial.print(i+1);
    Serial.println(F(" Info ====="));

    Serial.print(F("Has been homed?:                  "));
    Serial.println(hasHomed[i]);
    
    Serial.print(F("Current position (mm):            "));
    Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);

    Serial.print(F("Lower position limit (mm):        "));
    Serial.println(minPosition[i]);

    Serial.print(F("Upper position limit (mm):        "));
    Serial.println(maxPosition[i]);

    Serial.print(F("Homing buffer gap (mm):           "));
    Serial.println(homingBuffer[i]);

    Serial.print(F("Disable when not moving?:         "));
    Serial.println(disableWhenStill[i]);

    Serial.print(F("Is forward CCW?:                  "));
    Serial.println(isForwardCCW[i]);

    Serial.print(F("Max speed (mm/s [rev/s]):         "));
    Serial.print(maxMotorSpeed[i] * distPerRev[i]);
    Serial.print(" [");
    Serial.print(maxMotorSpeed[i]);
    Serial.println("]");

    Serial.print(F("Acceleration (mm/s^2 [rev/s^2]):  "));
    Serial.print(maxMotorAccel[i] * distPerRev[i]);
    Serial.print(" [");
    Serial.print(maxMotorAccel[i]);
    Serial.println("]");

    Serial.print(F("Steps per distance (steps/mm):    "));
    Serial.println(stepsPerDist[i]);

    Serial.print(F("Steps per revolution:             "));
    Serial.println(stepsPerRev[i]);

    Serial.print(F("Distance per revolution (mm/rev): "));
    Serial.println(distPerRev[i]);
}

//printing control char utility functions
void printACK()
{
    Serial.print(charACK);
    Serial.print(": ");
}

void printlnACK()
{
    Serial.print(charACK);
    Serial.print(": ");
    Serial.println(incomingChars);
}

void printNAK()
{
    Serial.print(charNAK);
    Serial.print(": ");
}

void printlnNAK()
{
    Serial.println(charNAK);
}

void printSUCC()
{
    Serial.print(charSUCC);
    Serial.print(": ");
}


void printlnSUCC()
{
    Serial.println(charSUCC);
}

void printFAIL()
{
    Serial.print(charFAIL);
    Serial.print(": ");
}

void printlnFAIL()
{
    Serial.println(charFAIL);
}

void printWARN()
{
    Serial.print(charWARN);
    Serial.print(": ");
}

void printlnWARN()
{
    Serial.println(charWARN);
}
