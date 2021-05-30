#include <AccelStepper.h>
#include <CircularBuffer.h>
#include <MultiStepper.h>
#include <TMCStepper.h>
#include <Wire.h>
#include <WireSlaveRequest.h>
#include <config.h>
#include <Calibration.h>
#include <math.h>

#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
 
#include "SPIFFS.h"

//------------------------------------------------------------------------------
// GLOBALS

#define mmXSteps 40.0 
#define stepsXrev (200.0 * MICROSTEPPING)

#define MaxSpeedMMxSec 200

//board dimensions
#define BoardLength 475 
#define BoardHeight 475 
#define d1 9.19
#define d2 9.19

#define MaxStepsXSecondAllowed 8000
struct BoardPosition {
    double x;
    double y;
};

struct StringLengths {
    double s1;
    double s2;
    double s3;
    double s4;
};

CircularBuffer<String, 100> commands;
String commandRead;

float a, b, c, d, e, f, g;
int amountData;
float offsetx = 90;
float offsety = 90;
char whiteVector[16] = { 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v'};
char blackVector[16]  = { 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v', 'v'};
int indexB = 0;
int indexN = 0;
float xB = 4.0;
float yB = 3.5;
float xN = -4.0;
float yN = -3.5;

float xB2 = 4.5;
float xN2 = -4.5;


extern TMC2209Stepper driver;
extern TMC2209Stepper driver2;
extern TMC2209Stepper driver3;
extern TMC2209Stepper driver4;


AccelStepper stepper1(AccelStepper::DRIVER, MOTOR_0_STEP_PIN, MOTOR_0_DIR_PIN); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN);
AccelStepper stepper3(AccelStepper::DRIVER, MOTOR_2_STEP_PIN, MOTOR_2_DIR_PIN);
AccelStepper stepper4(AccelStepper::DRIVER, MOTOR_3_STEP_PIN, MOTOR_3_DIR_PIN);

MultiStepper steppers;

int flag = 0;
int contCycles = 0;
int cont_m = 0;
//------------------------------------------------------------------------------


void tenseRelease(float num);
StringLengths computeStringLengths(double x, double y);
int readFromSerial();

long positionOrigin1, positionOrigin2, positionOrigin3, positionOrigin4;

BoardPosition commandToCoordinates(String command);
String getCommand();
StringLengths getLenghtsFromI2C();
void resetEncoders();
void setEncodersTo(long s1, long s2, long s3, long s4);

BoardPosition currentPosition;
StringLengths currentStringLengths;

double module(BoardPosition firstPoint, BoardPosition secondPoint);
BoardPosition* generatePointsBetween(BoardPosition firstPosition, BoardPosition secondPosition, double segmentLength);
BoardPosition gondolaDk(StringLengths stringLengths);
void gondolaGoTo(BoardPosition nextPosition, double speed = 400);
void gondolaGoTo(double x, double y, double speed = 400);
void gondolaGoStraightTo(BoardPosition nextPosition, double segmentLength = 1);
void gondolaGoStraightTo(double x, double y, double segmentLength = 1);

long maxOfFour(long a, long b, long c, long d);
void computeSpeeds(double minSpeed, double maxSpeed, double accel, double speeds[], int quantity);
void configDriver(void);

//=================chess functions=============================
double squareBoardLength = 50;
void run_ajedrez();
void coordinates(char, char, double*, double*);
void infoMovement(char[], char*, char*, char*, char*, char*, int*);
void chess_King(char, char, char, char, int, bool, char);
void chess_Queen(char, char, char, char, int, bool, char);
void chess_Rook(char, char, char, char, int, bool, char);
void chess_Bishop(char, char, char, char, int, bool, char);
void chess_Knight(char, char, char, char, int, bool, char);
void chess_Pawn(char, char, char, char, int, bool, char);
void chessTake(char, char, int , bool);
void chessTakeVersion2(char, char, int , bool, char);
void kingsideCastling(bool);
void queensideCastling(bool);
void centerPiece(void);
void moveChessPiece( char, char, char, char, int, bool);

float segment_interp = 5;

double pre_fin_x = 0;
double pre_fin_y = 0;

// Chess variables

double distanceBetweenCurrentPositionAnd(double, double);


BoardPosition getBoardPositionFromString(String position);

void activateElectromagnet();
void deactivateElectromagnet();
#define LED_PIN     35

//====Calibration====
Calibration calibObj;


void setup()
{
    Serial.begin(115200);
    
    //====Init watchdog====
    esp_task_wdt_init(15,false);

    Wire.begin(SDA_PIN, SCL_PIN, 100000); // join i2c bus
    calibObj.init();
    deactivateElectromagnet();
    delay(1000);
    calibObj.start();

    configDriver();
	delay(5000);

    


    //========================================
    File myFile;
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
   }
 
    myFile = SPIFFS.open("/test.txt", FILE_READ);
 
    if(!myFile){
        Serial.println("There was an error opening the file for writing");
        return;
    }

    /*
    if(myFile.print("1. e2-e4 e7-e6 2. d2-d4 d7-d5 3. Nb1-d2 Ng8-f6 4. e4-e5 Nf6-d7 5. f2-f4 c7-c5 6. c2-c3 Nb8-c6 7. Nd2-f3 c5xd4 8. c3xd4 f7-f6 9. Bf1-d3 Bf8-b4+ 10. Bc1-d2 Qd8-b6 11. Ng1-e2 f6xe5 12. f4xe5 O-O 13. a2-a3 Bb4-e7 14. Qd1-c2 Rf8xf3 15. g2xf3 Nc6xd4 16. Ne2xd4 Qb6xd4 17. O-O-O Nd7xe5 18. Bd3xh7+ Kg8-h8 19. Kc1-b1 Qd4-h4 20. Bd2-c3 Be7-f6 21. f3-f4 Ne5-c4 22. Bc3xf6 Qh4xf6 23. Bh7-d3 b7-b5 24. Qc2-e2 Bc8-d7 25. Rh1-g1 Bd7-e8 26. Rd1-e1 Be8-f7 27. Rg1-g3 Ra8-c8 28. Re1-g1 Nc4-d6 29. Rg3xg7 Nd6-f5 30. Rg7-g5 Rc8-c7 31. Bd3xf5 e6xf5 32. Rg5-h5+ 1-0")){
        Serial.println("File was written");
    } else {
        Serial.println("File write failed");
    }
    */
    
    myFile.close();
    
    currentPosition.x = 0;
    currentPosition.y = 0;
    currentStringLengths = computeStringLengths(currentPosition.x, currentPosition.x);

    stepper1.setCurrentPosition((currentStringLengths.s1 * stepsXrev) / mmXSteps);
    stepper2.setCurrentPosition((currentStringLengths.s2 * stepsXrev) / mmXSteps);
    stepper3.setCurrentPosition((currentStringLengths.s3 * stepsXrev) / mmXSteps);
    stepper4.setCurrentPosition((currentStringLengths.s4 * stepsXrev) / mmXSteps);

    positionOrigin1 = stepper1.currentPosition();
    positionOrigin2 = stepper2.currentPosition();
    positionOrigin3 = stepper3.currentPosition();
    positionOrigin4 = stepper4.currentPosition();

    Serial.print("s1Origin: ");
    Serial.println(positionOrigin1);
    Serial.print("s2Origin: ");
    Serial.println(positionOrigin2);
    Serial.print("s3Origin: ");
    Serial.println(positionOrigin3);
    Serial.print("s4Origin: ");
    Serial.println(positionOrigin4);

     //stepper1.setPinsInverted(true);
    stepper2.setPinsInverted(true);
    stepper3.setPinsInverted(true);
     //stepper4.setPinsInverted(true);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);
    steppers.addStepper(stepper3);
    steppers.addStepper(stepper4);
    Serial.println("....");
    //delay(500);

    setEncodersTo(positionOrigin1, positionOrigin2, positionOrigin3, positionOrigin4);
    Serial.println("encoder were set to origin values");
    // resetEncoders();

    flag = 1;
    
   
    //run_ajedrez();
    
    }

    void loop()
    {
        amountData = readFromSerial();
        if (amountData == 7) {
            Serial.print("number of received commands: ");
            Serial.println(amountData);

            gondolaGoStraightTo(a, b, g);
            gondolaGoStraightTo(c, d, g);
            gondolaGoStraightTo(e, f, g);
            gondolaGoStraightTo(0, 0, g);
        }
        
        if (amountData == 4) {
            Serial.print("number of received commands: ");
            Serial.println(amountData);


            gondolaGoStraightTo(a, b);
            gondolaGoStraightTo(c, d);
        }

        if (amountData == 3) {
            Serial.print("number of received commands: ");
            Serial.println(amountData);

            if(c == 2)
            {
           float xCoordinate;
           float yCoordinate;

           xCoordinate = a;
           yCoordinate = b;

            //======System error position compensation======
            if(a > 0)
            {
                if(pre_fin_x < a)
                {
                    a = a + (a/100);
                }
            }
            if(a < 0)
            {
                if(pre_fin_x > a)
                {
                    a = a + (a/100); 
                }
            }
            if(b > 0)
            {
                if(pre_fin_y < b)
                {
                    b = b + (b/100);
                }
            }
            if(b < 0)
            {
                if(pre_fin_y > b)
                {
                    b = b + (b/100);
                } 
            }
            //===========================================================

            if(a > 0)
            {
                if(pre_fin_x < a)
                {
                    a = a + (pow(1.9,(xCoordinate/offsetx)));    
                }
            }
            if(a < 0)
            {
                if(pre_fin_x > a)
                {
                    a = a - (pow(1.9,(-xCoordinate/offsetx)));
                }
            }
            if(b > 0)
            {
                if(pre_fin_y < b)
                {
                    b = b + (pow(1.9,(yCoordinate/offsety)));
                }
            }
            if(b < 0)
            {
                if(pre_fin_y > b)
                {
                    b = b - (pow(1.9,(-yCoordinate/offsety)));
                }
            }
            //================================================================================================================== 
            }
            Serial.println("Real position");
            Serial.print("x: ");
            Serial.println(a);
            Serial.print("y: ");
            Serial.println(b);

            gondolaGoStraightTo(a, b, 3);
            pre_fin_x = a;
            pre_fin_y = b;
        }

        if (amountData == 2) {
            Serial.print("number of received commands: ");
            Serial.println(amountData);
            long tGoTo = millis();
            gondolaGoTo(a, b);
            Serial.print("time without interpolation: ");
            Serial.println(millis() - tGoTo);
        }

        if (amountData == 1) {

            if (a == 10) {
                flag = 0;
            } else {
                Serial.print("number of received commands: ");
                Serial.println(amountData);
                // tenseRelease(a);

                if (a == 0) {
                    gondolaGoStraightTo(0, 0, 1);
                    flag = 1;
                } else {
                    tenseRelease(a);
                }
            }
        }

        if (amountData == 0 && flag == 0) {

            run_ajedrez();
            gondolaGoStraightTo(0, 0, 5);

            
        }

// #define CustomTrayectory
#ifdef CustomTrayectory
    int delayBetweenMovement = 1000;
    //   Serial.println("s1\ts2\ts3\ts4");

    BoardPosition nextPosition;
    nextPosition.x = 25;
    nextPosition.y = 25;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 100;
    nextPosition.y = -50;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 100;
    nextPosition.y = -100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -50;
    nextPosition.y = -100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -50;
    nextPosition.y = -50;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -100;
    nextPosition.y = -50;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -100;
    nextPosition.y = 100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -50;
    nextPosition.y = 100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -25;
    nextPosition.y = 125;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 0;
    nextPosition.y = 100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 100;
    nextPosition.y = 100;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 100;
    nextPosition.y = 50;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -50;
    nextPosition.y = 50;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -75;
    nextPosition.y = 25;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = -25;
    nextPosition.y = -25;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);

    nextPosition.x = 0;
    nextPosition.y = 0;
    gondolaGoStraightTo(nextPosition);
    delay(delayBetweenMovement);
#endif
}

long tGondola = millis();
void gondolaGoTo(double x, double y, double speed)
{
    #ifdef CommentsForGondolaGoTo
    Serial.print("time= ");
    Serial.println(millis() - tGondola);
    Serial.print("move to: ");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
    #endif
    long positions[4]; //Array of desired stepper positions

    StringLengths stringLengths;

    stringLengths = computeStringLengths(x, y);

    double speedSteps;

    BoardPosition nextPosition;
    nextPosition.x = x;
    nextPosition.y = y;
    double distance = module(currentPosition, nextPosition);
    
    positions[0] = stringLengths.s1 * (double(stepsXrev) / double(mmXSteps)); 
    positions[1] = stringLengths.s2 * (double(stepsXrev) / double(mmXSteps)); 
    positions[2] = stringLengths.s3 * (double(stepsXrev) / double(mmXSteps)); 
    positions[3] = stringLengths.s4 * (double(stepsXrev) / double(mmXSteps)); 

    double maxSteps;
    long    stepsMotorA = positions[0] - stepper1.currentPosition(),
            stepsMotorB = positions[1] - stepper2.currentPosition(),
            stepsMotorC = positions[2] - stepper3.currentPosition(),
            stepsMotorD = positions[3] - stepper4.currentPosition();
    maxSteps = maxOfFour(abs(stepsMotorA), abs(stepsMotorB), abs(stepsMotorC), abs(stepsMotorD));

    // it finishes the function if there is no steps to move
    if (maxSteps == 0){
        return;
    }

    speedSteps = speed * maxSteps/distance;
    if (speedSteps > MaxStepsXSecondAllowed){
        speedSteps = MaxStepsXSecondAllowed;
    }
    #ifdef CommentsForGondolaGoTo
    Serial.print("speed= ");
    Serial.println(speed);
    Serial.print("maxSteps= ");
    Serial.println(maxSteps);
    Serial.print("distance= ");
    Serial.println(distance);
    Serial.print("maxSpeedSteps= ");
    Serial.println(speedSteps);
    #endif

    // set maxspeed to achieve a constant speed
    stepper1.setMaxSpeed(speedSteps);
    stepper2.setMaxSpeed(speedSteps);
    stepper3.setMaxSpeed(speedSteps);
    stepper4.setMaxSpeed(speedSteps);
    
    steppers.moveTo(positions);
    steppers.runSpeedToPosition(); // Blocks until all are in position

#ifdef CommentsForGondolaGoTo
    Serial.print("positions:");
    Serial.print(positions[0]);
    Serial.print(",");
    Serial.print(positions[1]);
    Serial.print(",");
    Serial.print(positions[2]);
    Serial.print(",");
    Serial.println(positions[3]);
#endif

    stringLengths.s1 = positions[0] * double(mmXSteps) / double(stepsXrev);
    stringLengths.s2 = positions[1] * double(mmXSteps) / double(stepsXrev);
    stringLengths.s3 = positions[2] * double(mmXSteps) / double(stepsXrev);
    stringLengths.s4 = positions[3] * double(mmXSteps) / double(stepsXrev);

    currentStringLengths = stringLengths;
    currentPosition = gondolaDk(currentStringLengths);

    #ifdef CommentsForGondolaGoTo
    Serial.print("position x,y: ");
    Serial.print(currentPosition.x);
    Serial.print(",");
    Serial.println(currentPosition.y);
    tGondola = millis();
    #endif
    delay(1);
}

void tenseRelease(float num)
{
    int dirPin = MOTOR_0_DIR_PIN;
    int stepPin = MOTOR_0_STEP_PIN;

    if (num == 3.0 || num == 4.0) {
        dirPin = MOTOR_1_DIR_PIN;
        stepPin = MOTOR_1_STEP_PIN;
    }
    if (num == 5.0 || num == 6.0) {
        dirPin = MOTOR_2_DIR_PIN;
        stepPin = MOTOR_2_STEP_PIN;
    }
    if (num == 7.0 || num == 8.0) {
        dirPin = MOTOR_3_DIR_PIN;
        stepPin = MOTOR_3_STEP_PIN;
    }

    const int steps = 4 * stepsXrev/200;
    int stepDelay;

    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);

    if (num == 2.0 || num == 3.0 || num == 5.0 || num == 8.0) {
        digitalWrite(dirPin, HIGH);
    }
    if (num == 1.0 || num == 4.0 || num == 6.0 || num == 7.0) {
        digitalWrite(dirPin, LOW);
    }

    stepDelay = 250;
    
    for (int x = 0; x < steps; x++) {
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
    }
}

StringLengths computeStringLengths(double x, double y)
{
    StringLengths stringLengths;
    double l1, l2, l3, l4, l5, l6, l7, l8;
    l1 = l2 = l3 = l4 = l5 = l6 = l7 = l8 = d1 / 2.0;
    stringLengths.s1 = sqrt((((BoardLength / 2.0) - l1 + x) * ((BoardLength / 2.0) - l1 + x)) + (((BoardHeight / 2.0) - l8 - y) * ((BoardHeight / 2.0) - l8 - y)));
    stringLengths.s2 = sqrt((((BoardLength / 2.0) - l2 - x) * ((BoardLength / 2.0) - l2 - x)) + (((BoardHeight / 2.0) - l3 - y) * ((BoardHeight / 2.0) - l3 - y)));
    stringLengths.s3 = sqrt((((BoardLength / 2.0) - l6 + x) * ((BoardLength / 2.0) - l6 + x)) + (((BoardHeight / 2.0) - l7 + y) * ((BoardHeight / 2.0) - l7 + y)));
    stringLengths.s4 = sqrt((((BoardLength / 2.0) - l5 - x) * ((BoardLength / 2.0) - l5 - x)) + (((BoardHeight / 2.0) - l4 + y) * ((BoardHeight / 2.0) - l4 + y)));
    return stringLengths;
}

int readFromSerial()
{
    int numData = 0;
    while (Serial.available()) {
        char uartValue = Serial.read();
        if (uartValue == ',' || uartValue == '\n') {
            commands.push(commandRead);
            Serial.print("input command: ");
            Serial.println(commandRead);
            commandRead = "";
            numData += 1;
        } else {
            commandRead.concat(uartValue);
        }
    }
    a = commands.shift().toFloat();
    b = commands.shift().toFloat();
    c = commands.shift().toFloat();
    d = commands.shift().toFloat();
    e = commands.shift().toFloat();
    f = commands.shift().toFloat();
    g = commands.shift().toFloat();

    return numData;
}

StringLengths getLenghtsFromI2C()
{
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("getPosition1");
    Wire.endTransmission(); // stop transmitting
    Wire.requestFrom(I2C_SLAVE_ADDR, 60);
    String receivedData;
    while (Wire.available()) { // slave may send less than requested
        char c = Wire.read(); // receive a byte as character
        receivedData.concat(c);
    }
    int dotIndex = receivedData.indexOf('\n');
    String position1 = receivedData.substring(0, dotIndex);

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("getPosition2");
    Wire.endTransmission(); // stop transmitting
    Wire.requestFrom(I2C_SLAVE_ADDR, 60); 
    receivedData = "";
    while (Wire.available()) { // slave may send less than requested
        char c = Wire.read(); // receive a byte as character
        receivedData.concat(c);
    }
    dotIndex = receivedData.indexOf('\n');
    String position2 = receivedData.substring(0, dotIndex);

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("getPosition3");
    Wire.endTransmission(); // stop transmitting
    Wire.requestFrom(I2C_SLAVE_ADDR, 60); 
    receivedData = "";
    while (Wire.available()) { // slave may send less than requested
        char c = Wire.read(); // receive a byte as character
        receivedData.concat(c);
    }
    dotIndex = receivedData.indexOf('\n');
    String position3 = receivedData.substring(0, dotIndex);

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("getPosition4");
    Wire.endTransmission(); // stop transmitting
    Wire.requestFrom(I2C_SLAVE_ADDR, 60); 
    receivedData = "";
    while (Wire.available()) { // slave may send less than requested
        char c = Wire.read(); // receive a byte as character
        receivedData.concat(c);
    }
    dotIndex = receivedData.indexOf('\n');
    
    String position4 = receivedData.substring(0, dotIndex);

    String command = position1 + "," + position2 + "," + position3 + "," + position4;

    StringLengths lengths;
    lengths.s1 = position1.toDouble();
    lengths.s2 = position2.toDouble();
    lengths.s3 = position3.toDouble();
    lengths.s4 = position4.toDouble();

    return lengths;
}

BoardPosition commandToCoordinates(String command)
{
    BoardPosition boardPosition;
    int dotIndex = command.indexOf(',');

    String x = command.substring(0, dotIndex);
    String y = command.substring(dotIndex + 1);

    Serial.print("x= ");
    Serial.println(x);
    Serial.print("y= ");
    Serial.println(y);

    boardPosition.x = x.toDouble();
    boardPosition.y = y.toDouble();

    return boardPosition;
}

void resetEncoders()
{
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("reset");
    Wire.endTransmission(); // stop transmitting
}

void setEncodersTo(long s1, long s2, long s3, long s4)
{
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("s1=");
    Wire.write(String(s1).c_str());
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("s2=");
    Wire.write(String(s2).c_str());
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("s3=");
    Wire.write(String(s3).c_str());
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(I2C_SLAVE_ADDR);
    Wire.write("s4=");
    Wire.write(String(s4).c_str());
    Wire.endTransmission(); // stop transmitting
}

void gondolaGoStraightTo(double x, double y, double segmentLength)
{
    BoardPosition nextPosition;
    nextPosition.x = x;
    nextPosition.y = y;
    gondolaGoStraightTo(nextPosition, segmentLength);
}

void gondolaGoStraightTo(BoardPosition nextPosition, double segmentLength)
{
    // <compute segment>
    BoardPosition firstPosition, secondPosition;
    firstPosition = currentPosition;

    secondPosition = nextPosition;
    double alpha = atan2(secondPosition.y - firstPosition.y, secondPosition.x - firstPosition.x);

    double deltaX, deltaY;

    double length = module(firstPosition, secondPosition);

    deltaX = segmentLength * cos(alpha);
    deltaY = segmentLength * sin(alpha);

    // we increase the number of segments by 2 because final and star position are included in the array
    int numberOfSegments = length / segmentLength + 2;
    //BoardPosition boardPosition[numberOfSegments];
    BoardPosition * boardPosition = new BoardPosition[numberOfSegments];
    boardPosition[0] = firstPosition;
    // the for cycle stops in the second-last position of the boardPosition array because the last one is reserved to the last point of the interpolation
    for (int i = 1; i < numberOfSegments - 1; i++) {
        boardPosition[i].x = boardPosition[i - 1].x + deltaX;
        boardPosition[i].y = boardPosition[i - 1].y + deltaY;
    }
    // the last point is assigned to the last position of the array
    boardPosition[numberOfSegments - 1].x = secondPosition.x;
    boardPosition[numberOfSegments - 1].y = secondPosition.y;
    // </compute segment>

    double * speeds = new double[numberOfSegments];
    double minSpeed = 5, maxSpeed = MaxSpeedMMxSec, accel = 5;
    computeSpeeds(minSpeed,maxSpeed, accel, speeds, numberOfSegments);

    long tInterpolate = millis();
    for (int segmentNumber = 0; segmentNumber < numberOfSegments; segmentNumber++) {
        gondolaGoTo(boardPosition[segmentNumber], speeds[segmentNumber]);
    }
    Serial.print("interpolation time= ");
    Serial.println(millis() - tInterpolate);

    delete[] boardPosition;
    delete[] speeds;
}

void gondolaGoTo(BoardPosition nextPosition, double speed)
{
    gondolaGoTo(nextPosition.x, nextPosition.y, speed);
}

BoardPosition gondolaDk(StringLengths stringLengths)
{
    BoardPosition boardPosition;
    boardPosition.x = (pow(stringLengths.s2, 2) - pow(currentStringLengths.s1, 2) + 2.0 * BoardLength * d1 - pow(d1, 2) - pow(BoardLength, 2)) / double(2.0 * (d1 - BoardLength)) + d1 / 2.0 - BoardLength / 2.0;
    boardPosition.y = (pow(stringLengths.s2, 2) - pow(currentStringLengths.s4, 2) + 2.0 * BoardHeight * d2 - pow(d2, 2) - pow(BoardHeight, 2)) / double(2.0 * (d2 - BoardHeight)) + d2 / 2.0 - BoardHeight / 2.0;
    return boardPosition;
}

double module(BoardPosition firstPoint, BoardPosition secondPoint)
{
    return sqrt(pow(secondPoint.x - firstPoint.x, 2) + pow(secondPoint.y - firstPoint.y, 2));
}

long maxOfFour(long a, long b, long c, long d){
    long max = a;
    if (b > max){
        max = b;
    }
    if (c > max){
        max = c;
    }
    if (d > max){
        max = d;
    }

    return max;
}

void computeSpeeds(double minSpeed, double maxSpeed, double accel, double speeds[], int quantity){
    speeds[0] = minSpeed;

    for (int i = 1; i < quantity; i++){
        speeds[i] = speeds[i - 1] + accel;
        if (speeds[i] > maxSpeed){
            speeds[i] = maxSpeed;
        }
    }
    speeds[quantity - 1] = minSpeed;
    for (int i = quantity - 2; i > 0; i --){
        double newSpeed = speeds[i + 1] + accel;

        if (newSpeed < speeds[i]){
            speeds[i] = newSpeed;
        } else {
            break;
        }
    }
}

void configDriver(void)
{
    SERIAL_PORT2.begin(115200);

    pinMode(MOTOR_0_STEP_PIN, OUTPUT);
    pinMode(MOTOR_0_DIR_PIN, OUTPUT);
    pinMode(MOTOR_1_STEP_PIN, OUTPUT);
    pinMode(MOTOR_1_DIR_PIN, OUTPUT);

    digitalWrite(MOTOR_0_DIR_PIN, HIGH);
    digitalWrite(MOTOR_1_DIR_PIN, HIGH);

    //===========Motor 3 y 4============
    pinMode(MOTOR_2_STEP_PIN, OUTPUT);
    pinMode(MOTOR_2_DIR_PIN, OUTPUT);

    pinMode(MOTOR_3_STEP_PIN, OUTPUT);
    pinMode(MOTOR_3_DIR_PIN, OUTPUT);

    digitalWrite(MOTOR_2_DIR_PIN, HIGH);

    digitalWrite(MOTOR_3_DIR_PIN, HIGH);

    //===========Motor 1 y 2============
    driver.begin();
    driver.pdn_disable(true); // enables the  PDN/UART comunication.
    driver.toff(4); 
    driver.blank_time(24);
    driver.rms_current(CURRENT_IN_CALIBRATION); // set the current value in miliamps
    driver.microsteps(MICROSTEPPING); // it set the microsteps value
    driver.TCOOLTHRS(0xFFFFF); // inferior speed threshold to turn on the Coolstep and stallGuard smart energy to the output of DIAG
    driver.semin(0); // inferior Coolstep threshold [0 ... 15].
        // if SG_RESULT is less than this threshold, coolStep increase the current for both coils.
        // 0: disables CoolStep
    driver.shaft(false); // it set the direction of the motor via UART
    driver.sedn(0b01); 
    driver.SGTHRS(STALL_VALUE); 

    driver2.begin();
    driver2.pdn_disable(true); 
    driver2.toff(4); 
    driver2.blank_time(24);
    driver2.rms_current(CURRENT_IN_CALIBRATION); 
    driver2.microsteps(MICROSTEPPING); 
    driver2.TCOOLTHRS(0xFFFFF); 
    driver2.semin(0); 
    
    driver2.shaft(false); 
    driver2.sedn(0b01); 
    driver2.SGTHRS(STALL_VALUE2); 
    //===========Motor 3 y 4============
    driver3.begin();
    driver3.pdn_disable(true); 
    driver3.toff(4);
    driver3.blank_time(24);
    driver3.rms_current(CURRENT_IN_CALIBRATION);
    driver3.microsteps(MICROSTEPPING);
    driver3.TCOOLTHRS(0xFFFFF); 
    driver3.semin(0); 
        
    driver3.shaft(false); 
    driver3.sedn(0b01); 
    driver3.SGTHRS(STALL_VALUE3); 
    driver4.begin();
    driver4.pdn_disable(true); 
    driver4.toff(4);
    driver4.blank_time(24);
    driver4.rms_current(CURRENT_IN_CALIBRATION);
    driver4.microsteps(MICROSTEPPING);
    driver4.TCOOLTHRS(0xFFFFF); 
    driver4.semin(0); 
    
    driver4.shaft(false); 
    driver4.sedn(0b01); 
    driver4.SGTHRS(STALL_VALUE4); 
    driver.ihold(19);
    driver2.ihold(19); 
    driver3.ihold(19); 
    driver4.ihold(19); 
    
    delay(1000);
    int cont = 0;
    while (driver.microsteps() != MICROSTEPPING) {
        driver.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver2.microsteps() != MICROSTEPPING) {
        driver2.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver3.microsteps() != MICROSTEPPING) {
        driver3.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
    cont = 0;
    while (driver4.microsteps() != MICROSTEPPING) {
        driver4.microsteps(MICROSTEPPING);
        delay(100);
        cont++;
        if (cont == 5) {
            break;
        }
    }
}

void run_ajedrez()
{
    char char_x_ini, char_y_ini, char_x_fin, char_y_fin, char_pieza;
    int int_accion = 0;
    char* Ap_y_ini = &char_y_ini;
    char* Ap_x_ini = &char_x_ini;
    char* Ap_y_fin = &char_y_fin;
    char* Ap_x_fin = &char_x_fin;
    char* Ap_pieza = &char_pieza;
    int* Ap_accion = &int_accion;
    char mov_chess[7] = { 'v', 'v', 'v', 'v', 'v', 'v', 'v' };
    bool chessColor = false;
    char currentChar = 'v';
    int index_mov = 0;
    Serial.println("In chess function");
    
    //====run program===============
    int band_vec = 0;
    
    File myFileChess;
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
   }
    
    myFileChess = SPIFFS.open("/test.txt");

    Serial.println("File Content:");
 
    while(myFileChess.available()){
        if (index_mov == 0) {
                currentChar = (myFileChess.read());
        }
        if(currentChar == ' ') {
                index_mov = 0;
                do {
                    if (myFileChess.available()) {
                        currentChar = (myFileChess.read());
                    } else {
                        break;
                    }

                    if (currentChar != ' ') {
                        mov_chess[index_mov] = currentChar;
                        index_mov++;
                    }
                    
                } while (currentChar != ' ');

                for (int i = 0; i < 7; i++) 
                {
                    if (mov_chess[i] == '.') {
                        band_vec = 1;
                        for (int j = 0; j < 7; j++) {
                            mov_chess[j] = 'v'; 
                        }
                    }
                }
                //============Check if there is any value in serial buffer to go home==========
                for(int i = 0; i < 1000; i++)
                    {
                        amountData = readFromSerial();
                        delay(2);                  
                        if(amountData == 1)
                        {
                        Serial.print("commands received: ");
                        Serial.println(amountData);
                        if (a == 10)
                        {
                            gondolaGoStraightTo(0, 0, 5);
                        }
                        delay(10000);
                }
                    }
                // =======================================================================
                Serial.println(" ");
                if (band_vec == 0) {
                    
                    cont_m++;

                    chessColor = !chessColor;
                    infoMovement(mov_chess, Ap_x_ini, Ap_y_ini, Ap_x_fin, Ap_y_fin, Ap_pieza, Ap_accion);
                    for (int i = 0; i < 7; i++) {
                        mov_chess[i] = 'v';
                    }
                    Serial.println("movement ");
                    Serial.println(chessColor);
                    Serial.println(int_accion);
                    switch (int_accion) {
                    case 0:
                        Serial.println(" ");
                        break;
                    case 1:
                        Serial.println("only moves");
                        break;
                    case 2:
                        Serial.println("it moves and promotes");
                        break;   
                    case 3:
                        Serial.println("it moves and eats");
                        break;
                    case 4:
                        Serial.println("it moves and check");
                        break;
                    case 5:
                        Serial.println("it moves, promotes and check");
                        break;    
                    case 6:
                        Serial.println("it moves, eats, and check");
                        break;
                    case 7:
                        Serial.println("kingside castking");
                        break;
                    case 8:
                        Serial.println("queenside castling");
                        break;
                    default:
                        Serial.println("invalid movement");
                        break;
                    }

                    switch (char_pieza) {
                    case 'K':
                        Serial.println("King");
                        chess_King(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        break;
                    case 'Q':
                        Serial.println("Queen");
                        chess_Queen(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        break;
                    case 'R':
                        Serial.println("Rook");
                        chess_Rook(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        break;
                    case 'B':
                        Serial.println("Bishop");
                        chess_Bishop(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        break;
                    case 'N':
                        Serial.println("Knight");
                        chess_Knight(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        break;
                    case 'P':
                        if(int_accion != 0)
                        {
                            Serial.println("Pawn moves");
                            chess_Pawn(char_x_ini, char_y_ini, char_x_fin, char_y_fin, int_accion, chessColor, char_pieza);
                        }
                        break;
                    case 'S':
                        kingsideCastling(chessColor);
                        break;
                    case 'L':
                        queensideCastling(chessColor);
                        break;
                    default:
                        Serial.println("invalid piece");
                        break;
                    }

                    Serial.print(char_x_ini);
                    Serial.println(char_y_ini);
                    Serial.print(char_x_fin);
                    Serial.println(char_y_fin);
                    int_accion = 0;
                    Serial.print("number of movements: ");
                    Serial.println(cont_m);
                }
                band_vec = 0;
                
            }
    }
    
    myFileChess.close();
    

}

//=========================symbols description==============================
//    Chess Pieces
//    King: the letter K
//    Queen: the letter Q
//    Rook: the letter R
//    Bishop: the letter B
//    Knight: the letter N
//    Pawn: no letter assigned

//    Actions
//    - : moves piece
//   ' ': moves piece
//    x : eats piece
//    + : check
//    o : castling

//    acc
//    1 : only moves
//    2 : it moves and promotion
//    3 : it moves and eats
//    4 : it moves and checks
//    5 : it moves, promotes and check
//    6 : it moves, eats and check
//    7 : kingside castling
//    8 : queenside castling
//    9 : Passant
//   10 : it moves, eat and promotes

void infoMovement(char v[7], char* ini_c1, char* ini_c2, char* fin_c1, char* fin_c2, char* chessPiece, int* acc)
{
    char v_piece[5] = { 'K', 'Q', 'R', 'B', 'N' }; 
    char v_action[5] = { '-', ' ', 'x', '+', 'O' };
    char ini_end[4] = { 'V', 'V', 'V', 'V' };
    char action;
    int pawn = 1;
    int takes = 0;
    int check = 0;
    int castling = 5;
    int move = 0;
    int promotion = 0;
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 5; j++) {
            if (v[i] == v_piece[j]) {
                if(i != 0){
                    pawn = 1;
                    promotion = 1;
                }
                else{
                    pawn = 0;
                }
                *chessPiece = v_piece[j];
                v[i] = 'v';
            }
            if (v[i] == v_action[j]) {
                action = v_action[j];
                v[i] = 'v';
                if (action == '-' || action == ' ') {
                    move = 1;
                }
                if (action == 'x') {
                    takes = 3;
                }
                if (action == '+') {
                    check = 3;
                }
                if (action == 'O') {
                    castling++;
                }
            }
        }
        if (pawn == 1) {
            *chessPiece = 'P';
        }
    }
    if (castling == 7) {
        *chessPiece = 'S'; //kingside castling
    }
    if (castling == 8) {
        *chessPiece = 'L'; //queenside casatling
    }
    int k = 0;
    for (int i = 0; i < 7; i++) {
        if (v[i] != 'v') {
            ini_end[k] = v[i];
            k++;
        }
    }
    *ini_c1 = ini_end[0];
    *ini_c2 = ini_end[1];
    *fin_c1 = ini_end[2];
    *fin_c2 = ini_end[3];

    Serial.println("Vector ini_end");
    Serial.println(ini_end[0]);
    Serial.println(ini_end[1]);
    Serial.println("\n");
    Serial.println(ini_end[2]);
    Serial.println(ini_end[3]);


    if (castling == 5) {
        *acc = takes + check + move + promotion;
    } else {
        *acc = castling;
    }


    if(takes == 3 && promotion == 1)
    {
        *acc = 10;
    }

    if(ini_end[0] == '1' && ini_end[1] == '0'){
        Serial.println("white pieces win");
        *acc = 0;
    }
    if(ini_end[0] == '0' && ini_end[1] == '1'){
        Serial.println("black pieces win");
        *acc = 0;
    }
    if(ini_end[0] == '1' && ini_end[1] == '2'){
        Serial.println("draw");
        *acc = 0;
    }
    if(ini_end[1] == 'e' && ini_end[2] == 'p'){
        Serial.println("Passant");
        *acc = 9;
    }
}

void moveChessPiece(char x_ini, char y_ini, char x_fin, char y_fin, int movement, bool chess_color, char chessPiece)
{
    double comp_ini_x, comp_ini_y;
    double* Ap_ini_x = &comp_ini_x;
    double* Ap_ini_y = &comp_ini_y;
    double comp_fin_x, comp_fin_y;
    double* Ap_fin_x = &comp_fin_x;
    double* Ap_fin_y = &comp_fin_y;
    
    if (movement == 3 || movement == 6 || movement == 9 || movement == 10) {
        chessTake(x_fin, y_fin, movement, chess_color);
    }


    coordinates(x_ini, y_ini, Ap_ini_x, Ap_ini_y);
    gondolaGoStraightTo(comp_ini_x, comp_ini_y,segment_interp);
    activateElectromagnet();

    coordinates(x_fin, y_fin, Ap_fin_x, Ap_fin_y);
    
    gondolaGoStraightTo(comp_fin_x, comp_fin_y,segment_interp);
    
    deactivateElectromagnet();

    //=======condition for a pawm promotion======
    if(movement == 2 || movement == 5 || movement == 10)
    {
        chessTake(x_fin, y_fin, movement, chess_color);
    }
    //===========================================================
}

void activateElectromagnet()
{
    for(int i = 0; i < 10; i++)
    {
        // first create a WirePacker that will assemble a packet
        WirePacker packer;

        // then add data the same way as you would with Wire
        packer.write("Z1");
        // packer.write(x);

        // after adding all data you want to send, close the packet
        packer.end();

        // now transmit the packed data
        Wire.beginTransmission(I2C_ESP32_ADDR);
        while (packer.available()) { // write every packet byte
            Wire.write(packer.read());
        }
        Wire.endTransmission(); // stop transmitting
        delay(50);
    }
}

void deactivateElectromagnet()
{
    for(int i = 0; i < 10; i++)
    {
        // first create a WirePacker that will assemble a packet
        WirePacker packer;

        // then add data the same way as you would with Wire
        packer.write("Z0");
        // packer.write(x);

        // after adding all data you want to send, close the packet
        packer.end();

        // now transmit the packed data
        Wire.beginTransmission(I2C_ESP32_ADDR);
        while (packer.available()) { // write every packet byte
            Wire.write(packer.read());
        }
        Wire.endTransmission(); // stop transmitting
        delay(50);
    }
    
}

void chess_King(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color, char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chess_Queen(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color, char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chess_Rook(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color,char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chess_Bishop(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color, char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chess_Knight(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color, char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chess_Pawn(char xStart, char yStart, char xEnd, char yEnd, int movement, bool chess_color, char chessPiece)
{
    moveChessPiece(xStart, yStart, xEnd, yEnd, movement, chess_color, chessPiece);
}

void chessTake(char xEnd, char yEnd,int movement, bool chess_color)
{
    //first we move to the taken piece
    BoardPosition boardPosition;
    double x[5];
    double y[5];

    coordinates(xEnd, yEnd, &boardPosition.x, &boardPosition.y);
    //====condition in the passant case===//
    if(movement == 9)
    {
        if(chess_color == 1) //for white pieces
        {
            boardPosition.x  = boardPosition.x + 50;

        }
        if(chess_color == 0) //for black pieces
        {
            boardPosition.x  = boardPosition.x - 50;
        }
    }
    //==================================================================
    gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
    activateElectromagnet();

    //we moves the piece to the nearest inferior line
    if (chess_color == true) {
        boardPosition.y = boardPosition.y + (0.5 * squareBoardLength);
    } else {
        boardPosition.y = boardPosition.y - (0.5 * squareBoardLength);
    }
    x[0] = boardPosition.x;
    y[0] = boardPosition.y;
    gondolaGoStraightTo(x[0],y[0],segment_interp);

    if (chess_color == true) {
        boardPosition.x = 4.0 * squareBoardLength;    
    } else {
        boardPosition.x = -4.0 * squareBoardLength;  
    }
    x[1] = boardPosition.x;
    y[1] = boardPosition.y;
    gondolaGoStraightTo(x[1],y[1],segment_interp);
    //moveThroughThisPoints(x, y, 2);
    deactivateElectromagnet();
}

void kingsideCastling(bool chess_color)
{
    double x[5];
    double y[5];

    BoardPosition boardPosition;

    if (chess_color == true) //if it's white's turn
    {
        boardPosition = getBoardPositionFromString("e1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition = getBoardPositionFromString("g1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        deactivateElectromagnet();

        boardPosition = getBoardPositionFromString("h1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        //we move the castle to the inferior edge of the board
        boardPosition.x = 3.5 * squareBoardLength;
        boardPosition.y = -4.0 * squareBoardLength;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        gondolaGoStraightTo(x[0],y[0],segment_interp);
        delay(1000);

        boardPosition.x = 1.5 * squareBoardLength;
        boardPosition.y = -4.0 * squareBoardLength;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        gondolaGoStraightTo(x[1],y[1],segment_interp);
        delay(1000);

        boardPosition = getBoardPositionFromString("f1");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;

        gondolaGoStraightTo(x[2],y[2],segment_interp);
        deactivateElectromagnet();
    } else //this condition is for the black's turn
    {
        boardPosition = getBoardPositionFromString("e8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition = getBoardPositionFromString("g8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        deactivateElectromagnet();

        boardPosition = getBoardPositionFromString("h8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition.x = 3.5 * squareBoardLength;
        boardPosition.y = 4.0 * squareBoardLength;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        gondolaGoStraightTo(x[0],y[0],segment_interp);
        delay(1000);

        boardPosition.x = 1.5 * squareBoardLength;
        boardPosition.y = 4.0 * squareBoardLength;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        gondolaGoStraightTo(x[1],y[1],segment_interp);
        delay(1000);

        boardPosition = getBoardPositionFromString("f8");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        gondolaGoStraightTo(x[2],y[2],segment_interp);
        deactivateElectromagnet();
    }
}

void queensideCastling(bool chess_color)
{
    double x[5];
    double y[5];
    BoardPosition boardPosition;

    if (chess_color == true) //this is for the white's turn
    {
        boardPosition = getBoardPositionFromString("e1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition = getBoardPositionFromString("c1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        deactivateElectromagnet();

        boardPosition = getBoardPositionFromString("a1");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition.x = -3.5 * squareBoardLength;
        boardPosition.y = -4.0 * squareBoardLength;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        gondolaGoStraightTo(x[0],y[0],segment_interp);
        delay(1000);

        boardPosition.x = -0.5 * squareBoardLength;
        boardPosition.y = -4.0 * squareBoardLength;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        gondolaGoStraightTo(x[1],y[1],segment_interp);
        delay(1000);

        boardPosition = getBoardPositionFromString("d1");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        gondolaGoStraightTo(x[2],y[2],segment_interp);
        deactivateElectromagnet();
    } else //this is for the black's turn
    {
        boardPosition = getBoardPositionFromString("e8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition = getBoardPositionFromString("c8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        deactivateElectromagnet();

        boardPosition = getBoardPositionFromString("a8");
        gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
        activateElectromagnet();

        boardPosition.x = -3.5 * squareBoardLength;
        boardPosition.y = 4.0 * squareBoardLength;
        x[0] = boardPosition.x;
        y[0] = boardPosition.y;
        gondolaGoStraightTo(x[0],y[0],segment_interp);
        delay(1000);

        boardPosition.x = -0.5 * squareBoardLength;
        boardPosition.y = 4.0 * squareBoardLength;
        x[1] = boardPosition.x;
        y[1] = boardPosition.y;
        gondolaGoStraightTo(x[1],y[1],segment_interp);
        delay(1000);

        boardPosition = getBoardPositionFromString("d8");
        x[2] = boardPosition.x;
        y[2] = boardPosition.y;
        gondolaGoStraightTo(x[2],y[2],segment_interp);

        deactivateElectromagnet();
    }
}

BoardPosition getBoardPositionFromString(String position)
{
    BoardPosition boardPosition;
    position.toLowerCase();
    if (position.length() != 2) {
        boardPosition.x = 0;
        boardPosition.y = 0;
        Serial.println("getBoardPositionFromString Error");
        return boardPosition;
    }
    char letterPosition = position.charAt(0);
    char numberPosition = position.charAt(1);
    coordinates(letterPosition, numberPosition, &boardPosition.x, &boardPosition.y);
    return boardPosition;
}

void coordinates(char caract_1, char caract_2, double* c_1, double* c_2)
{
    if (caract_1 == 'a') {
        *c_1 = -3.5 * squareBoardLength;
    }
    if (caract_1 == 'b') {
        *c_1 = -2.5 * squareBoardLength;
    }
    if (caract_1 == 'c') {
        *c_1 = -1.5 * squareBoardLength;
    }
    if (caract_1 == 'd') {
        *c_1 = -0.5 * squareBoardLength;
    }
    if (caract_1 == 'e') {
        *c_1 = 0.5 * squareBoardLength;
    }
    if (caract_1 == 'f') {
        *c_1 = 1.5 * squareBoardLength;
    }
    if (caract_1 == 'g') {
        *c_1 = 2.5 * squareBoardLength;
    }
    if (caract_1 == 'h') {
        *c_1 = 3.5 * squareBoardLength;
    }
    if (caract_2 == '1') {
        *c_2 = -3.5 * squareBoardLength;
    }
    if (caract_2 == '2') {
        *c_2 = -2.5 * squareBoardLength;
    }
    if (caract_2 == '3') {
        *c_2 = -1.5 * squareBoardLength;
    }
    if (caract_2 == '4') {
        *c_2 = -0.5 * squareBoardLength;
    }
    if (caract_2 == '5') {
        *c_2 = 0.5 * squareBoardLength;
    }
    if (caract_2 == '6') {
        *c_2 = 1.5 * squareBoardLength;
    }
    if (caract_2 == '7') {
        *c_2 = 2.5 * squareBoardLength;
    }
    if (caract_2 == '8') {
        *c_2 = 3.5 * squareBoardLength;
    }
}

void centerPiece()
{
    BoardPosition boardPosition;
    double ren = -3.5;
    double col = -3.5;

    for (int r = 0; r < 2; r++)
    {
        for(int c = 0; c < 8; c++)
        {
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            activateElectromagnet();

            ren = ren - 0.5;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            delay(500);

            ren = ren + 0.4;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            deactivateElectromagnet();

            ren = ren + 0.1;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            activateElectromagnet();
            deactivateElectromagnet();

            col = col + 1.0;
        }
        col = -3.5;
        ren = ren + 1.0;
    }

    
    ren = 2.5;
    col = -3.5;
    for (int r = 0; r < 2; r++)
    {
        for(int c = 0; c < 8; c++)
        {
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            activateElectromagnet();

            ren = ren - 0.5;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            delay(500);

            ren = ren + 0.4;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            deactivateElectromagnet();

            ren = ren + 0.1;
            boardPosition.x = col * squareBoardLength;
            boardPosition.y = ren * squareBoardLength;
            gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
            activateElectromagnet();
            deactivateElectromagnet();

            col = col + 1.0;
        }
        col = -3.5;
        ren = ren + 1.0;
    }
}


void chessTakeVersion2(char xEnd, char yEnd,int movement, bool chess_color, char pieza)
{
    BoardPosition boardPosition;
    double x[5];
    double y[5];

    coordinates(xEnd, yEnd, &boardPosition.x, &boardPosition.y);
    //====condition for passsant movement===//
    if(movement == 9)
    {
        if(chess_color == 1) 
        {
            boardPosition.x  = boardPosition.x + 50;

        }
        if(chess_color == 0) 
        {
            boardPosition.x  = boardPosition.x - 50;
        }
    }
    //==================================================================
    gondolaGoStraightTo(boardPosition.x, boardPosition.y,segment_interp);
    activateElectromagnet();



    if (chess_color == true) {
        if(indexB < 8) //if the lateral escaques are not full yet
        {
            boardPosition.y = boardPosition.y + (0.5 * squareBoardLength); 
        }
        else
        {
            boardPosition.x = boardPosition.x + (0.5 * squareBoardLength); 
        }
        
    } else {
        if(indexN < 8) //if the lateral escaques are not full yet
        {
            boardPosition.y = boardPosition.y - (0.5 * squareBoardLength);
        }
        else
        {
            boardPosition.x = boardPosition.x - (0.5 * squareBoardLength);
        }
        
    }
    x[0] = boardPosition.x;
    y[0] = boardPosition.y;
    gondolaGoStraightTo(x[0],y[0],segment_interp);



    if (chess_color == true) {
        if(indexB < 8 )
        {
            boardPosition.x = 4.0 * squareBoardLength;    
        }
        else
        {
            boardPosition.y = -4.0 * squareBoardLength;  
        }
        
    } else {
        if(indexN < 8 )
        {
            boardPosition.x = -4.0 * squareBoardLength; 
        }
        else
        {
            boardPosition.y = 4.0 * squareBoardLength;  
        }
        
    }
    x[1] = boardPosition.x;
    y[1] = boardPosition.y;
    gondolaGoStraightTo(x[1],y[1],segment_interp);
    
    if (chess_color == true) {
        boardPosition.x = xB * squareBoardLength; 
        boardPosition.y = yB * squareBoardLength; 

        whiteVector[indexB] = pieza;
        indexB++;
        if(indexB < 8 )
        {
            xB = 4.0;
            yB = yB - 1;
        }
        else
        {
            xB = xB2 - 1;
            xB2 = xB2 - 1;
            yB = -4.0;
        }
        
    } else {
        boardPosition.x = xN * squareBoardLength;  
        boardPosition.y = yN * squareBoardLength;  
        blackVector[indexN] = pieza;
        indexN++;
        if(indexN < 8 )
        {
            xN = -4.0;
            yN = yN + 1;
        }
        else
        {
            xN = xN2 + 1;
            xN2 = xN2 + 1;
            yN = 4.0;
        }
        
    }
    x[1] = boardPosition.x;
    y[1] = boardPosition.y;
    gondolaGoStraightTo(x[1],y[1],segment_interp);
    //=====================================================================================================================

    if (chess_color == true) {
        if((indexB - 1) < 8 )
        {
            xB = 4.5;
        }
        else
        {
            yB = -4.5;
        }
        boardPosition.x = xB * squareBoardLength;    
        boardPosition.y = yB * squareBoardLength;    
        
    } else {
        if((indexN -1) < 8 )
        {
            xN = -4.5;
        }
        else
        {
            yN = 4.5;
        }
        boardPosition.x = xN * squareBoardLength;
        boardPosition.y = yN * squareBoardLength; 
        
    }
    x[1] = boardPosition.x;
    y[1] = boardPosition.y;
    gondolaGoStraightTo(x[1],y[1],segment_interp);
    //=====================================================================================================================

    deactivateElectromagnet();
}