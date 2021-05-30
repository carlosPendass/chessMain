
//pins of Motor 1
#define MOTOR_0_DIR_PIN (14)
#define MOTOR_0_STEP_PIN (25)
//pins of Motor 2
#define MOTOR_1_DIR_PIN (32) 
#define MOTOR_1_STEP_PIN (33)
//pins of Motor 3
#define MOTOR_2_DIR_PIN (18)   
#define MOTOR_2_STEP_PIN (13)  
//pins of Motor 4
#define MOTOR_3_DIR_PIN (21)    
#define MOTOR_3_STEP_PIN (19)   

#define ENABLE_PIN 17

#define hall1 39
#define hall2 36

// variables for I2C

#define SDA_PIN 22
#define SCL_PIN 23
#define I2C_SLAVE_ADDR 0x05
#define I2C_ESP32_ADDR 0x04
#define I2C_ARDUINO_ADDR 0x06

// 124 is the highest value
#define MAX_SLAVE_RESPONSE_LENGTH 124

//===========================================

//this is the value resistence of the the dc motor drivers
#define R_SENSE 0.11f

#define STALL_VALUE 20
#define STALL_VALUE2 20 
#define STALL_VALUE3 20 
#define STALL_VALUE4 20 

// this is used to set the microstepping value in the drivers
#define MICROSTEPPING 16

#define NORMAL_CURRENT 1200 
#define CURRENT_IN_CALIBRATION 1200 

#define SERIAL_PORT2 Serial1 
#define DRIVER_ADDRESS1 0b00 
#define DRIVER_ADDRESS2 0b01 
#define DRIVER_ADDRESS3 0b10 
#define DRIVER_ADDRESS4 0b11 