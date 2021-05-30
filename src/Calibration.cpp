#include "Calibration.h"

hw_timer_t *timer1 = NULL;

MeanFilter<long> meanFilter(5);
MeanFilter<long> meanFilter2(40);
MeanFilter<long> meanFilter3(5);
MeanFilter<long> meanFilter4(10);
volatile unsigned long flag_timer = 0;
int avoid = 0;
int avoid2 = 0;
int A = 0;
int B = 0;
int p = 0;
int motor_degrees = 0;
int Speed = 5000;
int value[1200];
int value_r[1200];
int vect_prom[5];
int vect_simi[5];
int vect_max[5];
int vect_min[5];
int vect_prom2[5];
int vect_simi2[5];
int vect_max2[5];
int vect_min2[5];
int maximum = 0;
int maximum2 = 0;
int minimum = 5000;
int minimum2 = 5000;
int max_hall1;
int min_hall1 = 1700;  
int level_zero1 = 1700;
int level_zero2 = 1700;
int min_hall2 = 1700;  
int max_hall2;
int read_hall2;
int read_hall1;
int Pole_sens1;
int Pole_sens2;
int Flag_adjust_ini = 0;

int stall_out = 0;
int *Ap_stall = &stall_out;

int ProductTypeChess;

TMC2209Stepper driver(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS1);
TMC2209Stepper driver2(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS2);
TMC2209Stepper driver3(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS3);
TMC2209Stepper driver4(&SERIAL_PORT2, R_SENSE, DRIVER_ADDRESS4);

void normal_turn();
void move(int, int, int);
void slow_Calibration_hall1_negative(void);
void slow_Calibration_hall2_negative(void);
int Check_ini(void);
void check_stall(int, int *);
void IRAM_ATTR onTimer();
void testMotors(void);
void configDrivers(void);

Calibration::Calibration()
{
}


void Calibration::init()
{
	SERIAL_PORT2.begin(115200);
    //===========Motor 1 y 2============
	pinMode(MOTOR_0_STEP_PIN, OUTPUT);
	pinMode(MOTOR_0_DIR_PIN, OUTPUT);

	pinMode(MOTOR_1_STEP_PIN, OUTPUT);
	pinMode(MOTOR_1_DIR_PIN, OUTPUT);

	digitalWrite(MOTOR_0_DIR_PIN, LOW);

	digitalWrite(MOTOR_1_DIR_PIN, LOW);

    //===========Motor 3 y 4============
	pinMode(MOTOR_2_STEP_PIN, OUTPUT);
	pinMode(MOTOR_2_DIR_PIN, OUTPUT);

	pinMode(MOTOR_3_STEP_PIN, OUTPUT);
	pinMode(MOTOR_3_DIR_PIN, OUTPUT);

	digitalWrite(MOTOR_2_DIR_PIN, LOW);

	digitalWrite(MOTOR_3_DIR_PIN, LOW);

    configDrivers();
    digitalWrite(MOTOR_0_DIR_PIN, HIGH);
    digitalWrite(MOTOR_1_DIR_PIN, HIGH);



    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    

    testMotors();
    delay(5000);

}



int Calibration::start()
{
    int value_f;
    int ref_hall_1 = 1700; 
    int ref_hall_2 = 1900; 
    int cont_back;
    int cont_pasos = 0;
    
    driver.rms_current(CURRENT_IN_CALIBRATION);
    driver.microsteps(MICROSTEPPING);
    delay(500);
    

    flag_timer = 0;
    normal_turn();

    
    //======we tense the belt between motor 1 and motor 2=====
    driver.microsteps(32);
    driver2.microsteps(32);
    Serial.println("tense se belt 1 and 2");
    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, HIGH); //Motor 2 tenses the belt
    delay(1000);
    flag_timer = 5; 
    delay(500);

    while(stall_out == 0)
    {
        Serial.println(driver.SG_RESULT(), DEC);
        check_stall(1, Ap_stall);
    }
    stall_out = 0;
    driver.microsteps(MICROSTEPPING);
    driver2.microsteps(MICROSTEPPING);

    

    //==============instead of check if it is near to the sensor in this case, as the threads measures almost the same
    // we assume that if we tense the threads 1 and 2 the gondola will be near to the center, so 
    // it goes to the right to find the center of the sensor
    int cont_back1 = 0;
    digitalWrite(MOTOR_0_DIR_PIN, HIGH); //Motor 1 releases the belt
    digitalWrite(MOTOR_1_DIR_PIN, HIGH); //Motor 2 tenses the belt
    while(cont_back1 <= 3200)    
    {
        move(1, 1, 200);
        move(1, 2, 200);
        cont_back1++;
    }

    // first approach to the sensor via interrupts
    delay(2000);
    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, LOW); //Motor 2 releases the belt
    flag_timer = 5; 
    int cont_centro1 = 0;
    do
    {
        for (int i = 0; i < 10; i++)
		{
			value_f = meanFilter4.AddValue(analogRead(hall1));
		}
		Serial.println(value_f);
        
    }
    while(value_f > ref_hall_1);
    flag_timer = 0; //stops movement via interrupts
    stall_out = 0;
    Serial.println("starts SlowCalibration");
    delay(5000);
    slow_Calibration_hall1_negative();
    delay(5000);
    //================================================================


    flag_timer = 0; //stops movement via interrupts
    cont_pasos = 0;
    digitalWrite(MOTOR_0_DIR_PIN, HIGH); //Motor 1 releases the belt
    digitalWrite(MOTOR_1_DIR_PIN, LOW); //Motor 2 releases the belt
    while(cont_pasos <= 27000)    //
    {
            move(1, 1, 200);
            move(1, 2, 200);
            cont_pasos++;
    }
    delay(5000);
    


    //======now we tense the belt between motor 3 and 4 =====
    driver3.microsteps(32);
    driver4.microsteps(32);
    digitalWrite(MOTOR_2_DIR_PIN, HIGH); //Motor 1 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, LOW); //Motor 2 tenses the belt
    flag_timer = 6; 
    delay(500);
    while(stall_out == 0)
    {       
        Serial.println(driver3.SG_RESULT(), DEC);
        check_stall(3, Ap_stall);
    }
    stall_out = 0;
    driver3.microsteps(MICROSTEPPING);
    driver4.microsteps(MICROSTEPPING);

    //==============instead of check if it is near to the sensor in this case, as the threads measures almost the same
    // we assume that if we tense the threads 3 and 4 the gondola will be near to the center, so
    // it goes to the right to find the center of the sensor
    int cont_back2 = 0;
    digitalWrite(MOTOR_2_DIR_PIN, LOW); //Motor 1 releases the belt
    digitalWrite(MOTOR_3_DIR_PIN, LOW); //Motor 2 tenses the belt
    while (cont_back2 <= 3200) {
        move(1, 3, 200);
        move(1, 4, 200);
        cont_back2++;
        }
    //=====================================================================================================================

    //=======we move motors 3 and 4 to move the gondola over the X axis=======     
    //first approach to the sensor via interrupts
    Serial.println("first approach to the magnet");
    delay(2000);
    digitalWrite(MOTOR_2_DIR_PIN, HIGH); //Motor 3 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, HIGH); //Motor 4 releases the belt
    flag_timer = 6; 
    int cont_centro2 = 0;
    do
    {
        for (int i = 0; i < 10; i++)
	{
	    value_f = meanFilter4.AddValue(analogRead(hall2));
	}
	Serial.println(value_f);
        
    }
    while(value_f > ref_hall_2);
    flag_timer = 0; //stops movement via interrupts
    stall_out = 0;
    Serial.println("starts SlowCalibration 2");
    delay(5000);
    slow_Calibration_hall2_negative();
    delay(5000);

    //================================================================

    flag_timer = 0;
    delay(5000);

    cont_pasos = 0;
    digitalWrite(MOTOR_2_DIR_PIN, LOW); //Motor 3 releases the belt
    digitalWrite(MOTOR_3_DIR_PIN, HIGH); //Motor 4 releases the belt
    while(cont_pasos <= 7840)
    {
            move(1, 3, 200);
            move(1, 4, 200);
            cont_pasos++;
    }

    cont_pasos = 0;
    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, HIGH); //Motor 2 tenses the belt
    while(cont_pasos <= 19160)
    {
            move(1, 1, 200);
            move(1, 2, 200);
            cont_pasos++;
    }
	
	delay(2000);
	return 0;
}

/**
 * @brief This function allows to move the motors according to a given steps. 
 * @param pasos is to indicate the number of steps to move.
 * @param motor_d indicates which motor to move, the options are 1, 2, 3 or 4.
 * @param Speed increase to reduce speed or decrease to speed up.
 */
void move(int pasos, int motor_d, int Speed)
{
	Speed = Speed;
	if (motor_d == 1)
	{
		for (int i = 0; i < pasos; i++)
		{
			digitalWrite(MOTOR_0_STEP_PIN, LOW);
			delayMicroseconds(Speed);
			digitalWrite(MOTOR_0_STEP_PIN, HIGH);
			delayMicroseconds(Speed);
		}
	}

	if (motor_d == 2)
	{
		for (int j = 0; j < pasos; j++)
		{
			digitalWrite(MOTOR_1_STEP_PIN, LOW);
			delayMicroseconds(Speed);
			digitalWrite(MOTOR_1_STEP_PIN, HIGH);
			delayMicroseconds(Speed);
		}
	}

    if (motor_d == 3)
	{
		for (int j = 0; j < pasos; j++)
		{
			digitalWrite(MOTOR_2_STEP_PIN, LOW);
			delayMicroseconds(Speed);
			digitalWrite(MOTOR_2_STEP_PIN, HIGH);
			delayMicroseconds(Speed);
		}
	}

    if (motor_d == 4)
	{
		for (int j = 0; j < pasos; j++)
		{
			digitalWrite(MOTOR_3_STEP_PIN, LOW);
			delayMicroseconds(Speed);
			digitalWrite(MOTOR_3_STEP_PIN, HIGH);
			delayMicroseconds(Speed);
		}
	}
}

void check_stall(int driver_m, int *result_stall)
{
	int stall_data = 0;
	if (driver_m == 1)
	{

		for (int i = 0; i < 5; i++)
		{
			stall_data = meanFilter3.AddValue(driver.SG_RESULT());
		}

		if (stall_data < 12)   
		{
			flag_timer = 0;
			Serial.println("stall guard m1 was triggered");
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}

	if (driver_m == 2)
	{

		for (int i = 0; i < 5; i++)
		{
			stall_data = meanFilter3.AddValue(driver2.SG_RESULT());
		}

		if (stall_data < 12)  
		{
			flag_timer = 0;
			Serial.println("stall guard M2 was triggered");
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}

        if (driver_m == 3)
	{

		for (int i = 0; i < 5; i++)
		{
			stall_data = meanFilter3.AddValue(driver3.SG_RESULT());
		}

		if (stall_data < 12)  
		{
			flag_timer = 0;
			Serial.println("stall guard M3 was triggered");
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}

    if (driver_m == 4)
	{

		for (int i = 0; i < 5; i++)
		{
			stall_data = meanFilter3.AddValue(driver4.SG_RESULT());
		}

		if (stall_data < 12)  
		{
			flag_timer = 0;
			Serial.println("stall guard m4 was triggered");
			delay(1000);
			*result_stall = 1;
		}
		else
		{
			*result_stall = 0;
		}
	}
}


void slow_Calibration_hall1_negative()
{
	int value1_f;
	for (int i = 0; i < 1200; i++)
	{
		value[i] = -1;
	}
	for (int j = 0; j < 1200; j++)
	{
		value_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int minimum2_r = 5000;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
    int offset2;
	int cont_t1 = 0;
	int cont_t2 = 0;

    int cont_salir = 0;
    
    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, LOW); //Motor 2 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
    
    read_hall1 = value1_f;
	//======================================================
    
	while (read_hall1 > min_hall1)
	{
		move(1, 1, 500);
        move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			//read_hall1 = (analogRead(hall1)) / 4;
            read_hall1 = (analogRead(hall1));

			read_hall1 = meanFilter2.AddValue(read_hall1);
		}
	}
    
    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, LOW); //Motor 2 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
    
    read_hall1 = value1_f;
	for (int t = 0; t < 1200; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value1_f = meanFilter2.AddValue(analogRead(hall1));
		}
        
        read_hall1 = value1_f;

		move(1, 1, 500);
        move(1, 2, 500);

		ind_2 = 1199 - t;
		value_r[ind_2] = read_hall1;
		if (value_r[ind_2] < min_hall1)
		{
			cont_t2++;
		}

		if (read_hall1 < minimum2_r)
		{
			minimum2_r = read_hall1;
			index_min1 = ind_2;
		}
	}

    delay(3000);


    digitalWrite(MOTOR_0_DIR_PIN, HIGH); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, HIGH); //Motor 2 releases the belt

	for (int y = 0; y < 40; y++)
	{
		//read_hall1 = (analogRead(hall1)) / 4;
        read_hall1 = (analogRead(hall1));
		val_sens = meanFilter2.AddValue(read_hall1);
	}

	while (val_sens > min_hall1)
	{
		move(1, 1, 500);
        move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
			//read_hall1 = (analogRead(hall1)) / 4;
            read_hall1 = (analogRead(hall1));
			val_sens = meanFilter2.AddValue(read_hall1);
		}
	}
    Serial.println("the start of the sensor detection in the left side was found");
    

	while (k < 1200)
	{
		move(1, 1, 500);
        move(1 ,2 ,500);
		for (int y = 0; y < 40; y++)
		{
            read_hall1 = (analogRead(hall1));
			val_sens = meanFilter2.AddValue(read_hall1);
		}
		delay(1);

		if (val_sens < minimum2)
		{
			minimum2 = val_sens;
			index_min2 = k;
		}
		value[k] = val_sens;

		if (value[k] < min_hall1)
		{
			cont_t1++;
		}
		delay(1);
		k++;
	}
    delay(3000);

    minimum = (minimum2 + minimum2_r)/2;

	//====
	limit = minimum + ((level_zero1 - minimum) * 0.2);
	k = 1199;
	while (k >= 0)
	{
		if (value[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 1199;
	while (k >= 0)
	{
		if (value_r[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value_r[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (1200 - steps_ini2) +((steps_ini2 - steps_fin2) / 2);

    offset2 = (steps_fin) +((steps_ini - steps_fin) / 2);

    Serial.println("offset");
    Serial.println(offset);
    Serial.println("steps_ini");
    Serial.println(steps_ini);
    Serial.println("steps_fin");
    Serial.println(steps_fin);
    Serial.println("offset 2");
    Serial.println(offset2);
    Serial.println("steps_ini2");
    Serial.println(steps_ini2);
    Serial.println("steps_fin2");
    Serial.println(steps_fin2);
    Serial.println("minimum2_r");
    Serial.println(minimum2_r);
    Serial.println("minimum2");
    Serial.println(minimum2);


    digitalWrite(MOTOR_0_DIR_PIN, LOW); //Motor 1 tenses the belt
    digitalWrite(MOTOR_1_DIR_PIN, LOW); //Motor 2 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall1));
	}
    
    read_hall1 = value1_f ;
	while (read_hall1 > min_hall1)
	{
		move(1, 1, 500);
        move(1, 2, 500);
		for (int y = 0; y < 40; y++)
		{
            read_hall1 = (analogRead(hall1));
			read_hall1 = meanFilter2.AddValue(read_hall1);
		}
	}
    Serial.println("it goes back to the start of the sensor to go to the center point");
    delay(3000);
    pas = (offset2 + offset)/2;
    Serial.println("pas");
    Serial.println(pas);
    int cont_centrar = 0;
    
    while(cont_centrar < pas)
    {
        move(1, 1, 500);
        move(1, 2, 500);
        cont_centrar++;
    }
    
    for(int i = 0 ; i < 1200; i++)
    {
        Serial.print(value_r[i]);
        Serial.print(" ");
        Serial.print(value[i]);
        Serial.println("");
    }
}


void slow_Calibration_hall2_negative()
{
	int value1_f;
	for (int i = 0; i < 1200; i++)
	{
		value[i] = -1;
	}
	for (int j = 0; j < 1200; j++)
	{
		value_r[j] = -1;
	}
	int k = 0;
	int val_sens;
	int Flag_ini = 0;
	int Flag_fin = 0;
	int steps_ini;
	int steps_fin;
	int steps_ini2;
	int steps_fin2;
	int limit;
	int count_ini = 0;
	int count_fin = 0;
	int minimum2_r = 5000;
	int index_min1;
	int index_min2;
	int ind_2;
	int offset;
    int offset2;
	int cont_t1 = 0;
	int cont_t2 = 0;

    int cont_salir = 0;
    
    //it starts to find the hall sensor
	//digitalWrite(DIR_PIN, HIGH);
    digitalWrite(MOTOR_2_DIR_PIN, HIGH); //Motor 3 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, HIGH); //Motor 4 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall2));
	}

    read_hall1 = value1_f;
	//======================================================
    
	while (read_hall1 > min_hall2)
	{
		move(1, 3, 500);
        move(1, 4, 500);
		for (int y = 0; y < 40; y++)
		{
            read_hall1 = (analogRead(hall2));
			read_hall1 = meanFilter2.AddValue(read_hall1);
		}
	}
    Serial.println("It found the start of the sensor detection from the righ side");
    

    digitalWrite(MOTOR_2_DIR_PIN, HIGH); //Motor 3 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, HIGH); //Motor 4 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall2));
	}
    
    read_hall1 = value1_f;
	for (int t = 0; t < 1200; t++)
	{
		for (int i = 0; i < 40; i++)
		{
			value1_f = meanFilter2.AddValue(analogRead(hall2));
		}
        
        read_hall1 = value1_f;
		move(1, 3, 500);
        move(1, 4, 500);

		ind_2 = 1199 - t;
		value_r[ind_2] = read_hall1;
		if (value_r[ind_2] < min_hall2)
		{
			cont_t2++;
		}

		if (read_hall1 < minimum2_r)
		{
			minimum2_r = read_hall1;
			index_min1 = ind_2;
		}
	}
    
    delay(3000);

    digitalWrite(MOTOR_2_DIR_PIN, LOW); //Motor 3 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, LOW); //Motor 4 releases the belt

	for (int y = 0; y < 40; y++)
	{
		//read_hall1 = (analogRead(hall2)) / 4;
        read_hall1 = (analogRead(hall2)) ;
		val_sens = meanFilter2.AddValue(read_hall1);
	}

	while (val_sens > min_hall2)
	{
		move(1, 3, 500);
        move(1, 4, 500);
		for (int y = 0; y < 40; y++)
		{
			//read_hall1 = (analogRead(hall2)) / 4;
            read_hall1 = (analogRead(hall2));
			val_sens = meanFilter2.AddValue(read_hall1);
		}
	}
    Serial.println("it found the start of the sensor detection from the left side");
    

	while (k < 1200)
	{
		move(1, 3 ,500);
        move(1 ,4 ,500);
		for (int y = 0; y < 40; y++)
		{
            read_hall1 = (analogRead(hall2));
			val_sens = meanFilter2.AddValue(read_hall1);
		}
		delay(1);

		if (val_sens < minimum2)
		{
			minimum2 = val_sens;
			index_min2 = k;
		}
		value[k] = val_sens;

		if (value[k] < min_hall2)
		{
			cont_t1++;
		}
		delay(1);
		k++;
	}
    Serial.println("vector value is full");
    delay(3000);

    minimum = (minimum2 + minimum2_r)/2;

	//====
	limit = minimum + ((level_zero1 - minimum) * 0.2);
	k = 1199;
	while (k >= 0)
	{
		if (value[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini = k + 9;
				Flag_ini = 1;
			}
		}
		if (value[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	Flag_ini = 0;
	Flag_fin = 0;
	count_ini = 0;
	count_fin = 0;

	k = 1199;
	while (k >= 0)
	{
		if (value_r[k] < limit && Flag_ini == 0)
		{
			count_ini++;
			if (count_ini == 9)
			{
				steps_ini2 = k + 9;
				Flag_ini = 1;
			}
		}
		if (value_r[k] > limit && Flag_ini == 1 && Flag_fin == 0)
		{
			count_fin++;
			if (count_fin == 9)
			{
				steps_fin2 = k + 9;
				Flag_fin = 1;
			}
		}
		k--;
	}

	int pas;
	int half;
	half = (cont_t1 + cont_t2) / 2;

	int half1;

	half1 = (half) / 2;

	offset = (1200 - steps_ini2) +((steps_ini2 - steps_fin2) / 2);

    offset2 = (steps_fin) +((steps_ini - steps_fin) / 2);

    Serial.println("offset");
    Serial.println(offset);
    Serial.println("steps_ini");
    Serial.println(steps_ini);
    Serial.println("steps_fin");
    Serial.println(steps_fin);
    Serial.println("offset 2");
    Serial.println(offset2);
    Serial.println("steps_ini2");
    Serial.println(steps_ini2);
    Serial.println("steps_fin2");
    Serial.println(steps_fin2);
    Serial.println("minimum2_r");
    Serial.println(minimum2_r);
    Serial.println("minimum2");
    Serial.println(minimum2);

    digitalWrite(MOTOR_2_DIR_PIN, HIGH); //Motor 3 tenses the belt
    digitalWrite(MOTOR_3_DIR_PIN, HIGH); //Motor 4 releases the belt
	for (int i = 0; i < 40; i++)
	{
		value1_f = meanFilter2.AddValue(analogRead(hall2));
	}
    
    read_hall1 = value1_f ;
	while (read_hall1 > min_hall2)
	{
		move(1, 3, 500);
        move(1, 4, 500);
		for (int y = 0; y < 40; y++)
		{
            read_hall1 = (analogRead(hall2));
			read_hall1 = meanFilter2.AddValue(read_hall1);
		}
	}
    Serial.println("it goes back to the start of the sensor to go to the center point");
    delay(3000);
    pas = (offset2 + offset)/2;
    Serial.println("pas");
    Serial.println(pas);
    int cont_centrar = 0;
    while(cont_centrar < pas)
    {
        move(1, 3, 500);
        move(1, 4, 500);
        cont_centrar++;
    }
}


void testMotors()
{
    //=============TEST MOTORES START========= 

    //=================Test MOTOR 1============
    Serial.println("Test Motor 1");
    uint8_t result_1 = driver.test_connection();
	if (result_1 == 0)
	{
		Serial.println(F("M1_OK,"));
	}
	else
	{
		

		Serial.println(F("M1_Fail,"));
	}

    driver.rms_current(700);
    driver.microsteps(32);
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    delay(200);


    driver.rms_current(900);
    driver.microsteps(64);
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    delay(200);

    driver.rms_current(CURRENT_IN_CALIBRATION);
    driver.microsteps(MICROSTEPPING);
    Serial.println(driver.rms_current());
    Serial.println(driver.microsteps());
    //==============================================

    //=================Test MOTOR 2============
    Serial.println("Test Motor 2");
    uint8_t result_2 = driver2.test_connection();
	if (result_2 == 0)
	{
		Serial.println(F("M2_OK,"));
	}
	else
	{
		Serial.println(F("M2_Fail,"));
	}

    driver2.rms_current(700);
    driver2.microsteps(32);
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    delay(200);


    driver2.rms_current(900);
    driver2.microsteps(64);
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());
    delay(200);

    driver2.rms_current(CURRENT_IN_CALIBRATION);
    driver2.microsteps(MICROSTEPPING);
    Serial.println(driver2.rms_current());
    Serial.println(driver2.microsteps());

    //=================Test MOTOR 3============
    Serial.println("Test Motor 3");
    uint8_t result_3 = driver3.test_connection();
	if (result_3 == 0)
	{
		Serial.println(F("M3_OK,"));
	}
	else
	{
		Serial.println(F("M3_Fail,"));
	}

    driver3.rms_current(700);
    driver3.microsteps(32);
    Serial.println(driver3.rms_current());
    Serial.println(driver3.microsteps());
    delay(200);


    driver3.rms_current(900);
    driver3.microsteps(64);
    Serial.println(driver3.rms_current());
    Serial.println(driver3.microsteps());
    delay(200);

    driver3.rms_current(CURRENT_IN_CALIBRATION);
    driver3.microsteps(MICROSTEPPING);
    Serial.println(driver3.rms_current());
    Serial.println(driver3.microsteps());
    //=================Test MOTOR 4============
    Serial.println("Test Motor 4");
    uint8_t result_4 = driver4.test_connection();
	if (result_4 == 0)
	{
		Serial.println(F("M4_OK,"));
	}
	else
	{
		Serial.println(F("M4_Fail,"));
	}

    driver4.rms_current(700);
    driver4.microsteps(32);
    Serial.println(driver4.rms_current());
    Serial.println(driver4.microsteps());
    delay(200);


    driver4.rms_current(900);
    driver4.microsteps(64);
    Serial.println(driver4.rms_current());
    Serial.println(driver4.microsteps());
    delay(200);

    driver4.rms_current(CURRENT_IN_CALIBRATION);
    driver4.microsteps(MICROSTEPPING);
    Serial.println(driver4.rms_current());
    Serial.println(driver4.microsteps());
    
    //=============TEST MOTORES ENDS============

}

void normal_turn()
{
	{
		cli();										  
		timer1 = timerBegin(3, 8, true);			  
		timerAttachInterrupt(timer1, &onTimer, true); 
		timerAlarmWrite(timer1, 6000, true);		  
		timerAlarmEnable(timer1);					  
		sei();										  
	}
}


void IRAM_ATTR onTimer()
{
	if (flag_timer == 1)
	{
		digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
	}
        if (flag_timer == 2)
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
	}
	if (flag_timer == 3)
	{
		digitalWrite(MOTOR_2_STEP_PIN, !digitalRead(MOTOR_2_STEP_PIN));
	}
        if (flag_timer == 4)
	{
		digitalWrite(MOTOR_3_STEP_PIN, !digitalRead(MOTOR_3_STEP_PIN));
	}
        if (flag_timer == 5)
	{
		digitalWrite(MOTOR_1_STEP_PIN, !digitalRead(MOTOR_1_STEP_PIN));
        digitalWrite(MOTOR_0_STEP_PIN, !digitalRead(MOTOR_0_STEP_PIN));
	}
        if (flag_timer == 6)
	{
		digitalWrite(MOTOR_3_STEP_PIN, !digitalRead(MOTOR_3_STEP_PIN));
        digitalWrite(MOTOR_2_STEP_PIN, !digitalRead(MOTOR_2_STEP_PIN));
	}
}


void configDrivers(void)
{
	//===========Motor 1 y 2============
	driver.begin();

	driver.pdn_disable(true); // enables the  PDN/UART comunication.

	driver.toff(4); 

	driver.blank_time(24);

	driver.rms_current(CURRENT_IN_CALIBRATION); // set the current value in miliampers

	driver.microsteps(MICROSTEPPING); // it set the microsteps value

	driver.TCOOLTHRS(0xFFFFF); // inferior speed threshold to turn on the Coolstep and stallGuard smart energy to the output of DIAG

	driver.semin(0); // inferior Coolstep threshold [0 ... 15].
        // if SG_RESULT is less than this threshold, coolStep increase the current for both coils.
        // 0: disables CoolStep

	driver.shaft(false); //it set the direction of the motor via UART

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


    //===========Motor 3 and 4============
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
	while (driver.microsteps() != MICROSTEPPING)
	{
		driver.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
	cont = 0;
	while (driver2.microsteps() != MICROSTEPPING)
	{
		driver2.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}
    
    cont = 0;
	while (driver3.microsteps() != MICROSTEPPING)
	{
		driver3.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}

    cont = 0;
	while (driver4.microsteps() != MICROSTEPPING)
	{
		driver4.microsteps(MICROSTEPPING);
		delay(100);
		cont++;
		if (cont == 5)
		{
			break;
		}
	}

}