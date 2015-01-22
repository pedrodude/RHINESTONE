
uint8_t SENSOR_UCS = 4; //Digital Pin for Pushbutton Switch
uint8_t SENSOR_DS = 2; //Analog Pin for Reading Accelerometer
uint8_t ACTUATOR_SL = 3; //Digital Pin for LED

float CONSTANT_UPWARD_DT;
float CONSTANT_DOWNWARD_DT;
float CONSTANT_DEBOUNCETIME;
float CONSTANT_STRAIGHTANDLEVELRANGE;
float CONSTANT_GRAVITY;
const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 5;

uint8_t VARIABLE_UPDOWN;
float VARIABLE_COMPENSATED_DECELERATION;
String VARIABLE_CURRENT_MODE;
float VARIABLE_QUEUE_DECELERATION[VARIABLE_QUEUE_DECELERATION_SIZE];
float VARIABLE_QUEUE_RANGE;
float VARIABLE_COMPUTED_PITCHANGLE;
uint8_t VARIABLE_SAMPLE_RATE;
uint8_t VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE;


void setup()
{
	pinMode(ACTUATOR_SL, OUTPUT);
        Serial.begin(9600);
	BOOT();
}

void loop()
{
    digitalWrite(ACTUATOR_SL,HIGH);
    delay(1000);
    digitalWrite(ACTUATOR_SL,LOW);
    delay(1000);
    Serial.println(analogRead(SENSOR_DS));
}

void BOOT()
{
	VARIABLE_UPDOWN = 0;
	VARIABLE_COMPENSATED_DECELERATION = 0;
	VARIABLE_CURRENT_MODE = "boot";
	VARIABLE_QUEUE_RANGE = 0;
	VARIABLE_COMPUTED_PITCHANGLE = 0;
	VARIABLE_SAMPLE_RATE = 10;
	VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE = 1;

	for(uint8_t i = 0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
	{
		VARIABLE_QUEUE_DECELERATION[i] = 0;
	}
}

void TRANSITION_TO_SLEEP()
{
	VARIABLE_CURRENT_MODE = "transition_to_sleep";
	//deactivate all circuitry
	SLEEP_SIGNAL();
}

void MODE_SLEEP()
{
	VARIABLE_CURRENT_MODE = "mode_sleep";
}

void TRANSITION_TO_NORMAL()
{
	VARIABLE_CURRENT_MODE = "transition_to_normal";
	NORMAL_SIGNAL();
}

void MODE_NORMAL()
{
        uint16_t sample;
        unsigned long time;
        float min_val = 0;
        float max_val = 0;
        float mean = 0;
        float range;
	VARIABLE_CURRENT_MODE = "mode_normal";

        for(uint8_t i=0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
        {
            time  = millis();
            sample = analogRead(SENSOR_DS); //this takes roughly 100us
            VARIABLE_QUEUE_DECELERATION[i] = (sample/1024)*5;
            while(millis() - time < (1/VARIABLE_SAMPLE_RATE));
        }
        
        min_val = VARIABLE_QUEUE_DECELERATION[0];
        max_val = VARIABLE_QUEUE_DECELERATION[0];
        mean = VARIABLE_QUEUE_DECELERATION[0];
        for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
        {
            if(VARIABLE_QUEUE_DECELERATION[i] < min_val)
            {
                min_val = VARIABLE_QUEUE_DECELERATION[i];
            }
            if(VARIABLE_QUEUE_DECELERATION[i] > max_val)
            {
                max_val = VARIABLE_QUEUE_DECELERATION[i];
            }
            mean = mean + VARIABLE_QUEUE_DECELERATION[i];
        }
        mean = mean/VARIABLE_QUEUE_DECELERATION_SIZE;
        range = max_val - min_val;
        
        if(range <= CONSTANT_STRAIGHTANDLEVELRANGE)
        {
            VARIABLE_COMPUTED_PITCHANGLE = asin(mean/CONSTANT_GRAVITY); 
        }
        else
        {
            VARIABLE_COMPENSATED_DECELERATION = (mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
            if((VARIABLE_UPDOWN == 0)&&(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_UPWARD_DT))
            {
                TRANSITION_ACTIVATE_ACTUATOR_SL();
            }
            else if((VARIABLE_UPDOWN == 1)&&(VARIABLE_COMPENSATED_DECELERATION < CONSTANT_DOWNWARD_DT))
            {
                TRANSITION_DEACTIVATE_ACTUATOR_SL();
            }    
        }
}

void TRANSITION_ACTIVATE_ACTUATOR_SL()
{
	VARIABLE_CURRENT_MODE = "transition_activate_actuator_sl";
	BRAKING_SIGNAL_ON();
	VARIABLE_UPDOWN = 1;
}

void TRANSITION_DEACTIVATE_ACTUATOR_SL()
{
	VARIABLE_CURRENT_MODE = "transition_deactivate_actuator_sl";
	BRAKING_SIGNAL_OFF();
	VARIABLE_UPDOWN = 0;
}

void SLEEP_SIGNAL()
{
    unsigned long time = millis();
    
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis - time < 200);
    digitalWrite(ACTUATOR_SL,LOW);
    delay(50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis - time < 100);
    digitalWrite(ACTUATOR_SL,LOW);
    delay(50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis - time < 50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis - time < 1000);
}

void NORMAL_SIGNAL()
{
    unsigned long time = millis();
    
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis - time < 200);
    digitalWrite(ACTUATOR_SL,HIGH);
    delay(50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis - time < 100);
    digitalWrite(ACTUATOR_SL,HIGH);
    delay(50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis - time < 50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis - time < 1000);
}

void BRAKING_SIGNAL_ON()
{
    digitalWrite(ACTUATOR_SL,HIGH);
}

void BRAKING_SIGNAL_OFF()
{
    digitalWrite(ACTUATOR_SL,LOW);
}
