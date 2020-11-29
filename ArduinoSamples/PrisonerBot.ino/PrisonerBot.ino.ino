#include <avr/wdt.h>
#include "MeAuriga.h"

// Ultrasonic sensors
MeUltrasonicSensor *us = NULL;  //PORT_10
MeUltrasonicSensor *us2 = NULL; //PORT_7
int16_t distance = 0;
int16_t distance2 = 0;
boolean leftflag;
boolean rightflag;

// Motors
MeEncoderOnBoard Encoder_1(SLOT_1);
MeEncoderOnBoard Encoder_2(SLOT_2);
MeEncoderMotor encoders[2];
int16_t moveSpeed = 180;
int16_t turnSpeed = 180;
int16_t minSpeed = 45;

// Leds
MeRGBLed led(0);
#define BUZZER_PORT 45
#define LED_PORT 44
#define LEDNUM   12
bool Obstacle = false;
int FLAG;

// Buzzer
MeBuzzer buzzer;
uint8_t prevc = 0;


// ?????????
MePort generalDevice;
Me4Button buttonSensor;
MeJoystick joystick;

double  lastTime = 0.0;
double  currentTime = 0.0;
long blink_time = 0;
long rxruntime = 0;
boolean blink_flag = false;

uint8_t keyPressed = KEY_NULL;
uint8_t serialRead;
uint8_t buffer[52];

boolean isStart = false;
boolean isAvailable = false;

uint8_t command_index = 0;
uint8_t index = 0;
uint8_t dataLen;


typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

#define ULTRASONIC_SENSOR      1
#define LIGHT_SENSOR           3
#define JOYSTICK               5
#define SOUND_SENSOR           7
#define ENCODER_BOARD          61
//Read type
#define ENCODER_BOARD_POS    0x01
#define ENCODER_BOARD_SPEED  0x02
#define ENCODER_PID_MOTION     62
//Secondary command
#define ENCODER_BOARD_POS_MOTION         0x01
#define ENCODER_BOARD_SPEED_MOTION       0x02
#define ENCODER_BOARD_PWM_MOTION         0x03
#define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
#define ENCODER_BOARD_CAR_POS_MOTION     0x05

/////////////////////////////////////Q LEARNING PARAMETERS///////////////////////////////////////////
float ALPHA = 0.1;    //LEARNING RATE
float GAMMA = 0.5;    //DISCOUNT FACTOR
float EPSILON = 0.90; //EXPLORATION PARAMETER
int REWARD;           //REWARD FOR PERFORMING AN ACTION
int EPISODES  = 100;

int STATE;                        // CURRENT STATE OF THE ROBOT
int ACTION = 0;                   //ACTION PERFORMED BY THE ROBOT(0:FORWARD,1:BACKWARD ,2;STOP,3:LEFT)
float PROB;                       //USED FOR EPSILON DECAY
bool ACTION_TAKEN = false;        //THIS VARIABLES TELLS US WHETHER AN ACTION IS TAKEN OR NOT
int NEXT_STATE;                   // NEXT STATE OF THE ROBOT
const int STATES = 10;            //NUMBER OF STATES IN ENVIRONMENT
int ACTIONS[4] = {1, 2, 3, 4};
const int NUMBER_OF_ACTIONS = 4; //TOTAL WE HAVE 4 ACTION FORWARD,BACKWARD,LEFT AND STOP


/*THIS IS THE Q MATRIX OR Q TABLE. THIS IS BASICALLY THE DIARY THAT ROBOT WILL LOOK INTO
  BEFORE PERFORMING AN ACTION.BASED ON THE ACTION THE ROBOT WILL EARN REWARD AND THE Q VALUE
  WILL BE UPDATED IN THIS Q TABLE. HERE I HAVE CONISDERED 10 STATES. I HAVE ASSUMED ALL STATES
  ARE DIFFERENT EVEN THOUGH THEY ARE SAME.BASICALLY OBSTACLE AVOIDING ROBOT CONTAINS ONLY TWO STATES
  i.e:
  1:WHEN ITS AWAY FROM OBSTACLE
  2:WHEN ITS NEAR TO THE OBSTACLE
  BUT HERE TO ILLUSTRATE MORE COMPLEX ENVIRONMENT I HAVE ASSUMED THERE ARE 10 DIFFERENT STATES HERE
  EXPECTING SAME/DIFFERENT ACTION.*/

float Q[STATES][NUMBER_OF_ACTIONS] = {{0.0, 0.0, 0.0, 0.0}, //MOST IMPORTANT OF ALL IS THE Q TABLE.
  {0.0, 0.0, 0.0, 0.0}, //IT IS FORMED BY STATES AS ITS  ROWS
  {0.0, 0.0, 0.0, 0.0}, //AND COLLUMNS AS ITS NUMBER OF ACTIONS
  {0.0, 0.0, 0.0, 0.0}, //INITIALISED TO ZERO IN THE START
  {0.0, 0.0, 0.0, 0.0}, // THIS WILL UPDATED IN THE FUTURE.
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0},
  {0.0, 0.0, 0.0, 0.0}
};

/*THIS IS A REWARD MATRIX OR REWARD TABLE. THIS IS RESPONSIBLE FOR GIVING
  REWARD TO ROBOT FOR PERFORMING PARTICULAR ACTION. IT STORES THE REWARD FOR
  EACH ACTION TAKEN AT STATE. THE REWARD WILL BE POSITIVE IF THE ACTION
  PERFORMED IS GOOD AND NEGATIVE IF ACTION YIELDS BAD RESULTS.*/

int REWARDS[STATES][NUMBER_OF_ACTIONS] = {{ -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10},
  { -10, -2, -1, 10}
};
////////////////////////////////////////////END///////////////////////////////////////////////////

////////////////Q LEARNING UPDATE PARAMETERS////////////
float Q_OLD;
float Q_NEW;
float Q_MAX;
//New end


bool Obstacle_Avoider(int dist)
{
  for (int16_t i = 0; i < 300; i++)
  {
    wdt_reset();
    if (read_serial() == true)
    {
      break;
    }
    else
    {
      delay(2);
    }
  }

  if (us == NULL)
  {
    us = new MeUltrasonicSensor(PORT_10);
  }
  if (us != NULL)
  {
    distance = us->distanceCm();
  }

  if (us2 == NULL)
  {
    us2 = new MeUltrasonicSensor(PORT_7);
  }
  if (us2 != NULL)
  {
    distance2 = us2->distanceCm();
  }


  if (distance > 0 && distance <= dist)
  {
    led.setColor(11, 255, 255, 0);
    led.show();
    return true;
  }
  else if (distance > dist)
  {
    led.setColor(11, 0, 255, 0);
    led.show();
    return false;
  }

  if (distance2 > 0 && distance2 <= dist)
  {
    led.setColor(12, 255, 255, 0);
    led.show();
    return true;
  }
  else if (distance2 > dist)
  {
    led.setColor(12, 0, 255, 0);
    led.show();
    return false;
  }

}


float RANDOM(float EXPLORATION_PARAMETER)
{
  /*THIS FUNCTION FINDS RANDOM NUMBER WHICH
    DECIDES WHETHER AN ACTION TO BE TAKEN IS RANDOM
    OR FROM Q_TABLE*/

  float RANDOM_VARIABLE;
  float PROBABILITY;

  RANDOM_VARIABLE = random(0, 100);
  PROBABILITY = RANDOM_VARIABLE / 100;

  return PROBABILITY;
}


float DECAY(float PARAMETER)
{
  /*THIS FUNCTION IS USED TO REDUCE
    EPSILON(EXPLORATION PARAMETER) WITH
    TIME.FINALLY AT THE END YOU GET RID
    EPSILON AND THE ROBOT LEARNS TO AVOID
    OBSTACLES ON ITS OWN */

  PARAMETER = PARAMETER * 0.98; //PARAMETER HERE IS THE EPSILON
  return PARAMETER;
}

int GET_STATE()
{
  int STATE_NUMBER;
  STATE_NUMBER = random(0, 10);
  return STATE_NUMBER;
}


float MAX(float Q_Table[][4], int NEXT_S)
{
  /*THIS FUNCTION FINDS THE BIGGEST NUMBER
    IN Q_TABLE[NEXT_STATE]. THE MAIN ROLE OF
    THIS FUNCTION IS TO FIND Q_MAX PARAMETER*/

  float LIST[4];
  float N1;
  float N2;
  float MAX_VALUE = 0.0;
  float DIFF;

  for (int b = 0; b <= 3; b++)
  {
    LIST[b] = Q[NEXT_S][b];
  }

  for (int j = 0; j <= 2 ; j++)
  {
    if (MAX_VALUE > LIST[j])
    {
      N1 = MAX_VALUE;
    }
    else
    {
      N1 = LIST[j];
    }

    N2 = LIST[j + 1];
    DIFF = N1 - N2;

    if (DIFF > 0)
    {
      MAX_VALUE = N1;
    }

    else
    {
      MAX_VALUE = N2;
    }
  }
  return MAX_VALUE;
}


int ARGMAX(float Q_Table[][4], int S)
{
  /*THIS FUNCTION FINDS THE INDEX OF
    BIGGEST Q VALUE IN Q TABLE[STATE]*/

  float ARRAY[4];
  float N1;
  float N2;
  float MAX_VALUE = 0.0;
  float DIFF;
  float NUMBER;
  int MAX_INDEX;

  for (int u = 0; u <= 3; u++)
  {
    ARRAY[u] = Q_Table[S][u];
  }

  for (int p = 0; p <= 2 ; p++)
  {
    if (MAX_VALUE > ARRAY[p])
    {
      N1 = MAX_VALUE;
    }
    else
    {
      N1 = ARRAY[p];
    }

    N2 = ARRAY[p + 1];
    DIFF = N1 - N2;

    if (DIFF > 0)
    {
      MAX_VALUE = N1;
    }

    else
    {
      MAX_VALUE = N2;
    }
  }

  for (int r = 0; r <= 3; r++)
  {
    NUMBER = ARRAY[r];
    if (NUMBER == MAX_VALUE)
    {
      MAX_INDEX  = r;
      break;
    }
  }

  return MAX_INDEX;
}

void Update(float Q_TABLE[][4] , int S, int NEXT_S, int A, int ACTIONS[], int R, float LEARNING_RATE, float DISCOUNT_FACTOR)
{
  /*THIS FUNCTION UPDATES THE Q TABLE AND Q VALUES. THIS UPDATE KEEPS ON HAPPENING UNTILL THE
    MAIN LOOP ENDS. AT THE END OF EPISODES THE Q TABLE IS FILLED WITH VARIOUS VALUES. THE GREATER
    THE VALUES THE GREATER IMPORTANCE THE ACTION HAS AT THAT PARTICULAR STATE. "Q_OLD" IS OLD VALUE
    THAT THE Q MATRIX HAS.THIS IS THE VALUE WHICH GETS UPDATED EVENTUALLY. Q_NEW IS THE NEW Q_VALUE
    WHICH IS CALCULATED BY THE Q LEARNING FORMULA. THE Q LEARNING FORMULA USED HERE IS BASED ON
    BELLMAN EQUATION USES TEMPORAL DIFFERENCE LEARNING APPROACH.(MONTE CARLO APPROACH WILL NOT
    WORK IN THIS CASE OF OBSTACLE AVOIDING ROBOT.*/

  Q_OLD = Q_TABLE[S][A];
  Q_MAX = MAX(Q_TABLE, NEXT_S);
  Q_NEW = (1 - LEARNING_RATE) * Q_OLD + LEARNING_RATE * (R + DISCOUNT_FACTOR * Q_MAX);
  Q_TABLE[S][A] = Q_NEW;
}

/**
   Q-Learning based obstacle avoidence method.
 * */
void learnObstacleAvoidence(void)
{
  for (int I = 0; I < EPISODES; I++)
  {
    ACTION_TAKEN = false;
    FLAG = 0;
    Stop();
    led.setColor(0, 0, 0, 255);
    led.show();
    Obstacle = false;
    while (true)
    {
      Forward();
      led.setColor(0, 10, 10, 10);
      led.show();
      delay(2);
      Obstacle = Obstacle_Avoider(40);
      if (Obstacle == true)
      {
        led.setColor(0, 0, 0, 0);
        led.show();
        NEXT_STATE = STATE + 1;

        if (NEXT_STATE == 10)
        {
          NEXT_STATE = 0;
        }

        if (NEXT_STATE < 0)
        {
          NEXT_STATE = 0;
        }
        FLAG = 1;
        break;
      }
    }
    if (FLAG == 1)
    {
      PROB = RANDOM(EPSILON);
      if (PROB <= EPSILON)   //EXPLORE THE ACTIONS
      {
        led.setColor(1, 0, 255, 0);
        led.show();
        ACTION = random(4);
        FLAG = 2;
      }
      else                  //EXPLOIT THE ACTIONS FROM Q TABLE
      {
        led.setColor(1, 255, 0, 0);
        led.show();
        ACTION = ARGMAX(Q, STATE);
        FLAG = 2;
      }
    }

    if (FLAG == 2)
    {
      if (ACTION == 0)
      {
        led.setColor(5, 50, 50, 0);
        led.show();

        unsigned long startTime = millis();
        unsigned long timeToStop = millis() + 1500;
        bool goBack = false;
        unsigned long timeSpent = 0;
        while (startTime < timeToStop)
        {
          Forward();
          delay(2);
          Obstacle = Obstacle_Avoider(10);
          if (Obstacle)
          {
            Stop();
            unsigned long currentTime = millis();
            timeSpent = currentTime - startTime;
            goBack = true;
            break;
          }
        }
        if (goBack)
        {
          Backward();
          delay(timeSpent);
          Stop();
        }
        Stop();
        REWARD = REWARDS[STATE][ACTION];
      }


      if (ACTION == 1)
      {
        led.setColor(5, 50, 50, 0);
        led.setColor(6, 50, 50, 0);
        led.show();
        Backward();
        delay(1500);
        Stop();
        REWARD = REWARDS[STATE][ACTION];
      }

      if (ACTION == 2)
      {
        led.setColor(5, 50, 50, 0);
        led.setColor(6, 50, 50, 0);
        led.setColor(7, 50, 50, 0);
        led.show();
        Stop();
        REWARD = REWARDS[STATE][ACTION];
      }

      if (ACTION == 3)
      {
        led.setColor(5, 50, 50, 0);
        led.setColor(6, 50, 50, 0);
        led.setColor(7, 50, 50, 0);
        led.setColor(8, 50, 50, 0);
        led.show();
        TurnLeft1();
        delay(650);
        Stop();
        REWARD = REWARDS[STATE][ACTION];
      }

      ACTION_TAKEN = true;
      led.setColor(5, 0, 0, 0);
      led.setColor(6, 0, 0, 0);
      led.setColor(7, 0, 0, 0);
      led.setColor(8, 0, 0, 0);
      led.show();
    }

    if (ACTION_TAKEN == true)
    {
      Update(Q, STATE, NEXT_STATE, ACTION , ACTIONS, REWARD, ALPHA , GAMMA);
      STATE = NEXT_STATE;
      EPSILON = DECAY(EPSILON);
      if (EPSILON < 0.5)
      {
        EPSILON  == 0.9;
      }
    }
  }
  /////////////////////////////////////END OF TRAINING///////////////////////////////////
  ////////////////////////////////////////TESTING////////////////////////////////////////////
  led.setColor(0, 255, 255, 0);
  led.show();
  while (true)
  {
    Forward();
    Obstacle = Obstacle_Avoider(40);
    if (Obstacle == true)
    {
      STATE = GET_STATE();
      ACTION = ARGMAX(Q, STATE);
      Serial.print("ACTION TAKEN: ");
      Serial.println(ACTION);

      if (ACTION == 0)
      {
        Forward();
        delay(1500);
        Stop();
      }

      if (ACTION == 1)
      {
        Backward();
        delay(500);
        Stop();
      }
      if (ACTION == 2)
      {
        Stop();
      }

      if (ACTION == 3)
      {
        TurnLeft1();
        delay(650);
        Stop();
      }
    }
  }
}


void setup()
{
  delay(5);
  Serial.begin(115200);
  delay(5);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  led.setpin(LED_PORT);
  buzzer.setpin(BUZZER_PORT);
  led.setColor(0, 0, 0, 0);
  led.show();
  buzzer.tone(100, 100);
  buzzer.noTone();

  //  lasttime_receive_cmd =  millis();
  //  while(millis() - lasttime_receive_cmd < 300)
  //  {
  //    if(Serial.available() > 0)
  //    {
  //      char c = Serial.read();
  //      if((c=='\n') || (c=='#'))
  //      {
  //        boot_show_flag = false;
  //        break;
  //      }
  //    }
  //  }

  // enable the watchdog
  wdt_enable(WDTO_2S);
  delay(5);

  wdt_reset();

  pinMode(13, OUTPUT);
  encoders[0] = MeEncoderMotor(SLOT_1);
  encoders[1] = MeEncoderMotor(SLOT_2);
  encoders[0].begin();
  encoders[1].begin();
  wdt_reset();
  //  if(boot_show_flag == true)
  //  {
  //    init_form_power();
  //  }
  wdt_reset();
  encoders[0].runSpeed(0);
  encoders[1].runSpeed(0);

  //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);

  leftflag = false;
  rightflag = false;
  //  PID_angle.Setpoint = RELAX_ANGLE;
  //  PID_angle.P = 17;          //17;
  //  PID_angle.I = 0;           //0;
  //  PID_angle.D = -0.2;        //-0.2  PID_speed.Setpoint = 0;
  //  PID_speed.P = -0.1;        // -0.1
  //  PID_speed.I = -0.008;      // -0.008
  //  readEEPROM();
  //  auriga_mode = BALANCED_MODE;
  //  update_sensor = lasttime_speed = lasttime_angle = millis();
  blink_time = millis();
  rxruntime = millis();
  randomSeed(analogRead(A0));
}



void loop()
{
  currentTime = millis() / 1000.0 - lastTime;
  keyPressed = buttonSensor.pressed();

  if (millis() - blink_time > 10000)
  {
    blink_time = millis();
    blink_flag = !blink_flag;
    digitalWrite(13, blink_flag);
  }

  wdt_reset();
  //if(ir != NULL)
  //{
  //  IrProcess();
  //}
  //  steppers[0].runSpeedToPosition();
  //  steppers[1].runSpeedToPosition();
  //  steppers[2].runSpeedToPosition();
  //  steppers[3].runSpeedToPosition();
  //  get_power();
  Encoder_1.loop();
  Encoder_2.loop();


  //  while(Serial.available() > 0)
  //  {
  //    char c = Serial.read();
  //    Serial.write(c);
  //    buf[bufindex++]=c;
  //    if((c=='\n') || (c=='#'))
  //    {
  //      parseCmd(buf);
  //      memset(buf,0,64);
  //      bufindex = 0;
  //    }
  //  }

  readSerial();
  while (isAvailable)
  {
    unsigned char c = serialRead & 0xff;
    if ((c == 0x55) && (isStart == false))
    {
      if (prevc == 0xff)
      {
        index = 1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if (isStart)
      {
        if (index == 2)
        {
          dataLen = c;
        }
        else if (index > 2)
        {
          dataLen--;
        }
        writeBuffer(index, c);
      }
    }
    index++;
    if (index > 51)
    {
      index = 0;
      isStart = false;
    }
    if (isStart && (dataLen == 0) && (index > 3))
    {
      isStart = false;
      parseData();
      index = 0;
    }
    readSerial();
  }

  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);
  //ultrCarProcess();
  learnObstacleAvoidence();
}


/// **** DRIVING FUNCTIONS - START **** ///
void isr_process_encoder1(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if (digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void Forward(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Backward(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnLeft(void)
{
  Encoder_1.setMotorPwm(moveSpeed / 4);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnRight(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 4);
}

void TurnLeft(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 2);
}

void TurnRight(void)
{
  Encoder_1.setMotorPwm(-moveSpeed / 2);
  Encoder_2.setMotorPwm(moveSpeed);
}

void TurnLeft1(void)
{
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}


void TurnRight1(void)
{
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Stop(void)
{
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void ChangeSpeed(int16_t spd)
{
  moveSpeed = spd;
}

/// **** DRIVING FUNCTIONS - END **** ///


/// **** SENSOR FUNCTIONS - START **** ///
void readSensor(uint8_t device)
{
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value = 0.0;
  uint8_t port, slot, pin;
  port = readBuffer(6);
  pin = port;
  switch (device)
  {
    case ULTRASONIC_SENSOR:
      {
        if (us == NULL)
        {
          us = new MeUltrasonicSensor(port);
        }
        else if (us->getPort() != port)
        {
          delete us;
          us = new MeUltrasonicSensor(port);
        }
        value = (float)us->distanceCm();
        writeHead();
        writeSerial(command_index);
        sendFloat(value);

      }
      break;

    case  LIGHT_SENSOR:
    case  SOUND_SENSOR:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(), INPUT);
        }
        value = generalDevice.aRead2();
        sendFloat(value);
      }
      break;

    case  JOYSTICK:
      {
        slot = readBuffer(7);
        if (joystick.getPort() != port)
        {
          joystick.reset(port);
        }
        value = joystick.read(slot);
        sendFloat(value);
      }
      break;

    case ENCODER_BOARD:
      {
        if (port == 0)
        {
          slot = readBuffer(7);
          uint8_t read_type = readBuffer(8);
          if (slot == SLOT_1)
          {
            if (read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_1.getCurPos());
            }
            else if (read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_1.getCurrentSpeed());
            }
          }
          else if (slot == SLOT_2)
          {
            if (read_type == ENCODER_BOARD_POS)
            {
              sendLong(Encoder_2.getCurPos());
            }
            else if (read_type == ENCODER_BOARD_SPEED)
            {
              sendFloat(Encoder_2.getCurrentSpeed());
            }
          }
        }
      }
      break;
  }//switch
}


void runModule(uint8_t device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  uint8_t port = readBuffer(6);
  uint8_t pin = port;
  switch (device)
  {
    case ENCODER_BOARD:
      if (port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);
        if (slot == SLOT_1)
        {
          Encoder_1.setTarPWM(speed_value);
        }
        else if (slot == SLOT_2)
        {
          Encoder_2.setTarPWM(speed_value);
        }
      }
      break;
    case JOYSTICK:
      {
        int16_t leftSpeed = readShort(6);
        Encoder_1.setTarPWM(leftSpeed);
        int16_t rightSpeed = readShort(8);
        Encoder_2.setTarPWM(rightSpeed);
      }
      break;

    case LIGHT_SENSOR:
      {
        if (generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        generalDevice.dWrite1(v);
      }
      break;

    case ENCODER_PID_MOTION:
      {
        uint8_t subcmd = port;
        uint8_t slot_num = readBuffer(7);
        if (ENCODER_BOARD_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          if (slot_num == SLOT_1)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
        }
        else if (ENCODER_BOARD_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);
          if (slot_num == SLOT_1)
          {
            Encoder_1.runSpeed((float)speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.runSpeed((float)speed_temp);
          }
        }
        else if (ENCODER_BOARD_PWM_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);
          if (slot_num == SLOT_1)
          {
            Encoder_1.setTarPWM(speed_temp);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.setTarPWM(speed_temp);
          }
        }
        else if (ENCODER_BOARD_SET_CUR_POS_ZERO == subcmd)
        {
          if (slot_num == SLOT_1)
          {
            Encoder_1.setPulsePos(0);
          }
          else if (slot_num == SLOT_2)
          {
            Encoder_2.setPulsePos(0);
          }
        }
        else if (ENCODER_BOARD_CAR_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          if (slot_num == 1)
          {
            Encoder_1.move(-pos_temp, (float)speed_temp);
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
          else if (slot_num == 2)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
            Encoder_2.move(-pos_temp, (float)speed_temp);
          }
          else if (slot_num == 3)
          {
            Encoder_1.move(-pos_temp, (float)speed_temp);
            Encoder_2.move(-pos_temp, (float)speed_temp);
          }
          else if (slot_num == 4)
          {
            Encoder_1.move(pos_temp, (float)speed_temp);
            Encoder_2.move(pos_temp, (float)speed_temp);
          }
        }
      }
      break;
  }
}
/// **** SENSOR FUNCTIONS - END **** ///


/// **** COMM FUNCTIONS - START **** ///


void readSerial(void)
{
  isAvailable = false;
  if (Serial.available() > 0)
  {
    isAvailable = true;
    serialRead = Serial.read();
  }
}

boolean read_serial(void)
{
  boolean result = false;
  readSerial();
  if (isAvailable)
  {
    uint8_t c = serialRead & 0xff;
    result = true;
    if ((c == 0x55) && (isStart == false))
    {
      if (prevc == 0xff)
      {
        index = 1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if (isStart)
      {
        if (index == 2)
        {
          dataLen = c;
        }
        else if (index > 2)
        {
          dataLen--;
        }
        writeBuffer(index, c);
      }
    }
    index++;
    if (index > 51)
    {
      index = 0;
      isStart = false;
    }
    if (isStart && (dataLen == 0) && (index > 3))
    {
      isStart = false;
      parseData();
      index = 0;
    }
    return result;
  }
}

void writeBuffer(int16_t index, uint8_t c)
{
  buffer[index] = c;
}


void parseData(void)
{
  isStart = false;
  uint8_t idx = readBuffer(3);
  uint8_t action = readBuffer(4);
  uint8_t device = readBuffer(5);
  command_index = (uint8_t)idx;
  switch (action)
  {
    case GET:
      {
        if (device != ULTRASONIC_SENSOR)
        {
          writeHead();
          writeSerial(idx);
        }
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        //reset
        /* Off on-Board LED lights */
        buzzer.setpin(BUZZER_PORT);
        led.setColor(0, 0, 0, 0);
        led.show();

        /* reset On-Board encoder driver */
        Encoder_1.setPulsePos(0);
        Encoder_2.setPulsePos(0);
        Encoder_1.moveTo(0, 10);
        Encoder_2.moveTo(0, 10);
        Encoder_1.setMotorPwm(0);
        Encoder_2.setMotorPwm(0);
        Encoder_1.setMotionMode(DIRECT_MODE);
        Encoder_2.setMotionMode(DIRECT_MODE);

        /* reset ext encoder driver */
        encoders[0].runSpeed(0);
        encoders[1].runSpeed(0);

        callOK();
      }
      break;
    case START:
      {
        //start
        callOK();
      }
      break;
  }
}

uint8_t readBuffer(int16_t index)
{
  return buffer[index];
}

void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

void writeSerial(uint8_t c)
{
  Serial.write(c);
}

void sendFloat(float value)
{
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

void writeEnd(void)
{
  Serial.println();
}

void sendLong(long value)
{
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

int16_t readShort(int16_t idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}

long readLong(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}

void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}
/// **** COMM FUNCTIONS - END **** ///
