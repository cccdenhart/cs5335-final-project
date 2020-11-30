int NUM_ACTIONS = 7;
float RG_BASE_VEL = 180;
float RG_MAX_VEL = 150;
float RG_MIN_VEL = -150;

int cur_vl;
int cur_vr;

void realize_action(int action)
{
  int pwm = 180;
  switch(action)
  {
    case 0: // move forward
    cur_vl = pwm;
    cur_vr = -pwm;
    break;

    case 1: // move backward
    cur_vl = -pwm;
    cur_vr = pwm;
    break;

    case 2: //Correct left 
    cur_vl =  pwm - pwm/4;
    cur_vr = -pwm;
    break;

    case 3: //Correct Right
    cur_vl = pwm;
    cur_vr = - pwm + pwm/4;
    break;

    case 4: // Turn Left
    cur_vl = pwm/8;
    cur_vr = -2*pwm;
    break;

    case 5: //Turn Right
    cur_vl = 2*pwm;
    cur_vr = -pwm/8;
    break;

    case 6: //swirl
    cur_vl = -2*pwm;
    cur_vr = -2*pwm;
    break;
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < NUM_ACTIONS; i++) {
    realize_action(i);
    Serial.print(cur_vl);
    Serial.print("\t");
    Serial.println(cur_vr);
  }
  delay(100000);
}
