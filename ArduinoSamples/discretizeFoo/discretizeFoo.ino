int NUM_ACTIONS = 10;
float RG_BASE_VEL = 180;
float RG_MAX_VEL = 150;
float RG_MIN_VEL = -150;


int cur_vl;
int cur_vr;

void realize_action(int int_action) {
  switch (int_action) {
    case 0:     // move forward
      cur_vl = 200;
      cur_vr = -200;
      break;
    case 1:     // Hard left
      cur_vl = 100;
      cur_vr = 200;
      break;
    case 2:     // Hard right
      cur_vl = 132;
      cur_vr = 123;
      break;
    case 3:     // Veer left
      cur_vl = 123;
      cur_vr = 123;
      break;
    case 4:     // Veer right
      cur_vl = 123;
      cur_vr = 123;
      break;
    case 5:     // Swirl
      cur_vl = 123;
      cur_vr = 123;
      break;
  }
}


void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 10; i++) {
    realize_action(i);
  }
  delay(100000);
}
