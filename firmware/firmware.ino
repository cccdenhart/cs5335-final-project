
// ref: makeblock examples/.../Me_Auriga_encoder_pid_speed

#include <MeAuriga.h>

MeEncoderOnBoard E1(SLOT1);
MeEncoderOnBoard E2(SLOT2);

MeUltrasonicSensor Sonar(PORT_8);

char command[64];
char cmd_idx = 0;

int speed_L = 0;
int speed_R = 0;
int dist    = 0;

unsigned long last_tick = 0;

void
isr_e1()
{
    if(digitalRead(E1.getPortB()) == 0) {
        E1.pulsePosMinus();
    }
    else {
        E1.pulsePosPlus();;
    }
}

void
isr_e2()
{
    if(digitalRead(E2.getPortB()) == 0) {
        E2.pulsePosMinus();
    }
    else {
        E2.pulsePosPlus();;
    }
}

void
setup_pwm()
{
    // "Set PWM 8KHz", from the encoder PWM example
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);
    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
}

void
setup_enc(MeEncoderOnBoard* enc, void (*isr)(), int slot)
{
    //Need to reset to enable
    //https://github.com/Makeblock-official/Makeblock-Libraries/issues/67
    enc->reset(slot);
    attachInterrupt(enc->getIntNum(), isr, RISING);

    // values from example code
    enc->setPulse(9);
    enc->setRatio(39.267);
    enc->setPosPid(0.18,0,0);
    enc->setSpeedPid(0.18,0,0);
}

void
poll_serial()
{
    while (Serial.available()) {
        char cc = Serial.read();
        if (cc == '\n') {
            command[cmd_idx] = 0;
            sscanf(command, "%d %d", &speed_L, &speed_R);
            command[0] = 0;
            cmd_idx = 0;
            return;
        }
        else {
            command[cmd_idx++] = cc;
        }
    }
}

void
tick()
{
    char temp[100];
    snprintf(temp, 96, "%d %d %d", dist, speed_L, speed_R);
    Serial.println(temp);
}

void
maybe_tick()
{
    unsigned long now = millis();

    if (last_tick + 100 <= now) {
        tick();
        last_tick = now;
    }
}

void
setup()
{
    setup_pwm();
    setup_enc(&E1, isr_e1, SLOT1);
    setup_enc(&E2, isr_e2, SLOT2);
    Serial.begin(9600);
}

void
loop()
{
    dist = Sonar.distanceCm();

    poll_serial();
    maybe_tick();

    E1.runSpeed(-speed_L);
    E2.runSpeed(+speed_R);
    E1.loop();
    E2.loop();
}
