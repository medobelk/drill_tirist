#define WAVE_ZERO_DETECTOR 2
#define PWR_CONTROL_PIN    8
#define SPEED_POT_PIN      0

#define TWICH_LOW_LIM      4300
#define TWICH_HIGH_LIM     1000

const uint8_t timer_div = (1 << CS11) | (1 << CS10); // Timer1 prescaler is 64
int speedPotVal = 0;
int speedPotValPrev = 0;
int speedMapped = 0;

void setup() {
  digitalWrite(13, HIGH);
  delay(800);
  digitalWrite(13, LOW);
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
  TCNT1 = 0;

  pinMode(PWR_CONTROL_PIN, OUTPUT);
  digitalWrite(PWR_CONTROL_PIN, LOW);

  const int on_time = 1500;
  unsigned long timerOFF = (on_time * (F_CPU / 1000000UL)) / 64UL;
  OCR1B = timerOFF; 
  setFiringAngle(4000);

  attachInterrupt(digitalPinToInterrupt(WAVE_ZERO_DETECTOR), ZC_detect, CHANGE);
}

void ZC_detect() {
  if bitRead(PIND, WAVE_ZERO_DETECTOR) { // if rising edge
    if (speedMapped > 5000) {            // if slow speed
      TCCR1B |= timer_div;
      PORTB &= B11111110;
    }
  }
  else {                                 // if falling edge
    if (speedMapped <= 5000) {           // if high speed
      TCCR1B |= timer_div;
      //PORTB &= B11111110;
    }
  }

}

ISR(TIMER1_COMPA_vect) {
  //digitalWrite(PWR_CONTROL_PIN, HIGH);
  PORTB |= B00000001;
}

ISR(TIMER1_COMPB_vect) {
  TCCR1B &= ~timer_div;
  //if (speedMapped < TWICH_LOW_LIM) digitalWrite(PWR_CONTROL_PIN, LOW);
  PORTB &= B11111110;
}

void setFiringAngle(unsigned long microseconds) {
  unsigned long timerTicks = (microseconds * (F_CPU / 1000000UL)) / 64UL;

  //noInterrupts();
  cli();
  OCR1A = timerTicks;
  sei();
  //interrupts();
}

void loop() {
  int speedPotDiff;

  speedPotVal = analogRead(SPEED_POT_PIN);
  speedPotDiff = abs(speedPotValPrev - speedPotVal);
  if (speedPotDiff >= 2 && speedPotDiff < 300) {
    speedMapped = map(speedPotVal, 0, 1023, 3500, 9500);
    setFiringAngle(speedMapped);
  }
  speedPotValPrev = speedPotVal;
  delay(20);
}