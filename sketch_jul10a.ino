#define PWR_CONTROL_PIN    5
#define WAVE_ZERO_DETECTOR 3
#define TACHO              2

/*
*   минимум - 80 об/мин - 37500 мкс -> 5 периодов
*   В течении этих периодов считаются импульсы с таходатчика -- частота вращения шпинделя
*   TIM1 управляет тиристором
*/

int tachoPulsesCounter = 0;
int periodsCounter = 0;

bool pinstate=false;

const uint8_t timer_div = (1 << CS11) | (1 << CS10); // Timer1 prescaler is 64

void setup() {
  Serial.begin(9600);
  // timer setup
  // Set Timer1 in CTC mode
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);
  // Enable compare match interrupt
  //TIMSK1 = (1 << OCIE1A);
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
  // Reset the timer count
  TCNT1 = 0;
  
  //TCCR1B |= (1 << CS11) | (1 << CS10);
  setFiringAngle(2000);
  // GPIO setup
  pinMode(PWR_CONTROL_PIN, OUTPUT);
  digitalWrite(PWR_CONTROL_PIN, LOW);
  pinMode(TACHO, INPUT);

  attachInterrupt(digitalPinToInterrupt(WAVE_ZERO_DETECTOR), ZC_detect, FALLING);       // Enable external interrupt (INT0) 
  attachInterrupt(digitalPinToInterrupt(TACHO), countTachoPulses, RISING);
}

void countTachoPulses() {
  tachoPulsesCounter++;
}

void ZC_detect() {
  TCCR1B |= timer_div; // включаем таймер
  periodsCounter++;
  //digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  
  //Serial.println("0");
}

// Timer1 compare match interrupt handler
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(PWR_CONTROL_PIN, HIGH); // включаем ножку
  //Serial.println("1");
  //inline delay(1);
  //delayMicroseconds(1000);
  //for (int i=0; i<200; i++){}
  //digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  //Serial.println("0");
  //TCCR1B &= ~timer_div; // останавливаем таймер 
}

ISR(TIMER1_COMPB_vect)
{
  TCCR1B &= ~timer_div; // останавливаем таймер 
  digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  //Serial.println("0");
}

void setFiringAngle(unsigned long microseconds)
{
  if (microseconds < 4000) {
    microseconds = 000;
  }

  if (microseconds > 9000) {
    microseconds = 9000;
  }

  // Convert microseconds to timer ticks
  const int on_time = 1000; // !!! перенести в init? (время запускающего импульса в мкс)
  unsigned long timerTicks = (microseconds * (F_CPU / 1000000UL)) / 64UL;
  unsigned long timerOFF = (on_time * (F_CPU / 1000000UL)) / 64UL; // !!! перенести в init?


  // Disable interrupts
  noInterrupts();

  // Set the compare match value
  OCR1A = timerTicks;
  OCR1B = timerOFF;  // !!! перенести в init?

  // Enable interrupts
  interrupts();
}

int normalizeFiringAngle () {
  //here we correcting value of 1 5000 to actual 4000 9000
}

void loop() {
  int a;

  if (periodsCounter >= 5) {
    setFiringAngle(calculateAngle());
    periodsCounter = 0;
  }

  if (Serial.available() > 0) {
    int delay = Serial.parseInt();
    if (Serial.read() == '\n') {
      setFiringAngle(delay);
    }
    else {
      Serial.println("хуйню сморозил");
    }
  }
}
