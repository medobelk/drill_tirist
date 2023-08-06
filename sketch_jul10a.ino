#define TACHO              3
#define WAVE_ZERO_DETECTOR 2
#define PWR_CONTROL_PIN    4

#define ENABLE_PIN         7
#define GET_REVERSE_PIN    8

#define BRAKE_PIN          6
#define REVERSE_PIN        5
#define SPEED              A0
#define OVERLOAD_LED_PIN   A1

#define PWM_TEST   13

/*
*   ЕГОР ДОЛЖЕН ОТТЕСТИТЬ НАРМАЛЬНА НА АБАРУДОВАНИИ ФУНКЦИЮ setFiringAngle
*   минимум - 80 об/мин - 37500 мкс -> 5 периодов
*   В течении этих периодов считаются импульсы с таходатчика -- частота вращения шпинделя
*   TIM1 управляет тиристором
*
*   не останавливать таймер а в прерывании не включать ножку если двигатель на тормозе
*
*
*/

int tachoPulsesCounter = 0;
int periodsCounter = 0;
float detectedMinTacho = 100.0;
float detectedMaxTacho = 255.0;
int startDelay = 5000;
int newDelay;
int overloadsCount = 0;
int overloadsReverseCount = 0;

float previousIntegralError, previousDerrivativeError, previousTachoPulsesCounter;

const int delayMinMicroseconds = 2000;
const int delayMaxMicroseconds = 5000;
const int periodsBeforeCalculation = 15;
const int overloadMaxTries = 2;
const int overloadReverseMaxTries = 4;

const int delaySpinUpBias = 50;

int spinUpCurrentDelay = delayMaxMicroseconds;

int pidMeasureInterval = periodsBeforeCalculation * 10; //in milis
const uint8_t timer_div = (1 << CS11) | (1 << CS10); // Timer1 prescaler is 64

bool pinstate = false;
bool spinUpStatus = false;
bool isEnabled = false;
bool isOverloaded = false;
bool isOnBrakes = false;
bool isReversed = false;
bool timerStatus = false;
bool reversing = false;
bool a = true;

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

  // setFiringAngle(5000);
  // GPIO setup
  pinMode(PWR_CONTROL_PIN, OUTPUT);
  digitalWrite(PWR_CONTROL_PIN, LOW);
  pinMode(TACHO, INPUT);
  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(GET_REVERSE_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, OUTPUT);
  digitalWrite(BRAKE_PIN, HIGH);
  pinMode(REVERSE_PIN, OUTPUT);
  digitalWrite(REVERSE_PIN, LOW);
  pinMode(WAVE_ZERO_DETECTOR, INPUT_PULLUP);

  Serial.println("start");

  attachInterrupt(digitalPinToInterrupt(WAVE_ZERO_DETECTOR), ZC_detect, FALLING);       // Enable external interrupt (INT0) 
  attachInterrupt(digitalPinToInterrupt(TACHO), countTachoPulses, RISING);
}

void countTachoPulses() {
  tachoPulsesCounter++;
}

inline void startPowerDelayTimer() {
  timerStatus = true;
}

inline void stopPowerDelayTimer() {
  timerStatus = false;
}

void ZC_detect() {
  TCCR1B |= timer_div; //включаем таймер
  periodsCounter++;
  //digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  
  //Serial.println("0");
}

// Timer1 compare match interrupt handler
ISR(TIMER1_COMPA_vect) {
  if (timerStatus) {
    digitalWrite(PWR_CONTROL_PIN, HIGH); // включаем ножку
  }
  // Serial.println(6);
  //inline delay(1);
  //delayMicroseconds(1000);
  //for (int i=0; i<200; i++){}
  //digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  //Serial.println("0");
  //TCCR1B &= ~timer_div; // останавливаем таймер 
}

ISR(TIMER1_COMPB_vect) {
  TCCR1B &= ~timer_div; // останавливаем таймер 
  digitalWrite(PWR_CONTROL_PIN, LOW); // выключаем ножку
  //Serial.println("0");
}

void setFiringAngle(unsigned long microseconds) {
  // Serial.print("setting delay to: ");
  // Serial.println(microseconds);

  // Convert microseconds to timer ticks
  const int on_time = 1000; // !!! перенести в init? (время запускающего импульса в мкс)
  unsigned long timerTicks = (microseconds * (F_CPU / 1000000UL)) / 64UL;
  unsigned long timerOFF = (on_time * (F_CPU / 1000000UL)) / 64UL; // !!! перенести в init?


  // Disable interrupts - заменить на TIMSK1 = 0
  noInterrupts();

  // Set the compare match value
  OCR1A = timerTicks;
  OCR1B = timerOFF;  // !!! перенести в init?

  // Enable interrupts - заменить на TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B);
  interrupts();
}

void motorBrakes(bool brakeSwitch, String dbgMsg = "") {
  if (brakeSwitch && !isOnBrakes) {
    stopPowerDelayTimer();
    delay(50);
    digitalWrite(BRAKE_PIN, HIGH);
    Serial.println("on brakes");

    isOnBrakes = true;
    spinUpStatus = false;
  } 

  if (!brakeSwitch && isOnBrakes) {
    digitalWrite(BRAKE_PIN, LOW);
    delay(50);
    startPowerDelayTimer();

    Serial.println("no brakes");
    isOnBrakes = false;
  }

  if (dbgMsg.length() > 0) {
    Serial.println("brakes func msg: " + dbgMsg);
  }
}

bool spinUp(int desiredDelay) {
  motorBrakes(false);
  // startPowerDelayTimer();
  // timerStatus = true;
  
  if (desiredDelay < spinUpCurrentDelay && spinUpCurrentDelay - desiredDelay > delaySpinUpBias) {
    spinUpCurrentDelay = spinUpCurrentDelay - delaySpinUpBias;

    setFiringAngle(spinUpCurrentDelay);
    Serial.println("spinning up");

    return false;
  }

  spinUpCurrentDelay = delayMaxMicroseconds;
  newDelay = desiredDelay;

  setFiringAngle(desiredDelay);

  // Serial.println("spin up finished");
  return true;
}

bool detectOverload() {
  // Serial.println("overload func vars");
  // Serial.println(isEnabled);
  // Serial.println(tachoPulsesCounter);

  if (isOverloaded) {
    return true;
  }

  // if (isEnabled && spinUpStatus) {
  if (isEnabled) {
    if (tachoPulsesCounter == 0 && previousTachoPulsesCounter == 0) {
      overloadsCount++;

      if (overloadsCount >= overloadMaxTries) {
        Serial.println("overloaded");
        overloadsCount = 0;

        return true;
      }
      // Serial.println("overloaded");
      // overloadsCount = 0;

      // return true;
    }
  }

  return false;
}

void setMotorToReverse(bool reverseSwitch) {
  if (reverseSwitch && !isReversed) {
    digitalWrite(REVERSE_PIN, HIGH);
    Serial.println("reversing");
    isReversed = true;
  }
  
  if (!reverseSwitch && isReversed) {
    digitalWrite(REVERSE_PIN, LOW);
    Serial.println("reverse disabled");
    isReversed = false;
  }
}

bool checkReverse(int reverse) {
  if (isOverloaded) {
    if (reverse) {
      setMotorToOverloadReverse();
    }

    return false;
  }

  if (reverse && !isReversed) {
    if (tachoPulsesCounter == 0 && previousTachoPulsesCounter == 0) {
      setMotorToReverse(true);
    } else {
      motorBrakes(true, "reversing");

      return true;
    }
  }

  if (!reverse && isReversed) {
    if (tachoPulsesCounter == 0 && previousTachoPulsesCounter == 0) {
      setMotorToReverse(false);
    } else {
      motorBrakes(true, "no reversing");

      return true;
    }
  }

  return false;
}

void setMotorToOverloadReverse() {
  if (tachoPulsesCounter > 0) {
    isOverloaded = false;
    overloadsCount = 0;
    overloadsReverseCount = 0;
  } else {
    // Serial.println("reverse unblock");

    if (!isReversed) {
      motorBrakes(false);
      digitalWrite(REVERSE_PIN, HIGH);
      setFiringAngle(delayMinMicroseconds + (delayMaxMicroseconds - delayMinMicroseconds) / 2);
    }

    if (overloadsReverseCount >= overloadReverseMaxTries) {
      motorBrakes(true);
      digitalWrite(REVERSE_PIN, LOW);
    }

    overloadsReverseCount++;
    isReversed = true;
  }
}

int normalizeFiringAngle() {
  //here we correcting value of 1 5000 to actual 4000 9000
}

float normalizeTachoFrequencyToDelayTime(float tachoFrequency) {
  return (detectedMaxTacho * delayMaxMicroseconds) / delayMinMicroseconds;
}

float normalizeSpeedToFrequency(int setSpeed) {
  // return (setSpeed - 1023) * (detectedMaxTacho - detectedMinTacho) / (1023 - 0) + detectedMinTacho;
  // return detectedMaxTacho / 5 * setSpeed / 1023.0;
  return map(setSpeed, 0, 1023, detectedMinTacho, detectedMaxTacho);
}

float pid(float actualPoint, float setPoint, float p, float i, float d, int iterationTime) {
  float error, integral, derrivative, output;

  error = setPoint - actualPoint;
  // Serial.println("er");
  // Serial.println(setPoint);
  // Serial.println(actualPoint);
  // Serial.println(error);
  integral = previousIntegralError + error * iterationTime;
  derrivative = (error - previousDerrivativeError) / iterationTime;
  output = p * error + i * integral + d * derrivative;
  previousDerrivativeError = derrivative;
  previousIntegralError = integral;
  // Serial.println(output);
  return output;
}

void loop1() {
  int a;
  a = digitalRead(WAVE_ZERO_DETECTOR);
  Serial.println(a);
}

void loop() {
  // Serial.println(timerStatus);
  int setSpeed, desiredDelay, enable, reverse;
  float tachoFrequency, delayChange, desiredFrequency;

  enable = digitalRead(ENABLE_PIN);
  // Serial.println(enable);
  reverse = !digitalRead(GET_REVERSE_PIN);
  setSpeed = analogRead(SPEED);
  desiredDelay = map(setSpeed, 0, 1023, delayMaxMicroseconds, delayMinMicroseconds);

  if (enable == 1) {
    isEnabled = true;
    // Serial.println("enabled");
  } else {
    isEnabled = false;

    if (isOverloaded) {
      isOverloaded = false;
    }

    motorBrakes(true, "not enabled");
    // Serial.println("disabled");
    return;
  }

  if (setSpeed <= 15) {
    isEnabled = false;

    if (isOverloaded) {
      isOverloaded = false;
    }

    motorBrakes(true, "not enabled no speed");
    // Serial.println("disabled no speed");

    return;
  }

  reversing = checkReverse(reverse);

  // Serial.println(spinUpStatus);

  if (!spinUpStatus && !isOverloaded && !reversing) {
    spinUpStatus = spinUp(desiredDelay);
  }

  if (periodsCounter >= periodsBeforeCalculation) { //interval =  periodsBeforeCalculation * 10 ms
    if (spinUpStatus || reversing) {
      isOverloaded = detectOverload();
      // isOverloaded = false;

      if (isOverloaded) {
        if (!reverse) {
          motorBrakes(true, "overloaded");
        }
        
        return;
      }

      // motorBrakes(false);

      // setFiringAngle(speedTest);
      // setFiringAngle(calculateAngle());
      previousTachoPulsesCounter = tachoPulsesCounter;
      desiredFrequency = normalizeSpeedToFrequency(setSpeed) / (float)periodsCounter;

      tachoFrequency = tachoPulsesCounter / (float)periodsCounter;
      delayChange = pid(tachoFrequency, desiredFrequency, 50, 0, 0, pidMeasureInterval);

      newDelay = newDelay + round(delayChange * -1); //-1 to reverse cause less value if 4000 means more speed and vice versa

      if (newDelay < delayMinMicroseconds) {
        newDelay = delayMinMicroseconds;
      }

      if (newDelay > delayMaxMicroseconds) {
        newDelay = delayMaxMicroseconds;
      }

      setFiringAngle(newDelay);
    }

    periodsCounter = 0;
    tachoPulsesCounter = 0;
  }

  // if (Serial.available() > 0) {
  //   int delay = Serial.parseInt();
  //   if (Serial.read() == '\n') {
  //     setFiringAngle(delay);
  //   } else {
  //     Serial.println("хуйню сморозил");
  //   }
  // }

  // if (periodsCounter > 50) {
  //   periodsCounter = periodsBeforeCalculation + 1;
  // }

  // if (tachoPulsesCounter > 1000) {
  //   tachoPulsesCounter = 0;
  // }
}
