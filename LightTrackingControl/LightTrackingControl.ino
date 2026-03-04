#define SamplingFreqHz 1000
#define encoderPinA 0
#define encoderPinB 1
#define digPin 4
#define analogOutPin A0
#define analogReadPin A1

enum DirectionType {
  POSITIVE,
  NEGATIVE,
  ZERO
};

// encoder pulse counting
volatile int encoderPos = 0;
int prevEncoderPos = 0;

// advertising board rotation angle
double currAngleDeg = 0.0;
double prevAngleDeg = 0.0;

// cumulative reference angle and final PD control target angle
double targetAngleOffsetDeg = 0.0;
double controlReferenceDeg = 0.0;

double filteredLightVolt = 0.0;  // filtered light sensor value
double avgLightVolt = 0.0;       // ambient light average

// PD control parameters
double positionKp = 0.0;
double positionKd = 0.0;

// let the motor model be described b/(s+a)
const double motorTimeConstant = 181.8;  // [1/sec] a in b/(s+a)
const double motorGain = 1719000.0;      // [control units/sec] b in b/(s+a)

// control parameters
const double pd_Wn = 945.45;

const double ZERO_TOLERANCE_DEG = 0.1;

const double LIGHT_THRESHOLD = 0.03;
const double LIGHT_FILTER_COEFF = 0.102;

const double MAX_ANGLE_DEG = 65.0;

const double lightToAngleGain = 0.02;

const float VoltREF = 3.3;
const float ADC_MAX_VALUE = 1023.0;

// encoder parameter
const double ENCODER_PPR = 1400.0;

// status flags
int lightUpdateTimer = 0;  // Timer to control light sensor update frequency
int stuckTimer = 0;        // Timer to monitor if a mechanism is stuck

void setup() {
  Serial.begin(9600);

  // calculate the voltage caused by the current ambient light
  unsigned long startTime = millis();
  int nCount = 0;
  long int voltSumRaw = 0;

  while (millis() - startTime < 5000) {
    voltSumRaw += analogRead(analogReadPin);
    nCount++;
  }

  avgLightVolt = (VoltREF * voltSumRaw) / (ADC_MAX_VALUE * nCount);

  // set outputPin
  pinMode(digPin, OUTPUT);

  // turn on pull-up resistor
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  analogWriteResolution(12);
  analogWrite(analogOutPin, 500);  // set duty cycle

  // set up interrupts for encoder pins to handle position counting
  attachInterrupt(digitalPinToInterrupt(encoderPinA), UpdateEncoderCountFromA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), UpdateEncoderCountFromB, CHANGE);

  setup_timer4_freq(SamplingFreqHz);

  // let the desired closed-loop tranfer function be Wn^2/(s^2+2*Wn*s+Wn^2)
  positionKp = pd_Wn * pd_Wn / motorGain;
  positionKd = (2.0 * pd_Wn - motorTimeConstant) / motorGain;
}

void loop() {
  // control the auxiliary light: turn ON or OFF
  digitalWrite(digPin, LOW);
}

// update encoder position when trigger
void UpdateEncoderCountFromA() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}

// update encoder position when trigger
void UpdateEncoderCountFromB() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderPos--;
  } else {
    encoderPos++;
  }
}

DirectionType GetCurrDirectionType(double currAngleDeg, double tolerance) {
  if (currAngleDeg > tolerance) return POSITIVE;
  if (currAngleDeg < -tolerance) return NEGATIVE;
  return ZERO;
}

void HandleLightTrackingMode(double currLightVolt) {
  // first-order digital low-pass filter
  // smooth error signal to suppress high-frequency measurement noise
  // improve control stability and prevent actuator oscillation
  filteredLightVolt = LIGHT_FILTER_COEFF * (currLightVolt - avgLightVolt) + (1 - LIGHT_FILTER_COEFF) * filteredLightVolt;

  // adjust target angle
  if (filteredLightVolt > LIGHT_THRESHOLD) {
    targetAngleOffsetDeg += 0.25;
  } else if (filteredLightVolt < -LIGHT_THRESHOLD) {
    targetAngleOffsetDeg -= 0.25;
  }
  controlReferenceDeg = lightToAngleGain * filteredLightVolt + targetAngleOffsetDeg;
}

// determine rotation direction to return to zero position
void HandleZeroReturnMode() {
  DirectionType directionType = GetCurrDirectionType(currAngleDeg, ZERO_TOLERANCE_DEG);
  switch (directionType) {
    case POSITIVE:
      targetAngleOffsetDeg -= 0.05;
      controlReferenceDeg = lightToAngleGain * filteredLightVolt + targetAngleOffsetDeg;
      break;
    case NEGATIVE:
      targetAngleOffsetDeg += 0.05;
      controlReferenceDeg = lightToAngleGain * filteredLightVolt + targetAngleOffsetDeg;
      break;
    case ZERO:
      break;
  }
}

bool UpdateStuckState() {
  // compute current angle from encoder counts
  // phase A CHANGEs 1200 times per revolution
  currAngleDeg = (double)encoderPos * 360 / 1200.0;
  double deltaAngle = currAngleDeg - prevAngleDeg;

  // encoder output resolution (ppr) = 750 before quadrature decoding
  return abs(deltaAngle) < 0.01 && abs(currAngleDeg) > 7.0;
}

// detect motor stuck condition
bool UpdateControlMode(bool isStuck) {
  bool isLightTrackingControl = true;
  if (isStuck) {
    stuckTimer++;
    if (stuckTimer == 5000) {

      // disable light tracking to allow zero-return
      isLightTrackingControl = false;
      stuckTimer = 0;
    }
  } else {
    // reset stall counter
    stuckTimer = 0;
  }
  return isLightTrackingControl;
}

void UpdateReferenceByControlMode(bool isLightTrackingControl) {
  if (isLightTrackingControl) {
    double currLightVolt = VoltREF * analogRead(analogReadPin) / ADC_MAX_VALUE;
    HandleLightTrackingMode(currLightVolt);

    // return to zero position if angle exceeds defined limit range
    if (abs(currAngleDeg) > MAX_ANGLE_DEG) {
      HandleZeroReturnMode();
    }
  } else {
    // return to zero position if angle remains unchanged for more than 5 seconds
    HandleZeroReturnMode();
  }
}

double ComputePDOutput() {
  // velocity feedback
  double positionErrorDeg = controlReferenceDeg - currAngleDeg;
  double angularVelocityDegPerSec = (double)((encoderPos - prevEncoderPos))
                                    * 360.0 / ENCODER_PPR
                                    * SamplingFreqHz;

  return -positionKp * positionErrorDeg + positionKd * angularVelocityDegPerSec;
}

void ApplyControlOutput(double controlSignal) {
  double dacOutputVolt = -controlSignal * 0.2315 + 1.77;

  // convert control signal to DAC voltage for analog output
  analogWrite(analogOutPin, constrain((int)(dacOutputVolt * 1024.0 / VoltREF), 0, ADC_MAX_VALUE));
}

void RunPDControl() {
  bool isStuck = UpdateStuckState();
  bool isLightTrackingControl = UpdateControlMode(isStuck);

  // update light sensor reading at lower frequency
  lightUpdateTimer++;
  if (lightUpdateTimer < 10) {
    return;
  }
  lightUpdateTimer = 0;
  UpdateReferenceByControlMode(isLightTrackingControl);

  double controlSignal = ComputePDOutput();
  ApplyControlOutput(controlSignal);

  // save current encoder and angle for next iteration
  prevEncoderPos = encoderPos;
  prevAngleDeg = currAngleDeg;
}

void TC4_Handler() {
  if (!(TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)) {
    return;
  }
  RunPDControl();

  // clear interrupt flag
  REG_TC4_INTFLAG = TC_INTFLAG_OVF;
}

// round up to the next power of two (returns v_ if already power-of-two)
uint16_t next_pow2(uint16_t v_) {
  if (v_ == 0) {
    return 1;
  }
  --v_;
  v_ |= v_ >> 1;
  v_ |= v_ >> 2;
  v_ |= v_ >> 4;
  v_ |= v_ >> 8;
  return v_ + 1;
}

// select a suitable power-of-two prescaler for the target frequency.
uint16_t get_clk_div(uint32_t freq_) {
  float ideal_clk_div = 48000000.0f / (256.0f * float(freq_));
  uint16_t clk_div = next_pow2(uint16_t(ceil(ideal_clk_div)));

  // adjust to valid hardware-supported prescaler values
  switch (clk_div) {
    case 32: clk_div = 64; break;
    case 128: clk_div = 256; break;
    case 512: clk_div = 1024; break;
  }
  return clk_div;
}

// user-friendly version
void setup_timer4_freq(uint32_t freq_) {
  uint16_t clk_div = get_clk_div(freq_);
  uint8_t clk_cnt = (48000000 / clk_div) / freq_;
  setup_timer4(clk_div, clk_cnt);
}

void setup_timer4(uint16_t clk_div_, uint8_t count_) {
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |  // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;                                            // Wait for synchronization
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |      // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;  // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;                                     // Wait for synchronization
  REG_TC4_CTRLA |= TC_CTRLA_MODE_COUNT8;  // Set the counter to 8-bit mode
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY)
    ;                           // Wait for synchronization
  REG_TC4_COUNT8_CC0 = count_;  // Set the TC4 CC0 register to some arbitary value
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY)
    ;                                  // Wait for synchronization
  NVIC_SetPriority(TC4_IRQn, 0);       // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);            // Connect TC4 to Nested Vector Interrupt Controller (NVIC)
  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;   // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;  // Enable TC4 interrupts
  uint16_t prescale = 0;
  switch (clk_div_) {
    case 1: prescale = TC_CTRLA_PRESCALER(0); break;
    case 2: prescale = TC_CTRLA_PRESCALER(1); break;
    case 4: prescale = TC_CTRLA_PRESCALER(2); break;
    case 8: prescale = TC_CTRLA_PRESCALER(3); break;
    case 16: prescale = TC_CTRLA_PRESCALER(4); break;
    case 64: prescale = TC_CTRLA_PRESCALER(5); break;
    case 256: prescale = TC_CTRLA_PRESCALER(6); break;
    case 1024: prescale = TC_CTRLA_PRESCALER(7); break;
  }
  REG_TC4_CTRLA |= prescale | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_ENABLE;  // Enable TC4
  while (TC4->COUNT8.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
}