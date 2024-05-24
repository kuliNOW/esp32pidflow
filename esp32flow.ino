const int sensorPin = 13;
const int IN1 = 27;
const int IN2 = 26;
const int EN = 21;
const float setPoint = 0.2;
const float calibrationFactor = 4.5;
int defPWM = 125;
int pwmValue = 125;
unsigned long previousMillis = 0;
volatile unsigned long pulseCount = 0;

float Kp = 2.0;
float Ki = 0.5;
float Kd = 1.0;

float integral = 0;
float previousError = 0;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN, defPWM);
  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, RISING);
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long elapsedTime = currentMillis - previousMillis;

  if (elapsedTime >= 1000) {      
    detachInterrupt(digitalPinToInterrupt(sensorPin));

    float flowRate = pulseCount / calibrationFactor;
    float error = setPoint - flowRate;
    integral += error * elapsedTime / 1000.0;
    float derivative = (error - previousError) / (elapsedTime / 1000.0);
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    if (flowRate == 0.00) {
      pwmValue += 22.5;
      pwmValue = constrain(pwmValue, 165, 255);
      analogWrite(EN, pwmValue);
    } else {
      pwmValue = 165;
      analogWrite(EN, pwmValue);
    }
    Serial.print("Flow rate: ");
    Serial.print(flowRate);
    Serial.print(" L/min");
    Serial.print(" | PWM Value: ");
    Serial.print(pwmValue);
    Serial.print(" | pulseCount: ");
    Serial.print(pulseCount);
    Serial.print(" | ");
    Serial.println(pwmValue);

    pulseCount = 0;
    previousMillis = currentMillis;
    previousError = error;

    attachInterrupt(digitalPinToInterrupt(sensorPin), pulseCounter, RISING);
  }
}