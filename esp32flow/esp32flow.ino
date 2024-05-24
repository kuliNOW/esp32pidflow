#include <functional>

struct FlowSetting {
  const int sensorPin;
  const int IN1;
  const int IN2;
  const int EN;
  const float setPoint;
  const float calibrationFactor;
  const float pwmAdd;
  const int defPWM;
  const int minPWM;
  const int maxPWM;
  const float Kp;
  const float Ki;
  const float Kd;
};

class FlowControl {
  private:
    FlowSetting* sett;
    int pwmValue;
    unsigned long previousMillis;
    volatile unsigned long pulseCount;
    float integral;
    float previousError;

  public:
    FlowControl(FlowSetting* sett)
      : sett(sett), pwmValue(sett->defPWM), previousMillis(0), pulseCount(0), integral(0), previousError(0) {}

    void IRAM_ATTR pulseCounter() {
      pulseCount++;
    }

    void setup() {
      Serial.begin(115200);
      pinMode(sett->IN1, OUTPUT);
      pinMode(sett->IN2, OUTPUT);
      pinMode(sett->EN, OUTPUT);
      digitalWrite(sett->IN1, LOW);
      digitalWrite(sett->IN2, HIGH);
      analogWrite(sett->EN, sett->defPWM);
      pinMode(sett->sensorPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(sett->sensorPin), std::bind(&FlowControl::pulseCounter, this), RISING);
    }

    void loop() {
      unsigned long currentMillis = millis();
      unsigned long elapsedTime = currentMillis - previousMillis;

      if (elapsedTime >= 1000) {
        detachInterrupt(digitalPinToInterrupt(sett->sensorPin));

        float flowRate = pulseCount / sett->calibrationFactor;
        float error = sett->setPoint - flowRate;
        integral += error * elapsedTime / 1000.0;
        float derivative = (error - previousError) / (elapsedTime / 1000.0);
        float output = sett->Kp * error + sett->Ki * integral + sett->Kd * derivative;

        if (flowRate == 0.00) { 
          pwmValue += sett->pwmAdd; 
          pwmValue = constrain(pwmValue, sett->minPWM, sett->maxPWM);
        } else {
          pwmValue = sett->minPWM;
        }
        analogWrite(sett->EN, pwmValue);
        Serial.print("Flow rate: ");
        Serial.print(flowRate);
        Serial.print(" L/min");
        Serial.print(" | PWM Value: ");
        Serial.print(pwmValue);
        Serial.print(" | Pulse Count: ");
        Serial.println(pulseCount);
        
        pulseCount = 0;
        previousMillis = currentMillis;
        previousError = error;

        attachInterrupt(digitalPinToInterrupt(sett->sensorPin), std::bind(&FlowControl::pulseCounter, this), RISING);
      }
    }
};

// Inisialisasi pengaturan aliran Flow Control
FlowSetting sett = {
  .sensorPin = 13, //PIn untuk ke sensor Flow
  .IN1 = 27, // Pin untuk ke IN1 L298N
  .IN2 = 26, // Pin untuk ke IN2 L298N
  .EN = 21, // Pin untuk ke EN L298N
  .setPoint = 0.2, // setPoint
  .calibrationFactor = 4.5, //calibration factor default
  .pwmAdd = 22.5, // Nilai PWM yang digunakan untuk menambah PWM pada saat itu juga
  .defPWM = 125, // Dari pengguna minta 125
  .minPWM = 165, // PWM minim dari hasil kalibrasi
  .maxPWM = 255, // PWM maksimum dari hasil kalibrasi
  .Kp = 2.0,
  .Ki = 0.5,
  .Kd = 1.0
};

FlowControl flowControl(&sett);

void setup() {
  flowControl.setup();
}

void loop() {
  flowControl.loop();
}