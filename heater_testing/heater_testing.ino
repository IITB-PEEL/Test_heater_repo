//i am makig this code as a version v2 for git testing
#include <Wire.h>                              //// WORKING
#include <Adafruit_MLX90614.h>

#define LED_PIN 25 // GPIO pin for the built-in LED
// === USER CONFIG ===
const int PWM_PIN = 4;       // PWM output pin to drive heater
float set_temp = 45.0;       // target temperature in °C

// === CONTROL VARIABLES ===
float present_temp = 0;//25.0;
float hist_temp = 0;//25.0;
float diff_temp = 0.0;
float hist_pwm_dc = 0.0;

unsigned long last_ms = 0;
const float dt = 0.1;  // control step ~100 ms
bool status = 0;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// === SENSOR FUNCTION (replace with your sensor read) ===
float Temprature() {
  // Example: return analogRead(A0) * scale;
  // Replace with your actual sensor code
  return mlx.readObjectTempC();
  // return 25.0 + random(-10,10)*0.01;  // dummy for now
}

// === CLAMP FUNCTION ===
float clamp_pwm(float x) {
  if (x > 100.0) return 100.0;
  if (x < 0.0) return 0.0;
  return x;
}

// === MAIN CONTROL TASK ===
void one_step() {
  present_temp = Temprature();
  diff_temp = present_temp - hist_temp;
  hist_temp = present_temp;

  if ((set_temp - diff_temp) > present_temp) {//here it was ">=" i rmoved "="
    // hist_pwm_dc = 100.0;
    //insted of this i have writw here a big algo for low speed update purpose.
    //i am commneting the abouve line.
  }
  else if(present_temp >= set_temp)
  {
    hist_pwm_dc = 0;
  }
   else {
    float temp_err = set_temp - present_temp;
    float denom = (fabs(diff_temp) > 1e-6) ? diff_temp : 1e-6;

    float new_pwm_dc = hist_pwm_dc * (temp_err / denom);

    // prevent crazy jumps
    // float max_rel_change = 2.0;                        //restore
    // if (new_pwm_dc > hist_pwm_dc * max_rel_change)
    //   new_pwm_dc = hist_pwm_dc * max_rel_change;
    // if (new_pwm_dc < hist_pwm_dc / max_rel_change)
    //   new_pwm_dc = hist_pwm_dc / max_rel_change;

    hist_pwm_dc = clamp_pwm(new_pwm_dc);
  }

  // === APPLY PWM TO HEATER ===
  int pwm_out = map((int)hist_pwm_dc, 0, 100, 0, 255);
  analogWrite(PWM_PIN, pwm_out);

  // === SERIAL DEBUG ===
  Serial.print(millis()/1000.0); Serial.print(",");
  Serial.print(present_temp);    Serial.print(",");
  Serial.print(set_temp);        Serial.print(",");
  Serial.println(hist_pwm_dc);
}

void pmd_test_code(void)
{
  
  while(1)
  {
    for(int i = 0; i < 256; i++)
    {
      analogWrite(PWM_PIN, i);
      delay(100);
    }
    digitalWrite(LED_PIN, status);
    status = !status;
  }
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  delay(2000);

  Wire1.setSDA(2);  // for example
  Wire1.setSCL(3);
  Wire1.setClock(100000);
  Wire1.begin();
  delay(1000);
  digitalWrite(LED_PIN, 1);
  if (!mlx.begin(0x5A, &Wire1)) {
    Serial.println("Sensor not found on Wire1");
    while (1);
  }
  digitalWrite(LED_PIN, 0);
  Serial.println("Sensor initialized!");

  Serial.println("time,temp,pwm");
}

void loop() {
  // pmd_test_code();
  if (millis() - last_ms >= (unsigned long)(dt * 1000)) {
    last_ms = millis();
    one_step();
    digitalWrite(LED_PIN, status);
    status = !status;
  }
}
