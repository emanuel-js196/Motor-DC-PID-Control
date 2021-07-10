#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>

// GPIO
//#define LED_BUILDIN 34
#define RPM_SENSOR 36
#define MOTOR_CONTROL 2

// GPIO PWM Settings
#define PWM_FREQ 1000
#define PWM_CHAN 0
#define PWM_RES 10                // Resolution in bits. Ej 10 bits -> (0-1023)
const int max_duty_cycle = 1023;  //(2^PWM_RES)-1;  
int pwm_duty_cycle = 0;
int pwm_duty_cycle_100 = 0;   // round((pwm_duty_cycle/max_duty_cycle)*100);

// RPM Sensor settings
int count_threshold = 50;               // hall trips
int count = 0;
bool new_rpm_data = true;              // True if fresh data is available
unsigned long last_rpm_sens_time = 0;

// Globals
int measured_rpm = 0;             // Global - Measured RPM value

// Interruption Service Rutine
void IRAM_ATTR rpm_sens() {
    count++;
}

// RPM Kaplan filter settings
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

// Set rpm to 0 if more than 300ms passed without changes in count variable
unsigned long max_wait_for_signal = 133333; // Max t threshold time (MIN RPM -> 900)
unsigned long last_signal_time = 0;
unsigned long last_measure_timestamp = 0;
int last_rpm_value = 0;

void measure_rpm_loop(){
  unsigned long current_time = micros(); 

  if (count >= count_threshold){  //  si la cnatidad de pulsos acumulados supera el threshold elegido, calculamos las RPM
    // calcular las rpm
    noInterrupts();
    float elapsed_time_f = (current_time - last_rpm_sens_time)/1000000.0;
    //float rpm_val = (count/elapsed_time_f) * 60.0;
    float rpm_val = (count/elapsed_time_f) * 30.0; 

    measured_rpm = round(rpm_val);

    // filtrarlas 
    if (measured_rpm >= 100){
      measured_rpm = round(pressureKalmanFilter.updateEstimate(measured_rpm));
    }else{
      measured_rpm=0;
    }

    count = 0;
    new_rpm_data = true;
    last_rpm_sens_time = micros();

    // Actualizar el timestamp de la ultima medición de velocidad  
    interrupts();

  } else { // En cambio, Si pasan mas de 300ms sin que cambie count, decimos que measured_rpm vale 0
    // Si pasaron 300ms / 300000 desde el ultimo cambio en rpm
    if ((current_time - last_rpm_sens_time) > max_wait_for_signal){
      //Serial.println("[measure_rpm_loop]: reached max_wait_for_signal. 'measured_rpm' forced to cero");
      //measured_rpm = round(pressureKalmanFilter.updateEstimate(0));
    }
  }
}

/* ******************* SETUP ******************* */
void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

  // GPIO declaration
  //pinMode(LED_BUILDIN, OUTPUT);
  pinMode(RPM_SENSOR, INPUT);
  //pinMode(MOTOR_CONTROL, OUTPUT); digitalWrite(MOTOR_CONTROL, LOW);

  // PWM setup
  ledcSetup(PWM_CHAN, PWM_FREQ, PWM_RES); // channel, freq, resolution_bits
  ledcAttachPin(MOTOR_CONTROL, PWM_CHAN); // pin, channel
  ledcWrite(PWM_CHAN, 0);                 // Start with MOTOR_CONTROL on LOW

  // Attach interrupts
  attachInterrupt(RPM_SENSOR, rpm_sens, FALLING);
}

int pwm = 400;

float rpmToPwm (int rpm){
    long double A = -0.000000000000240686463264349;
    long double B = 0.0000000112894079827467;
    long double C = -0.000117616413667552;
    long double D = 0.451751609695345;
    long double E = -181.957074041058;

    double y = A*(pow(rpm,4)) + B*(pow(rpm,3)) + C*(pow(rpm,2)) + D*rpm + E;

    return y;
} 

/* ******************* LOOP ******************* */
void loop(){
    for (int i = 3000; i<10000; i++){
        Serial.print("Input");
        Serial.print(i);
        Serial.print("/");
        Serial.println(rpmToPwm(i));
        delay(1);
    }


    // Measure RPM loop
    /* measure_rpm_loop();

    if(new_rpm_data && pwm <= 1023){
        ledcWrite(PWM_CHAN, pwm);                 // Start with MOTOR_CONTROL on LOW
        Serial.print(pwm);
        Serial.print("/");
        Serial.println(measured_rpm);
        new_rpm_data = false;
        pwm++;
    }
    if (pwm > 1024){
        ledcWrite(PWM_CHAN, 0);                 // Start with MOTOR_CONTROL on LOW
        delay(1000);
    } */

}