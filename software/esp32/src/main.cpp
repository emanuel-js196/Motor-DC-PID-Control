#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>

/*
Next version:
  - Agregar la curva de velocidad seteada al gráfico del hmi
  - Implementar el PID
*/

// Network credentials
const char* ssid = "GS-TP";
const char* password = "$rivera40";

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
bool new_rpm_data = false;              // True if fresh data is available
unsigned long last_rpm_sens_time = 0;

// RPM Kaplan filter settings
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

// RPM/PID Control settings
#define MAX_SET_RPM 10000
#define MIN_SET_RPM 1000
#define SPEED_INTERVAL 500

float kp = 3.4; //0.0548;
float ki = 0;//0.04;
float kd = 0;//0.00017;

int measured_rpm = 0;             // Global - Measured RPM value
int set_rpm = 6000;

// Server config
#define HTTP_PORT 80
AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");

// Interruption Service Rutine
void IRAM_ATTR rpm_sens() {
    count++;
}

// Function prototipes
String collect_data ();

bool MOTOR_STATUS = false; // false STOP | true START

// Process data from HMI
void motor_management(char *data){
  Serial.println("[motor_management]: Recieved: " + String(data));

  if (strcmp((char*)data, "STOP") == 0) {
    //digitalWrite(MOTOR_CONTROL, LOW);
    MOTOR_STATUS = false;

  }else if (strcmp((char*)data, "START") == 0){
    //digitalWrite(MOTOR_CONTROL, HIGH);
    MOTOR_STATUS = true;

  }else if (strchr((char*)data, '/') != NULL){    // PID config is recieved as JSON
    char control[5] = {'\0'};
    float value_i;

    sscanf(data, "%[^/]/%f", control, &value_i); // Example: "fkp/100"

    if(strcmp((char*)control, "pid_set_rpm") == 0){
        set_rpm = value_i; Serial.println(set_rpm);

    }else if(strcmp((char*)control, "fkp") == 0){
        kp = value_i; Serial.println(kp);

    }else if (strcmp((char*)control, "fki") == 0){
        ki = value_i; Serial.println(ki);

    }else if (strcmp((char*)control, "fkd") == 0){
        kd = value_i; Serial.println(kd);

    }else{
      Serial.println("[motor_management]: PID Command: " + String(control) + "not handled");
    }

  }else if (strcmp((char*)data, "Vset_UP") == 0){
    // Set +1000 RPM to v_set
    set_rpm += SPEED_INTERVAL;
    set_rpm = constrain(set_rpm, MIN_SET_RPM, MAX_SET_RPM);
    //Serial.println(set_rpm);

  }else if (strcmp((char*)data, "Vset_DOWN") == 0){
    // Set -1000 RPM to v_set
    set_rpm -= SPEED_INTERVAL;
    set_rpm = constrain(set_rpm, MIN_SET_RPM, MAX_SET_RPM);
    //Serial.println(set_rpm);
  }
  else{
    Serial.println("[motor_management]: Command: " + String((char*)data) + "not handled");
  }

  // Feed new data to the HMI if something changed
  ws.textAll(collect_data()); // Send PID constants to the HMI
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    motor_management((char*)data);
  }
}

// Get all the data to fill up the dashboard in JSON string
String collect_data (){
  pwm_duty_cycle_100 = round((pwm_duty_cycle/max_duty_cycle)*100);

  Serial.print("pwm_duty_cycle_100: ");
  Serial.println(pwm_duty_cycle_100);

  String buf; 
  StaticJsonDocument<300> json_data;

  json_data["voltage"] = 0.00;
  json_data["current"] = 0.00;
  json_data["v_set"] = set_rpm;
  json_data["v_mes"] = measured_rpm;
  json_data["power"] = 0;
  json_data["pwm_dc"] = pwm_duty_cycle; // round((pwm_duty_cycle/max_duty_cycle)*100);
  json_data["spin"] = (random(1) ? "CW" : "CCW");
  json_data["status"] = (random(1) ? "Running" : "Stopped");
  json_data["kp"] = kp;
  json_data["ki"] = ki;
  json_data["kd"] = kd;

  serializeJson(json_data, buf);

  return buf;
}

// WebSockets onEvent
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      ws.textAll(collect_data()); // Send PID constants to the HMI
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
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

  //Initialize SPIFFS
  if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

  // WebSocket server init
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/page.html");
  });

  // Start server
  server.begin();

  // Attach interrupts
  attachInterrupt(RPM_SENSOR, rpm_sens, FALLING);
}

// Set rpm to 0 if more than 300ms passed without changes in count variable
unsigned long max_wait_for_signal = 500000; // Max t threshold time
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
      measured_rpm = round(pressureKalmanFilter.updateEstimate(0));
    }
  }
}

// int new_pulses_amount = 0;
// Angular velocity calculation timing function
//unsigned long previousMillis = 0;
//unsigned long currentMillis = 0;
// const long interval = 1000;

// RPM Timing function
//const long period = 1000;
//int new_measured_rpm = 0;
unsigned long pid_current_time, pid_previous_time=0;
double error, cumError, rateError, lastError;

/*
//VARIABLES DE CONTROL
float kp = 1.0;
float ki = 0.0;
float kd = 0.0;

int measured_rpm = 0;             // Global - Measured RPM value
int set_rpm = 0;
*/

float rpmToPwm (int rpm){

    // Polinomica de segundo orden:
    // y=0.000042550446584393300000000000x-0.446400016758173000000000000000x+1,555.009874243100000000000000000000

    // Polinomica de cuarto orden:

    long double A = -0.000000000000240686463264349;
    long double B = 0.0000000112894079827467;
    long double C = -0.000117616413667552;
    long double D = 0.451751609695345;
    long double E = -181.957074041058;

    double y = A*(pow(rpm,4)) + B*(pow(rpm,3)) + C*(pow(rpm,2)) + D*rpm + E;

    return y;
} 

void pid_loop(){
  int output;

  if (MOTOR_STATUS){
    pid_current_time = millis();
    double elapsed_time = (double)(pid_current_time-pid_previous_time);

    // Error 
    /*
    if (measured_rpm <= set_rpm) {
      error = set_rpm - measured_rpm;  

    }else if (measured_rpm >= set_rpm){
      error = measured_rpm - set_rpm;  

    }else{
      error = 0;

    }*/

    // Pruebita:
    error = (set_rpm+1000) - measured_rpm;
    cumError += error * elapsed_time;
    rateError = (error - lastError) / elapsed_time;

    // PID Output (in RPM)
    output = kp*error + ki*cumError + kd*rateError;

    // Store the max pid
    //if (output >= pid_max)      pid_max=output;}

    // Scale output down to PWM 
    //output = round(300 + ((1023-300)*(output/pid_max)));              // Valor = Malor_min + (PWM - Valor_min)*(pid/PIDmax)

    output = (int)rpmToPwm(output);

    // Store the data for t+1
    lastError = error;
    pid_previous_time = pid_current_time;

    Serial.print("[PWM]: Output: ");
    Serial.println(output);

    // Limit output to PWM friendly values
    output = constrain(output, 0, max_duty_cycle);
  }else{
    output = 0;
  }

  pwm_duty_cycle = output;

  ledcWrite(PWM_CHAN, pwm_duty_cycle);

  // Write PWM -> ledcWrite(PWM_CHAN, 0);              // Start with MOTOR_CONTROL on LOW

  // Report new duty cycle -> Store DutyCycle in global and set new data to true
  //pwm_duty_cycle_100 = round((pwm_duty_cycle/max_duty_cycle)*100);
  //new_rpm_data = true;
}


/* ******************* LOOP ******************* */
void loop(){
  ws.cleanupClients();

  // Measure RPM loop
  measure_rpm_loop();

  // Feed the pid
  pid_loop();

  // If new data was adquired report to WebSocket
  if(new_rpm_data){
    ws.textAll(collect_data());
    new_rpm_data = false;
  }
}

