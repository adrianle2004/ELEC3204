float cycle = 100.0;
int buttonstate_on = 0;
int lastbuttonstate_on = 0;
int buttonstate_off = 0;
int lastbuttonstate_off = 0;

int switch_func = 0;     // 0 for close loop, 1 for open loop
int loop_state;
float motor_state = 0.0; // 0 mean stopping, 1 mean running - like a step function

float ratio = 0.5; // initial ratio
int encA, encB;
int dir = 0; // 1 for clock wise, 0 for anti clock wise

float desire_spd = 70.0; // initial desire spd
float desire_ratio;
float max_spd = 90.0;
float rpm;

int station_state = 0; // LDR input
int laststation_state = 0; // debounce
int station_button = 0; 
int station_check = 0; // 0 mean not in station, 1 vice versal

unsigned long lastButtonTime_on = 0;
unsigned long lastButtonTime_off = 0;
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds.

volatile unsigned int pulseCount = 0;
const unsigned int countsPerRev = 12;
unsigned long previousMillis = 0;
const unsigned long interval = 500; // 1 second

void countPulse() {
  pulseCount++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(10, INPUT);
  pinMode(9, OUTPUT); // ENA
  pinMode(A0, INPUT); // desire spd
  pinMode(6, INPUT);  // run button
  pinMode(7, INPUT);  // stop button
  pinMode(8, INPUT);  // 1st station
  pinMode(13, OUTPUT); // Direction
  pinMode(12, OUTPUT); // Direction
  pinMode(2, INPUT);   // encA
  pinMode(3, INPUT);   // encB
  pinMode(11, OUTPUT); // signal loop - 0 for close loop, 1 for open loop
  attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING);

}

void loop(){
  unsigned long currentMillis = millis();
  if (currentMillis - lastButtonTime_on > debounceDelay) {
    loop_state = digitalRead(10);
    if (loop_state == HIGH && lastbuttonstate_on == LOW) { // posedge button - pressed and released
      switch_func = !switch_func;

      lastButtonTime_on = currentMillis;
      lastbuttonstate_on = buttonstate_on;
    }
  }
  if (switch_func == 1){
    open_loop();
  } else {
    close_loop();
  }
  digitalWrite(11, switch_func);
}

void open_loop(){
  desire_ratio = analogRead(A0)/1023.0;
  unsigned long currentMillis = millis();
    // turning the motor on
  if (currentMillis - lastButtonTime_on > debounceDelay) {
    buttonstate_on = digitalRead(6);
    if (buttonstate_on == HIGH && lastbuttonstate_on == LOW) { // posedge button - pressed and released
      motor_state = 1.0;
      ratio = desire_ratio;
      if (station_check == 1){
        station_check = 0;
        dir = !dir;
      } 
    }
    lastButtonTime_on = currentMillis;
    lastbuttonstate_on = buttonstate_on;
  }
  if (currentMillis - lastButtonTime_off > debounceDelay) {
    buttonstate_off = digitalRead(7);
    if (buttonstate_off == HIGH && lastbuttonstate_off == LOW) { // posedge button - pressed and released
        motor_state = 0.0;
      }
    lastButtonTime_off = currentMillis;
    lastbuttonstate_off = buttonstate_off;
  }


  //ratio = analogRead(A0)/1023.0;
  encA = digitalRead(2);
  encB = digitalRead(3);

  if (currentMillis - station_button > debounceDelay) {
    station_state = digitalRead(8);
    if (station_state == HIGH && laststation_state == LOW) { // posedge button - pressed and released
      check_station();
    }
    station_button = currentMillis;
    laststation_state = station_state;
  }
  direction(dir);

  duty_cycle(ratio*motor_state);

  Serial.print("Open loop - ");
  Serial.print("Direction: ");
  Serial.print(dir);
  Serial.print(", desire_ratio: ");
  Serial.print(desire_ratio);
  Serial.print(", motor_state: ");
  Serial.print(motor_state);
  Serial.print(", Station_state: ");
  Serial.print(station_state);
  Serial.print(",");
  Serial.print(encA);
  Serial.print(",");
  Serial.println(encB);
  Serial.print("Station_check: ");
  Serial.println(station_check);

}

void close_loop() {
  // put your main code here, to run repeatedly:



  desire_ratio = analogRead(A0)/1023.0;
  desire_spd = max_spd*desire_ratio;
  
  encA = digitalRead(2);
  encB = digitalRead(3);
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    detachInterrupt(digitalPinToInterrupt(2)); // Temporarily disable interrupt

    rpm = ((float)(pulseCount/99) / (float)countsPerRev) * (60000.0 / (interval) ); // still 1 second, shorter interval
    Serial.print("Close loop - ");
    Serial.print("RPM: ");
    Serial.print(rpm);
    Serial.print(", Pulse count: ");
    Serial.print(pulseCount);
    Serial.print(",");
    Serial.print(cycle*ratio);
    Serial.print(", dir: ");
    Serial.print(dir);
    Serial.print(",");
    Serial.print("ratio: ");
    Serial.print(ratio);
    Serial.print(", desire_spd: ");
    Serial.print(desire_spd);
    Serial.print(", motor_state: ");
    Serial.print(motor_state);
    Serial.print(", Station_state: ");
    Serial.print(station_state);
    Serial.print(",");
    Serial.print(encA);
    Serial.print(",");
    Serial.println(encB);
    Serial.print("Station_check: ");
    Serial.println(station_check);

    pulseCount = 0; // Reset counter
    previousMillis = currentMillis;
    if (rpm != 0){
      control_loop(rpm);
    }
    attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING); // Re-enable interrupt
  }

  // package reach station
  if (currentMillis - station_button > debounceDelay) {
    station_state = digitalRead(8);
    if (station_state == HIGH && laststation_state == LOW) { // posedge button - pressed and released
      check_station();
    }
    station_button = currentMillis;
    laststation_state = station_state;
  }
  /*
  buttonstate = digitalRead(6);
  if (buttonstate == HIGH) {
    ratio = 1.0;
      duty_cycle(ratio);
    } else {
      ratio = 0.0;
      duty_cycle(ratio);
    }
    */

  
  duty_cycle(ratio*motor_state);

  // turning the motor on
  if (currentMillis - lastButtonTime_on > debounceDelay) {
    buttonstate_on = digitalRead(6);
    if (buttonstate_on == HIGH && lastbuttonstate_on == LOW) { // posedge button - pressed and released
      motor_state = 1.0;
      if (station_check == 1){
        station_check = 0;
        ratio = 0.5;
        dir = !dir;
      }
      
    }
    lastButtonTime_on = currentMillis;
    lastbuttonstate_on = buttonstate_on;
  }
  
  // turning the motor off
  if (currentMillis - lastButtonTime_off > debounceDelay) {
    buttonstate_off = digitalRead(7);
    if (buttonstate_off == HIGH && lastbuttonstate_off == LOW) { // posedge button - pressed and released
      motor_state = 0.0;
    }
    lastButtonTime_off = currentMillis;
    lastbuttonstate_off = buttonstate_off;
  }

  //dir = digitalRead(7);
  direction(dir);
}


void duty_cycle(float D){
  int duty = int(D * 255.0);
  analogWrite(9, duty);
}


void direction(int dir){
  if (dir == 1){ // clock wise
    digitalWrite(13, HIGH);
    digitalWrite(12, LOW);
  } else if (dir == 0){ // anti clock wise
    digitalWrite(13, LOW);
    digitalWrite(12, HIGH);
  }
}


void check_station(){
  ratio = 0;
  station_check = 1;
  //dir = !dir;
}


void control_loop(float spd) {
  float error = desire_spd - spd; // Calculate the difference between desired and actual speed.
  float Kp = 0.01;  // Proportional constant

  ratio = ratio + Kp * error; // Adjust ratio based on the error
  if (station_check == 1){
    ratio = 0;
  }
  // Ensure the ratio stays between 0 and 1.
  if (ratio > 1.0) ratio = 1.0;
  if (ratio < 0.0) ratio = 0.0;
}
