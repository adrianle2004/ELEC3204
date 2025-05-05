float cycle = 10.0;
int buttonstate = 0;
float ratio = 0.3;
int encA, encB;
int dir = 1; // clock wise

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(9, OUTPUT); // ENA
  pinMode(A0, INPUT); // INA
  pinMode(2, INPUT);  // speed button
  pinMode(3, INPUT);  // direction button
  pinMode(13, OUTPUT); // Direction
  pinMode(12, OUTPUT); // Direction
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  buttonstate = digitalRead(2);
  if (buttonstate == HIGH) {
    if (ratio < 0.8){
      ratio += 0.1;
    }
    duty_cycle(ratio);
  } else {
    ratio = 0;
    duty_cycle(ratio);
  }
  //ratio = analogRead(A0)/1023.0;
  encA = analogRead(A1);
  encB = analogRead(A2);

  dir = digitalRead(3);
  direction(dir);


  Serial.print(cycle*ratio);
  Serial.print(",");
  Serial.print(dir);
  Serial.print(",");
  Serial.print(encA);
  Serial.print(",");
  Serial.println(encB);


}

void duty_cycle(float D){
  digitalWrite(9, HIGH);
  delay(cycle*D);
  digitalWrite(9, LOW);
  delay(cycle*D);
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
