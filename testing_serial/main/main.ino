int ctr = 0;
void setup() {
  Serial.begin(115200);
  while(!Serial) {} // Wait until serial connection is set
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg += " " + String(ctr);
    ctr++;
    Serial.println(msg);
  }
}