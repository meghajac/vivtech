#define RX_Pin 0 // Bluetooth RX
#define TX_Pin 1 // Bluetooth TX
#define ItsCool 2 // Decrease Temp
#define ItsGettingHotinHere 3 // Increase Temp
#define DryerTime 4 // Decrease Humidity
#define RainOnMe 5 // Increase Humidity
#define TakePic 6 // Take a pic
#define LightItUp 7 // Turn on/off Light

void setup() {
  Serial.begin(115200); // Configured to match STM32
  pinMode(2, INPUT_PULLUP); // decrease temp button
  pinMode(3, INPUT_PULLUP); // increase temp button
  pinMode(4, INPUT_PULLUP); // decrease humidity button
  pinMode(5, INPUT_PULLUP); // increase humidity button
  pinMode(6, INPUT_PULLUP);// Take pic button
  pinMode(7, INPUT_PULLUP); // Turn light on/off
}

void loop() {
  if (digitalRead(2) == LOW) {
    Serial.write('A');
    delay(150);
  } else if (digitalRead(3) == LOW) {
    Serial.write('B');
    delay(150);
  } else if ((digitalRead(4) == LOW)) {
    Serial.write('C');
    delay(150);
  } else if ((digitalRead(5) == LOW)) {
    Serial.write('D');
    delay(150);
  } else if ((digitalRead(6) == LOW)) {
    Serial.write('E');
    delay(150);
  } else if ((digitalRead(7) == LOW)) {
    Serial.write('F');
    delay(150);
  }

}
