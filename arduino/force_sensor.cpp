#include <Wire.h>
#include "FX29K.h"
FX29K scale(FX29K0, 0010, &Wire);
void setup(){
  Wire.begin();
  Serial.begin(9600);
  scale.tare();
}
void loop(){
  uint16_t raw = scale.getRawBridgeData();
  float lb = scale.getPounds();
  Serial.print("Lbs:");
  Serial.print(lb);
  Serial.println(",");
  delay(50);
}