#include <Wire.h>
#include "FX29K.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle node_handle;

std_msgs::Float32 force_msg;
// std_msgs::UInt16 raw_msg;

FX29K scale(FX29K0, 0010, &Wire);

void tare_callback(){
    scale.tare();
}

ros::Publisher force_publisher("force", &force_msg);
ros::ServiceServer<Empty::Request, Empty::Response> server("tare",&tare_callback);



void setup(){
  Wire.begin();
  Serial.begin(9600);
  scale.tare();
  node_handle.initNode();
  node_handle.advertise(force_publisher);
  node_handle.advertiseService(server);
}

void loop(){
  // @TODO(Luke) change lbs to kg or N
  uint16_t raw = scale.getRawBridgeData();
  float lb = scale.getPounds();

  force_msg.data = lb;
//   raw_msg.data = raw;

  Serial.print("Lbs:");
  Serial.print(lb);
  Serial.println(",");
  node_handle.spinOnce();
  delay(50);
}