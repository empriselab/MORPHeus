#include <Wire.h>
#include "FX29K.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>

ros::NodeHandle node_handle;
using std_srvs::Empty;

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
  uint16_t raw = scale.getRawBridgeData();
  float kg = scale.getKilograms();

  force_msg.data = kg;
//   raw_msg.data = raw;

  Serial.print("Kgs:");
  Serial.print(kg);
  Serial.println(",");
  force_publisher.publish(&force_msg);
  node_handle.spinOnce();
 // delay(50);
}