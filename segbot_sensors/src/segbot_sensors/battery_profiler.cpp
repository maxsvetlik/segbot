/* Battery profiler node: 
 * A battery profile only needs to be build after a robot has a different battery installed, or after *some time* 
 * such that the age & use of the battery has changed its power/voltage curve (rendering the old profile inaccurate)
 *
 * Invokes a back-and-forth task to keep the battery under load.
 * Listens to the diagnostic voltage publisher so that input voltages have a weighted average already applied from battery dianostics.
 * Once battery runs down completely:
 * Outputs a comma delimited file titled battery_profile.csv in config with function weights in a format to be determined.
 *
 * Author: Maxwell J. Svetlik, 2015
 */

//TODO: add time calculations, get actual topic name, add getHostname functionality

#include "ros/ros.h"
#include "ros/time.h"
#include <vector>
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/KeyValue.h"
#include "smart_battery_msgs/SmartBatteryStatus.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <math.h>
#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/package.h>
#include <string>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

int sample_frequency = 1; //frequency in hz; assume at least 5 hours of data
double sum_v, sum_v2, sum_time, sum_vtime, a, b, A;
bool complete;
int n = 0;
std::vector<std::string> voltages;
std::vector<double> times;
double start;
std::string path = ros::package::getPath("segbot_sensors");
std::ofstream file;


void writeToFile(double v, double t){
    //std::ofstream file;
    file << v << "," << t;
}

//compute curve fitting sums while voltage exists and is above 10 
void voltagecb(diagnostic_msgs::DiagnosticArray msg){
    for(int i = 0; i < msg.status.size(); i++){
        if( !msg.status.at(i).name.compare("/Battery/voltage")){
            //voltages.push_back(msg.status.at(i).values.at(0).value);
            //times.push_back(ros::Time::now().toSec() - start);
            double v = std::atof(msg.status.at(i).values.at(0).value.c_str());
            double t = ros::Time::now().toSec() - start;
            writeToFile(v,t);
            ROS_INFO("Got Voltage: %s at time %f", msg.status.at(i).values.at(0).value.c_str(), ros::Time::now().toSec() - start);
        }
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "battery_profiler");
  ros::NodeHandle n;
  ros::NodeHandle privateNode("~");
  std::string loc_a, loc_b;
  privateNode.param<std::string>("a",loc_a,"l3_414b");
  privateNode.param<std::string>("b",loc_b,"l3_414");

  Client client("/action_executor/execute_plan", true);
  client.waitForServer();
  bool fromAtoB = true;
  start = ros::Time::now().toSec();
  ros::Rate loop_rate(sample_frequency);
  
  //subscribe to diagnostics agg and parse for voltage name, use key as value
  ros::Subscriber voltage_sub = n.subscribe("/diagnostics_agg", 10, voltagecb);

  std::string path = ros::package::getPath("segbot_sensors");

  //movement based on 'back and forth' node bwi_task
  while(ros::ok() && !complete){
      std::string loc = (fromAtoB)? loc_b : loc_a;
      fromAtoB = !fromAtoB;
      //ROS_INFO_STREAM("going to " << loc);
      bwi_kr_execution::ExecutePlanGoal goal;

      bwi_kr_execution::AspRule rule;
      bwi_kr_execution::AspFluent fluent;
      fluent.name = "not at";
      
      fluent.variables.push_back(loc);
      rule.body.push_back(fluent);
      goal.aspGoal.push_back(rule);
      
      ROS_INFO("Sending goal");
      client.sendGoalAndWait(goal);

      if ( client.getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO("Goal aborted");
      }
      else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal succeeded.");
      
      ros::spinOnce();
      loop_rate.sleep();
  }
  file.close();
  return 0;
}
