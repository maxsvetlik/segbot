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

int sample_frequency = .1; //frequency in hz; assume at least 5 hours of data
double sum_v, sum_v2, sum_time, sum_vtime, a, b;
bool complete;
int n = 0;

//compute curve fitting sums while voltage exists and is above 10 
void voltagecb(){
    if(voltage == NULL){
        ROS_INFO("Error: curvefitting not computing. Voltage data non-existent or bad. Is it being published?");
    } else if(voltage > 10.0 && !complete){
        a = ((sum_v2*sum_time - sum_v*sum_vtime)/(n*sum_v2 - sum_v*sum_time));
        b = ((n*sum_vtime - sum_v*sum_time)/(n*sum_v2 - sum_v*sum_v));
        writeToFile(a, b);
        complete = true;
    } else if(!complete){
        n++;
        sum_v = sumv + voltage;
        sum_v2 = sum_v2 + voltage*voltage;
        sum_time = sum_time + time_since_last_update;
        sum_vtime = sum_vtime + voltage*time_since_last_update;
    }
}

void writeToFile(double a, double b){

}

int main(int argc, char **argv){
  ros::init(argc, argv, "battery_profiler");
  ros::NodeHandle n;
  //ros::Subscriber voltage_sub = n.subscribe("/Diagnostics/voltage", 10, voltagecb);
  ros::Rate loop_rate(sample_frequency);
  
  return 0;
}
