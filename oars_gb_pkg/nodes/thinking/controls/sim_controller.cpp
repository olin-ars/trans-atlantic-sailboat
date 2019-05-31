/**
 * @brief Controller node for input to boat simulation
 * @file sim_controller.cpp
 * 
 * Provides basic controller output to move feedback values to the input values.
 * 
 * This file is part of the OARS project of Olin college.
 * 
 * @author Shashank Swaminathan <sh.swami235@gmail.com>
 */

#include "ros/ros.h"
#include "modelica_bridge/ModComm.h"
#include <sensor_msgs/Joy.h>

/** @def MAX_ARRAY
 *  @brief Defines the maximum size of internal data arrays
 */
#define MAX_ARRAY 100

ros::Publisher control_pub; ///< publisher for controller values

double feedback[MAX_ARRAY] = {0.0}; ///< array to store control values: 1 -> desired boat heading, 2:3 -> global wind vel

/**
 * Receives feedback from the model; calculates and publishes control values.
 * Reads from topic /model_values. Publishes to topic /control_values
 * Only reads MAX_ARRAY length of incoming data - rest is not stored.
 * @param [in] feedback_val ModComm message holding model feedback values
 * @see springSetPoints()
 * @see control_pub()
 * @see joyCallback()
 * @return none
 */
void controlCallback(const modelica_bridge::ModComm::ConstPtr& feedback_val) {

    modelica_bridge::ModComm control_val;

    for(int i = 0; i < feedback_val->size; i++) {
        control_val.data.push_back(180 * feedback[i]);
    }

    control_val.size = control_val.data.size();
    control_pub.publish(control_val);
}

/**
 * Scales joystick input
 * Reads from the /joy topic, of the joy_node
 * @param [in] joy Message for joystick input
 * @return none
 */
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  for(int i = 0; i < 8; i++) {
    feedback[i] = 1 + joy->axes[i];
  }
}

/**
 * Initializes node, pub, and subs. Spin to keep the node running
 * @see control_pub()
 * @see controlCallback()
 * @see joyCallback()
 * @return 0
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    
    control_pub = n.advertise<modelica_bridge::ModComm>("control_values", 1);
    ros::Subscriber feedback_sub = n.subscribe("model_values", 1, controlCallback);
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::spin();

    return 0;
}