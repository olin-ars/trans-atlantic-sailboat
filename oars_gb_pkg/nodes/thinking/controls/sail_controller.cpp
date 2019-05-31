/**
 * @brief SailControl class and node
 * @file sail_controller.cpp
 * 
 * Node for controlling either the physical or simulated sail on the boat.
 * 
 * This file is part of the Olin College of Engineering's OARS repository 
 * and work towards controlling and automating a sailboat.
 * 
 * Copyright 2019
 * @author Shashank Swaminathan <sh.swami235@gmail.com>
 */

#include <ros/ros.h>
#include <modelica_bridge/ModComm.h>
#include <sensor_msgs/Joy.h>

#include <stdlib.h>
#include <string.h>

/** @def MAX_ARRAY
    @brief Defines the maximum size of internal buffer arrays
*/
#define MAX_ARRAY 256 // define maximum size of internal buffer arrays

/**
 * Class to act as controller for the boat's sail.
 * It has two main modes: Simulation and physical boat control.
* It has two sub-modes, depending on the major mode: 
    * Major mode: Simulation | Fixed boat and free boat.
    * Major mode: Physical boat | Tele-op control and sensor-feedback-based control.
 */
class SailControl
{
    public:
        /**
         * Defult constructor initializes the controller publisher, the parameter node, and the appropriate subscribers.
         * No inputs.
         * @return none
         */
        SailControl();
        /**
         * Keep program from closing.
         * Run callbacks for ROS nodes, and publishes controller values.
         * No inputs.
         * @return none
         */
        void spin();

    private:
        /**
         * Callback for receiving feedback values from simulation.
         * Reads from topic /model_values.
         * Stores data into array of size MAX_ARRAY - only reads MAX_ARRAY amount of array slots.
         * @param[in] inVal ModComm message holding controller values.
         * @return none.
         */
        void simFeedback(const modelica_bridge::ModComm::ConstPtr& inVal);
        /**
         * Callback for receiving input values from tele-op for the fixed boat model.
         * Reads from topic /joy of the joy node.
         * Stores data into array of size MAX_ARRAY - only reads MAX_ARRAY amount of array slots.
         * @param[in] joyVal joy message holding controller values.
         * @return none.
         */
        void joyFixedBoat(const sensor_msgs::Joy::ConstPtr& joy);
        /**
         * Callback for receiving input values from tele-op for the free boat model.
         * Reads from topic /joy of the joy node.
         * Stores data into array of size MAX_ARRAY - only reads MAX_ARRAY amount of array slots.
         * @param[in] joyVal joy message holding controller values.
         * @return none.
         */
        void joyFreeBoat(const sensor_msgs::Joy::ConstPtr& joy);
        /**
         * Callback for receiving sensor and input values from the boat.
        * Reads from topic /joy of the joy node.
         * Stores data into array of size MAX_ARRAY - only reads MAX_ARRAY amount of array slots.
         * @param[in] joyVal joy message holding controller values.
         * TO BE IMPLEMENTED.
         */
        void joyPhysicalBoat(const sensor_msgs::Joy::ConstPtr& joy);
        

        ros::NodeHandle nh_; ///< ROS node handle
        ros::Subscriber feedback_sub_; ///< subscriber to feedback on boat position and velocity heading
        ros::Subscriber joy_sub_; ///< subscriber to joystick input
        ros::Subscriber sensor_sub_; ///< subscriber to boat sensors
        ros::Publisher pub_; ///< publisher for model feedback values

        int update_rate_; ///< ROS node maximum loop rate.
        int major_mode_; ///< Specifies major mode. 1 -> Simulation | 2 -> Physical boat
        int minor_mode_; ///< Specifies minor mode. If major_mode_ == 1, 1 ->  Fixed boat | 2 -> Free boat || if major_mode_ == 2, 1 -> Tele-op | 2 -> Sensors

        double controller_buffer_[MAX_ARRAY]; ///< Buffer holding finalized controller values to be published
        double feedback_buffer_[MAX_ARRAY]; ///< Buffer holding values received from feedback subscriber
        double input_buffer_[MAX_ARRAY]; ///< Buffer holding valeus received from either tele-op or sensor input

        modelica_bridge::ModComm outVal; ///< Output message of the controller
};

SailControl::SailControl()
{
    pub_ = nh_.advertise<modelica_bridge::ModComm>("control_values", 1);

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("update_rate", update_rate_, 20);
    nh_param.param<int>("major_mode", major_mode_, 1);
    nh_param.param<int>("minor_mode", minor_mode_, 1);
    
    // Setup subscribers according to the specified major modes.
    if (major_mode_ == 1)
        feedback_sub_ = nh_.subscribe("model_values", 1, &SailControl::simFeedback, this); // Subscribe to the modelica_bridge node to connect to Modelica.
    else if (major_mode_ == 2) {
        /*
         * Implement the connection to the physical boat here.
         * Ideally, the controller will read values of the boat's position and velocity heading at this point.
         */
    }
    else {
        perror("Not a valid input for the major mode.");
        exit(1);
    }

    // Setup subscribers according to the specified major modes.
    if (major_mode_ == 1 && minor_mode_ == 1)
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SailControl::joyFixedBoat, this);
    else if (major_mode_ == 1 && minor_mode_ == 2)
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SailControl::joyFreeBoat, this);
    else if (major_mode_ == 2 && minor_mode_ == 1)
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SailControl::joyPhysicalBoat, this);
    else if (major_mode_ == 2 && minor_mode_ == 2) {
        /*
         * Implement the connection to the physical boat's sensors here.
         */
    }
    else {
        perror("Not a valid input for the minor mode.");
        exit(1);
    }
}

void SailControl::spin() 
{
    ros::Rate loop(update_rate_);

    while(ros::ok())
    {
        outVal.data.clear();
        outVal.size = controller_buffer_[0];
        for (int i = 1; i <= outVal.size; i++) {
            outVal.data.push_back(controller_buffer_[i]);
        }
        pub_.publish(outVal);

        ros::spinOnce();
        loop.sleep();
    }
}

void SailControl::simFeedback(const modelica_bridge::ModComm::ConstPtr& inVal)
{
    feedback_buffer_[0] = inVal->size;
    int limit = (MAX_ARRAY > inVal->size) ? inVal->size : MAX_ARRAY-1;
    for(int i = 0; i < limit; i++) {
        feedback_buffer_[i+1] = inVal->data[i];
    }
}

void SailControl::joyFixedBoat(const sensor_msgs::Joy::ConstPtr& joyVal) 
{
    controller_buffer_[0] = 8;

    for(int i = 0; i < 8; i++) {
        controller_buffer_[i+1] = 180 * joyVal->axes[i];
    }
}

void SailControl::joyFreeBoat(const sensor_msgs::Joy::ConstPtr& joyVal) 
{
    for(int i = 0; i < 8; i++) {
        input_buffer_[i] = joyVal->axes[i];
    }
}

void SailControl::joyPhysicalBoat(const sensor_msgs::Joy::ConstPtr& joyVal) 
{
    for(int i = 0; i < 8; i++) {
        input_buffer_[i] = joyVal->axes[i];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sail_controller");

    SailControl sail_controller;
    
    sail_controller.spin();
    
    return 0;
}
