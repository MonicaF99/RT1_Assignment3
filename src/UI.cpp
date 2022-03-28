/**
* \file UI.cpp
* \brief Final assignment: controller for a mobile robot using RViz and Gazebo
* \author Monica Fossati
* \version 1.0
* \date 18/03/2022

* \param [in] GOAL_TH Define the threshold distance between the robot and the goal to consider the goal reached
* \param [in] D_TH Define the threshold distance between the robot and a wall to consider too close and so dangerous
* \param [in] TIMEOUT Define the timeout for reaching a goal. After this time the goal is considered not reachable
*
* \details
*
* Subscribes to: <BR>
* ° /move_base/feedback It contains the current status
* ° /move_base/goal It contains the actual goal
* ° /prov_cmd_vel It contains the velocity command to be check if the robot is in driving assistence mode
* ° /scan It contains the distances measured by the laser scanner sensors
*
* Publishes to: <BR>
* ° /move_base/goal It contains the actual goal
* ° /move_base/cancel It cancel the goal specified by an ID
* ° /cmd_vel It contains the confirmed velocity command
*
* Description :
*
* This node allows the user to control a mobile robot in 3 different modes:
* ° setting the coordinates of a goal to be achieved and the robot would have to dynamically calculate a path to reach it
* ° controlling it via keyboard
* ° controlling it via keyboard with assistance to avoid obstacles
*
* The robot is moving in an unknown environment; but it can construct a map and detect walls thanks to laser scanner sensors.
*
*/

**/

// Headers
#include <iostream>
#include <string>
#include <chrono>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

#define GOAL_TH 0.5 //The goal is considered to be reached if the distance between it and the robot is < 0.5
#define D_TH 0.5 //A wall is considered to be too close if the distance between it and the robot is < 0.5
#define TIMEOUT 180000000 // A goal is considered not reachable if the robot doesn't reach it within 3 minutes

actionlib_msgs::GoalID canc_goal; ///< Goal to cancel message

// Declare the publishers
ros::Publisher pub_vel; ///< Define publisher on topic /cmd_vel
ros::Publisher pub_goal; ///< Define publisher on topic /move_base/goal
ros::Publisher pub_cancel; ///< Define publisher on topic /move_base/cancel

float x_goal; ///< Goal coordinate x
float y_goal; ///< Goal coordinate y


bool goal = false; ///< Boolean variable for indicating if there is a goal
bool manDrive = false; ///< Boolean variable for indicating if manual driving is enabled
bool driveAssistence = false; ///< Boolean variable for indicating if driving assistence is enabled

geometry_msgs::Twist stop; ///< Null velocity message to stop the robot
geometry_msgs::Twist new_vel; ///< Message with velocity to be checked by the driving assistence before being published

char k = 'a'; ///< Variable used to store the user commands
std::string id = ""; ///< Goal ID
// Time variable to measure the time spent reaching the goal
std::chrono::high_resolution_clock::time_point time_start;  ///< Start time for reaching a goal
std::chrono::high_resolution_clock::time_point time_end; ///< Actual time to be compared with the start time and compute the time spent in reaching the goal

const char * menu = R"(
   MENU
   --------------------------------------------
   0 : Quit
   1 : Insert new goal's coordinates
   2 : Cancel current goal
   3 : Manual driving
)"; ///< Initialize a raw string literal to print the menu

/**
* \brief This function print on the screen the actual goal if a goal has been set
* \return void as this method cannot fail.
*
* This function print on the screen the actual goal if a goal has been set, otherwise it displays NaN as goal's coordinates
*/
void printGoal()
{
	if(goal)
	{
		printf("----------------------------\n");
		printf("Actual goal\n");
	    	printf("x = %f\n", x_goal);
	    	printf("y = %f\n", y_goal);
	    	printf("----------------------------\n");
    	}
    	else
    	{
    		printf("----------------------------\n");
    		printf("Actual goal\n");
	    	printf("x = NaN\n");
	    	printf("y = NaN\n");
	    	printf("----------------------------\n");
    	}
}

/**
* \brief This function set the new goal from the user
* \return void as this method cannot fail.
*
* This function ask the user to enter the goal's coordinates
* Then it sends a message to the goal topic and takes the starting time
*/
void setGoal()
{
	move_base_msgs::MoveBaseActionGoal goal_msg;
	std::string s;
	float x,y;
	// Take new goal from the user
        printf("Enter goal's coordinates\n");
	printf("x: ");
	std::cin >> s;
	x = atof(s.c_str());
	printf("y: ");
	std::cin >> s;
	y = atof(s.c_str());
	
	// Set new coordinates to reach
	goal_msg.goal.target_pose.header.frame_id = "map";
	goal_msg.goal.target_pose.pose.orientation.w = 1;
		    
	goal_msg.goal.target_pose.pose.position.x = x;
	goal_msg.goal.target_pose.pose.position.y = y;

	// Publish new goal 
	pub_goal.publish(goal_msg);
	// Set the starting time
	time_start = std::chrono::high_resolution_clock::now();
	// There is a goal
	goal = true;
}


/**
* \brief This function cancels the actual goal
* \return True if the goal has been cancelled correctly, False if there is no goal to cancel
*
* This function return false if there isn't a goal to cancel, or cancel the goal by its ID publishing a message on /move_base/cancel topic and return true
*/
bool cancelGoal()
{
	if(!goal)
		return false;
	canc_goal.id = id;
	pub_cancel.publish(canc_goal);
	goal = false;
	return true;
	
}

/**
* \brief When a new current status is available, this function checks if the goal is been reached, updates the actual goal id and cancel the actual goal if the timeout is reached
* \param msg It's a pointer to the message published on move_base/feedback topic.
* \return void as this method cannot fail.
*
* This function updates the goal ID if there is a new goal
* If there is a goal checks if the robot has reached the goal, otherwise it computes for how much time the robot was trying to reach the goal
* If this time is greater than the timeout, the goal is cancelled
*/
void currentStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) {

    // Update the goal ID if there is a new goal
    if (id != msg->status.goal_id.id) {
        id = msg->status.goal_id.id;
    }
    
    // If there is a goal check if the robot has reached the goal or is trying to reach the goal for a too long time
    if(goal)
    {
    	// Take the current robot position
        float dist_x;
	float dist_y;
	float robot_x = msg->feedback.base_position.pose.position.x;
	float robot_y = msg->feedback.base_position.pose.position.y;
    
	// Compute the error from the actual position and the goal position
	dist_x = robot_x - x_goal;
	dist_y = robot_y - y_goal;

	// The robot is on the goal position
	if (abs(dist_x) <= GOAL_TH && abs(dist_y) <= GOAL_TH)
	{
	     printf("Goal reached\n");
	     cancelGoal();
	}

    	time_end = std::chrono::high_resolution_clock::now();
        auto time = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
        if(time > TIMEOUT)
        {
        	printf("The actual goal can't be reached!\n");
        	cancelGoal(); // Cancel the goal if it can't be reached
        }
    }
}

/**
* \brief This function saves in global variables the actual goal
* \return void as this method cannot fail.
*/
void actualGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    x_goal = msg->goal.target_pose.pose.position.x;
    y_goal = msg->goal.target_pose.pose.position.y;
}

/**
* \brief This function decides if a provvisory velocity has to be published or not.
* \param msg It's a pointer to the message published on /prov_cmd_vel topic.
* \return void as this method cannot fail.
*
* This function is called whenever the teleop twist keyboard want to change the robot velocity
* If the robot isn't in manual driving, the message is ignored.
* If the robot is in manual driving, but driving assistence is disabled, the message is sent to the /cmd_vel topic
* If the robot is in manual driving and driving assistence is enabled, the message is saved in a global variable and it will be corrected by the driving assistence before being published
*/
void getVel(const geometry_msgs::Twist::ConstPtr& msg) {
    if(!manDrive)
    	return;
    if(!driveAssistence)
    {
    	pub_vel.publish(msg);
    	return;
    }
    new_vel.linear.x = msg->linear.x;
    new_vel.angular.z = msg->angular.z;
}

/**
* \brief This function computes the minimum distance detected by the sensors
* \param scan[] It's the array containing the distances measured by the laser scanner
* \param size It's the scan[] length
* \return min_dist The minumum distance in the array provided
*/
double check_dist(double scan[], int size)
{
	double min_dist = 100;
	for(int i=0; i < size; i++)
	{
		if(scan[i] < min_dist)
		{
			min_dist = scan[i];
		}
	}
	return min_dist;
}

/**
* \brief This function decides if the robot can execute the user command without hitting a wall.
* \param msg It's a pointer to the message published on /scan topic.
* \return void as this method cannot fail.
*
* This function is called whenever a message from LaserScan topic arrives.
* If driving assistence is disabled the function does nothing.
* If the driving assistence is enabled:
* 	- divides the distances in input in 5 sectors
* 	- if there is a close wall in the direction where the user wants the robot to go, 
* 	  it prints a warning on the console and stops the robot
*/

void drivingAssistence(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	if(!driveAssistence) // If driving assistence is disabled, do nothing
		return;
	// get array by laserscan
	float scan[msg->ranges.size()];
	
	for(int k = 0; k < msg->ranges.size(); k++)
	{
		scan[k] = msg->ranges[k];
	}
	
	//divide the laserscan values in 5 sectors
	int sector_size = (msg->ranges.size())/5;
	double left[sector_size];
	double front_left[sector_size];
	double front[sector_size];
	double front_right[sector_size];
	double right[sector_size];
	
	//fill sector arrays
	for(int i = 0; i < sector_size; i++)
	{
		left[i] = scan[i + 4*sector_size];
	}

	for(int i = 0; i < sector_size; i++)
	{
		front_left[i] = scan[i + 3*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front[i] = scan[i + 2*sector_size];
	}
	
	for(int i = 0; i < sector_size; i++)
	{
		front_right[i] = scan[i + sector_size];
	}

	for(int i = 0; i < sector_size; i++)
	{
		right[i] = scan[i];
	}
	// check if there is a too close wall in the direction of the robot
	if(check_dist(front, sector_size) < D_TH) // Check if there is a wall in front of the robot
	{
		if(new_vel.linear.x > 0 && new_vel.angular.z == 0) // Check if the robot has to go straight on
		{
			new_vel.linear.x = 0; // Stop the robot
			printf("Attenction: wall!!\n"); // Print an error
		}
	}
		
	if(check_dist(front_left, sector_size) < D_TH)
	{
		if(new_vel.linear.x > 0 && new_vel.angular.z > 0)
		{
			new_vel.linear.x = 0;
			new_vel.angular.z = 0;
			printf("Attenction: wall!!\n");
		}
	}
	
	if(check_dist(front_right, sector_size) < D_TH)
	{
		if(new_vel.linear.x > 0 && new_vel.angular.z < 0)
		{
			new_vel.linear.x = 0;
			new_vel.angular.z = 0;
			printf("Attenction: wall!!\n");
		}
	}
		
	if(check_dist(left, sector_size) < D_TH)
	{
		if(new_vel.linear.x == 0 && new_vel.angular.z > 0)
		{
			new_vel.angular.z = 0;
			printf("Attenction: wall!!\n");
		}
	}
		
	if(check_dist(right, sector_size) < D_TH)
	{
		if(new_vel.linear.x == 0 && new_vel.angular.z < 0)
		{
			new_vel.angular.z = 0;
			printf("Attenction: wall!!\n");
		}
	}

    	pub_vel.publish(new_vel); // Publish the correct velocity 
		
}

/**
* \brief This function manages the manual driving mode
* \return void as this method cannot fail.
*
* This function is called when the user choose the manual driving.
* It changes the driving mode and enable/disable driving assistence if the user press 'a'.
* It exits from manual driving if the user press 'x'
*/
void manualDrive()
{
	manDrive = true; // Change driving mode
	while(k != 'x')
	{
		// Print manual driving menu
		printf("----------------------------\n");
		printf("MANUAL DRIVING\n");
		if(driveAssistence)
			printf("Driving assistence enabled\n");
		else
			printf("Driving assistence disabled\n");
		printf("----------------------------\n");
		printf("Press a for enable/disable driving assistence\n");
		printf("Press x for exiting\n");
		scanf(" %c", &k); // Read user command
		system("clear");
		switch(k)
		{
			case 'a':
				if(!driveAssistence) // Change driving assistence's state
					driveAssistence = true;
				else
					driveAssistence = false;
				break;
			case 'x':
				manDrive = false; // Change driving mode
				driveAssistence = false; // Disable driving assistence
				pub_vel.publish(stop); // Set velocity to zero before exiting
				break;
			default:
				printf("Invalid command!!\n");
		}
	}
}

/**
* \brief This function manages the main menu
* \return void as this method cannot fail.
*
*This function print the main menu on the screen and calls the functions needed for the selected operation
*/

void choice()
{
	while(k != '0')
	{
		printGoal(); // Print the actual goal on the console
	    	printf("%s\n", menu); // Print menu
	    	scanf(" %c", &k); // Read the input from the user
	    	system("clear"); // Clear the terminal for order
	    	switch(k) // Select the right action
	    	{
	    		case '0':
	    			// Quit
	    			break;
	    		case '1':
	    			setGoal(); 
	    			system("clear"); // Clear the terminal to rewrite the menu
	    			break;
	    		case '2':
	    			if(cancelGoal()) // If the goal cancel succeded
	    				printf("Previous goal cancelled\n");
	    			else
	    				printf("No goal to cancel\n");
	    			break;
	    		case '3':
	    			manualDrive(); // Enter in manual driving mode
	    			break;
	    		default:
	    			printf("***INVALID COMMAND***");
	    			break;
	    	
	    	}
    	}
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "final_UI");
    ros::NodeHandle nh;
    
    // Initialize the publishers
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000); // Update goal
    pub_cancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000); // Cancel actual goal
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // Update robot velocity
    
    // Define the subscribers
    ros::Subscriber sub_pos = nh.subscribe("/move_base/feedback", 1000, currentStatus); // Current  Status
    ros::Subscriber sub_goal = nh.subscribe("/move_base/goal", 1000, actualGoal); // Actual Goal feedback
    ros::Subscriber sub_key = nh.subscribe("/prov_cmd_vel", 1000, getVel); // Get velocity from teleop twist keyboard
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistence); // Laser scanner
    
    
    // Multi-threading
    ros::AsyncSpinner spinner(4);
    spinner.start();
    choice(); // Enter in the main menu
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}
