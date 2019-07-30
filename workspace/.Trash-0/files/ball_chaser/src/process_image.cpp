#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot

    ROS_INFO_STREAM("Request a service and pass velocities to drive_bot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x=lin_x;
    srv.request.angular_z=ang_z;

    // Call the service and pass to drive the bot
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool find_whiteball=false;
    int ball_position=0;
    int left_section_end=(int)img.width/3;  // left_seciton=266
    int right_section_start=(int)img.width*2/3;  // right_section =533;
    //ROS_INFO("img width: %i", img.width);   //img.width=800
    //ROS_INFO("img step: %i", img.step);     // img.step=2400;

    // 1/3~ 2/3 is forward section

    // 1. Loop through each pixel in the image and check if there's a bright white one

    for (int i = 0; i < img.height * img.step; i+=3)  // size: step * rows(height) // stp each cell [r,g,b]
    {  
        if (img.data[i] == white_pixel) {   // The first R value is equal to 255
	    if (img.data[i+1] == white_pixel && img.data[i+2] == white_pixel) // Check if G&B values are equal to 255
		{
          	  find_whiteball = true;
	   	  ball_position =i/3; 
	       	  ball_position=ball_position%img.width;
          	  break;
		}
        }
    }

    // 2. Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    if (find_whiteball==false)
    {
        drive_robot(0.0,0.0);   // stop as default
    } 
    else if (ball_position< left_section_end) 
    {
	ROS_INFO("Left: %i",ball_position);
 	drive_robot(1.0,1.5);   // ccw direction--> left
    }
    else if (ball_position> right_section_start)
    {
	ROS_INFO("Right : %i",ball_position);
 	drive_robot(1.0,-1.5);  // cw direction --> right 
    } 
    else                      // move forward
    {
	ROS_INFO("Forward");
	drive_robot(1.0,0.0);
    }


}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // !!! Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}


