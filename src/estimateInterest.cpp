/**
	Node for estimating if a person is interested based on skeleton tracking from the kinect.
	


	Subscribes to:
		- /head/kinect2/bodyArray

	Publishes to:
		- people_interest

*/

#include "ros/ros.h"

#include <math.h>
#include <vector>



//Messages
#include "trollnode/PersonInterest.h"
#include "trollnode/PersonArray.h"
#include "k2_client/BodyArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

//Class header
#include "trollnode/Person.h" 


//Topic names
std::string bodyTopicName = "/head/kinect2/bodyArray";
std::string people_status_topic_name = "people_interest";


//Global variables
Person person_array [6]{Person(0), Person(1), Person(2), Person(3), Person(4), Person(5)}; //stores data about all people tracked

float speedlog = 0;



//callback function for body tracking topic. stores positions and prints info
void bodyListener(const k2_client::BodyArray::ConstPtr& body_array)
{
	int people_tracked = 0;
	for (int i = 0; i < 6; i++)
	{


		if(body_array->bodies[i].isTracked)
		{
			person_array[i].tracked();

			//Joint position 0 = SpineBase, joint position 3 = head.
			person_array[i].add_position(Position(body_array->bodies[i].jointPositions[3].position,body_array->bodies[i].jointPositions[0].position, body_array->bodies[i].header.stamp.toSec()));


			people_tracked++;


			ROS_INFO("Person: %d, time: [%f]",i, body_array->bodies[i].header.stamp.toSec());
			ROS_INFO("Speed: [%f], distance [%f], ", person_array[i].get_speed(), person_array[i].get_distance_to_cyborg());

			if(person_array[i].get_interested())
				ROS_INFO("Person is interested");

			if(person_array[i].get_indecisive())
				ROS_INFO("Person is indecisive");

			if(person_array[i].get_hesitating())
				ROS_INFO("Person is hesitating");

			if(person_array[i].get_not_interested())
				ROS_INFO("Person is not interested");


			
			if(person_array[i].is_moving_closer())
				ROS_INFO("Moving closer");
			else if(person_array[i].is_moving_away())
				ROS_INFO("Moving away");
			else
				ROS_INFO("Keeping distance");

			if(person_array[i].is_stationary())
				ROS_INFO("Person is stationary");
			else
				ROS_INFO("Person is moving");


			person_array[i].get_position().print_position();

		}
		else
			person_array[i].not_tracked();


	}
	if(people_tracked > 0)
		ROS_INFO("People tracked: %d \n",people_tracked);




}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Interestnode");
	ros::NodeHandle n;

	//subscribers
	ros::Subscriber sub = n.subscribe(bodyTopicName, 10, bodyListener);

	//Publishers
	ros::Publisher people_status_publisher = n.advertise<trollnode::PersonArray>(people_status_topic_name, 10); 

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		trollnode::PersonArray status_list_msg;


		
		for (int i = 0; i < 6; i++)
		{
			trollnode::PersonInterest person_msg;
			if (person_array[i].get_tracked() )
			{
				person_msg.tracked = true;

				//Classify interest
				person_array[i].estimate_interest();

				person_msg.interested = person_array[i].get_interested();
				person_msg.indecisive = person_array[i].get_indecisive();
				person_msg.hesitating = person_array[i].get_hesitating();
				person_msg.not_interested = person_array[i].get_not_interested();

				status_list_msg.people.push_back(person_msg);
			}
			else
			{
				person_msg.tracked = false;

				status_list_msg.people.push_back(person_msg);
			}
		}
		
	//Publish 
    people_status_publisher.publish(status_list_msg);

    ros::spinOnce();
    loop_rate.sleep();

	}
	


	return 0;
}

