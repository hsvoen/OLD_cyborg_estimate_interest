/**
	Node for estimating if a person is interested based on skeleton tracking from the kinect.
	


	Subscribes to:
		- /head/kinect2/bodyArray

	Publishes to:
		- person_status

	# TODO:
		- Tune Interest, approach and leave functions
		- Verify that they work
		- Add hesitant category to seperate between very interested and slightly interested people
		- seperating between people approaching the cyborg directly, or those moving towards and past it.
			* People passing it will walk quickly?
*/

#include "ros/ros.h"

#include <math.h>
#include <vector>



//Messages
#include "trollnode/PersonStatus.h"
#include "trollnode/PersonArray.h"
#include "k2_client/BodyArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"

//Class header
#include "trollnode/Person.h" 


//Topic names
std::string bodyTopicName = "/head/kinect2/bodyArray";
std::string people_status_topic_name = "person_status";


//Global variables
Person person_array [6]{Person(0), Person(1), Person(2), Person(3), Person(4), Person(5)}; //stores data about all people tracked




// Old code for detecting groups

//starting with detecting one group
//bool[6] group;




/* Arrays with integers of group numbers, check if standing close to any group members. Adding group number to Person?*/
/*
void find_groups(Person [] people_list)
{
	group_distance_treshold = 0.5;

	//Salient information order: Distance (to any member)

	for (int i = 0; i < 6; i++)
	{

		for (int j = 0; j < 6; j++)
		{
			if (people_list[i].is_tracked() && people_list[j].is_tracked)
			{

				//if close enough to be in a group, add member
				if( group_distance_treshold > people_list[i].get_distance_to_person(people_list[j]))
				{
					people_list[i].add_group_member(j);

				}
				// In group, but no longer close enough, remove member
				else if (people_list[i].is_in_group(j))
				{
					people_list[i].remove_group_member(j);
				}

			}
		}

	}

}
*/

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

			if(person_array[i].is_interested())
				ROS_INFO("Person is interested");
			else
				ROS_INFO("Person is not interested");

			if(person_array[i].is_slowing_down())
				ROS_INFO("Slowing down.");
			else if (person_array[i].is_moving_faster())
				ROS_INFO("speeding up");
			else
				ROS_INFO("Keeping speed");

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


/*
			if(person_array[i].is_approaching())
				ROS_INFO("Approaching the Cyborg");
			else if (person_array[i].is_leaving())
				ROS_INFO("Leaving the Cyborg");
			else
				ROS_INFO("Keeping distance");
*/

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

	ros::Rate loop_rate(0.5);

	while(ros::ok())
	{
		trollnode::PersonArray status_list_msg;


		
		for (int i = 0; i < 6; i++)
		{
			trollnode::PersonStatus person_msg;
			if (person_array[i].is_tracked() )
			{
				person_msg.tracked = true;
				person_msg.interested = person_array[i].is_interested();
				//person_msg.approaching = person_array[i].is_approaching();
				//person_msg.leaving = person_array[i].is_leaving();
				person_msg.stationary = person_array[i].is_stationary();
				//person_msg.keeping_distance = person_array[i].is_keeping_distance();
				person_msg.speed = person_array[i].get_speed();
				person_msg.distance = person_array[i].get_distance_to_cyborg();
				person_msg.position = person_array[i].get_position().head;

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

