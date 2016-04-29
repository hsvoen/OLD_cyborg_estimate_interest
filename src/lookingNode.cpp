/**
	Node that chooses in which direction the troll should look.
	A direction is chosen from where there are people, going for interested people first, and tracked people last.
	If no people are tracked, it looks straight ahead.

	Subscribes to:
		- person_status

	Publishes to:
		- trollExpression

	# TODO:
		- Adding granaluarity to which direction (looking a little to the right or a lot to the right)
		- Combining directions (up AND right)
*/


#include "ros/ros.h"

//Ros message files
#include "std_msgs/String.h"
#include "trollnode/Expression.h"
#include "trollnode/PersonStatus.h"
#include "trollnode/PersonArray.h"
#include "geometry_msgs/Point.h"

//ROS topic names
std::string people_status_topic_name = "person_status";

std::string expression_topic_name = "trollExpression";

enum Status {none = -1, tracked = 0, stationary = 1, approaching = 2, interested = 3};

//Global variables

Status 			person_status = none;
int 			person_to_look_at = -1;

ros::Publisher 	looking_publisher;


/*==== INIT=====*/
float left_treshold 	= 0.25;
float right_treshold 	= -0.25;
float up_treshold 		= 0.5;
float down_treshold 	= 0;


//Sends a ROS message to TrollExpression topic saying in which direction the troll should look 
// if the position exceeds a treshold value in left, right, up, or down order. If not, a neutral message is sent
void look_at_person(geometry_msgs::Point position)
{
	trollnode::Expression dir_msg;

	if (position.x >= left_treshold)
	{
		dir_msg.look = "left";
	}
	else if (position.x <= right_treshold)
	{
		dir_msg.look = "right";
	}
	else if (position.y >= up_treshold)
	{
		dir_msg.look = "up";
	}
	else if (position.y <=  down_treshold)
	{
		dir_msg.look = "down";
	}
	else
		dir_msg.look = "neutral";

	ROS_INFO("Looking in [%s] direction", dir_msg.look.c_str());
	looking_publisher.publish(dir_msg);
}




void people_status_listener(const trollnode::PersonArray::ConstPtr& person_array)
{

	//look at interested person first, then approaching, stationary then tracked
	person_to_look_at = -1;
	person_status = none;
	for (int i = 0; i < 6; i++)
	{

		//person is tracked at all		
		if(person_array->people[i].tracked)
		{

			if(person_array->people[i].interested)
			{
				person_to_look_at = i;
				person_status = interested;
			}
			else if(person_array->people[i].approaching && person_status <= approaching)
			{
				person_to_look_at = i;
				person_status = approaching;
			}
			else if(person_array->people[i].stationary && person_status <= stationary)
			{
				person_to_look_at = i;
				person_status = stationary;
			}
			else
			{
				person_to_look_at = i;
				person_status = tracked;
			}

		}
	}

	if(person_status > none)
	{
		look_at_person(person_array->people[person_to_look_at].position);
	}

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "lookingNode");
	ros::NodeHandle n;

	//Publisher
	looking_publisher = n.advertise<trollnode::Expression>(expression_topic_name, 10); 

	//Listener
	ros::Subscriber people_status_sub = n.subscribe(people_status_topic_name, 10, people_status_listener);


	ros::spin();

	return 0;
}
