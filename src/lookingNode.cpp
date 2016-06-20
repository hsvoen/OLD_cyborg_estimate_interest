/**
	Node that chooses in which direction the troll should look.
	A direction is chosen from where there are people, going for interested people first, and tracked people last.
	If no people are tracked, it looks straight ahead.

	Subscribes to:
		- people_interest

	Publishes to:
		- trollExpression

	# TODO:
		- Subscribe to k2_client so it knows which direction the person is.			-
*/


#include "ros/ros.h"

//Ros message files
#include "std_msgs/String.h"
#include "trollnode/Expression.h"
#include "estimate_interest/PersonInterest.h"
#include "estimate_interest/PersonArray.h"
#include "geometry_msgs/Point.h"

//ROS topic names
std::string people_interest_topic_name = "people_interest";

std::string expression_topic_name = "trollExpression";


enum Status {none = -1, tracked = 0, indecisive = 1, hesitating = 2, interested = 3};
enum Direction {up, down, left, right, neutral};

//Global variables

Direction 		prev_direction = neutral;
Direction 		new_direction = neutral;
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
	estimate_interest::Expression dir_msg;

	if (position.x >= left_treshold)
	{
		dir_msg.look = "left";
		new_direction = left;
	}
	else if (position.x <= right_treshold)
	{
		dir_msg.look = "right";
		new_direction = right;
	}
	else if (position.y >= up_treshold)
	{
		dir_msg.look = "up";
		new_direction = up;
	}
	else if (position.y <=  down_treshold)
	{
		dir_msg.look = "down";
		new_direction = down;
	}
	else
	{
		dir_msg.look = "neutral";
		new_direction = neutral;
	}

	if(new_direction != prev_direction)
	{
		ROS_INFO("Looking in [%s] direction", dir_msg.look.c_str());
		prev_direction = new_direction;
		looking_publisher.publish(dir_msg);
	}
	else
		ROS_INFO("Same looking direction as before");
}




void people_interest_listener(const estimate_interest::PersonArray::ConstPtr& person_array)
{

	//look at interested person first, then hesitating, then indecisive then tracked
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
			else if(person_array->people[i].hesitating && person_status <= hesitating)
			{
				person_to_look_at = i;
				person_status = hesitating;
			}
			else if(person_array->people[i].indecisive && person_status <= indecisive)
			{
				person_to_look_at = i;
				person_status = indecisive;
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
		//look_at_person(person_array->people[person_to_look_at].position);
	}

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "lookingNode");
	ros::NodeHandle n;

	//Publisher
	looking_publisher = n.advertise<trollnode::Expression>(expression_topic_name, 10); 

	//Listener
	ros::Subscriber people_interest_sub = n.subscribe(people_interest_topic_name, 10, people_interest_listener);


	ros::spin();

	return 0;
}
