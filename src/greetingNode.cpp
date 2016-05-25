/**
	Node that sends greetings to people when they show show interest in the robot and are approaching,
	and goodbyes when greeted people are leaving the Cyborg. A person is only greeted and goodbyed once
	as long as tracking persists.

	Subscribes to:
		- people_interest

	Publishes to:
		- trollExpression

	#TODO: 
		- Publish an interaction topic saying what interactions have taken place with people.

*/


#include "ros/ros.h"

//Message files
#include "std_msgs/String.h"
#include "trollnode/Expression.h"
#include "trollnode/PersonInterest.h"
#include "trollnode/PersonArray.h"

//Topic names
std::string expression_topic_name = "trollExpression";
std::string people_status_topic_name = "people_interest";


struct interaction
{
	bool tracked;
	bool greeted; //when interested
	bool encouraged; //when indecisive
	bool goodbye; //when leaving
};

interaction people [6] = {};

//global variables
ros::Publisher greeting_publisher;
trollnode::Expression greeting_msg;
trollnode::Expression goodbye_msg;
trollnode::Expression indecisive_msg;





void people_status_listener(const trollnode::PersonArray::ConstPtr& person_array)
{

	for (int i = 0; i < 6; i++)
	{

		//person is tracked at all		
		if(person_array->people[i].tracked)
		{
			people[i].tracked = true;

			//greeting conditions
			if( !(people[i].greeted) && person_array->people[i].interested )
			{
				ROS_INFO("person [%d] greeted", i);

				people[i].greeted = true;
				greeting_publisher.publish(greeting_msg);

			}
			else if( !(people[i].greeted) && !(people[i].encouraged) && !(people[i].goodbye) && person_array->people[i].indecisive )
			{	
				ROS_INFO("person [%d] encouraged", i);

				people[i].encouraged = true;
				greeting_publisher.publish(indecisive_msg);

			}
			//goodbye conditions: Greeted, not goodbye, not interested
			else if( people[i].greeted && !(people[i].goodbye) && person_array->people[i].not_interested )
			{
				ROS_INFO("person [%d] given goodbye", i);
				people[i].goodbye = true;
				greeting_publisher.publish(goodbye_msg);
			}


		}
		//Lost tracking of a tracked person
		else if(people[i].tracked == true)
		{
			people[i].tracked = false;
			people[i].greeted = false;
			people[i].goodbye = false;
			people[i].encouraged = false;
		}
	}



	


}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "greetingNode");
	ros::NodeHandle n;

	//Publisher
	greeting_publisher = n.advertise<trollnode::Expression>(expression_topic_name, 10); 

	//Subsciber
	ros::Subscriber people_status_sub = n.subscribe(people_status_topic_name, 10, people_status_listener);



	goodbye_msg.speech = "See you later.";
	goodbye_msg.expression = "sad";

	indecisive_msg.speech = "Don't be afraid.";
	indecisive_msg.expression = "smile";

	greeting_msg.speech = "Hello, it is nice to meet you.";
	greeting_msg.expression = "happy";

	ros::spin();

	return 0;
}
