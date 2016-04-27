#include "ros/ros.h"
#include "std_msgs/String.h"


#include "trollnode/Expression.h"

#include "trollnode/Interested.h"
#include "trollnode/PersonStatus.h"
#include "trollnode/PersonArray.h"


std::string interest_topic_name = "person_interest";
std::string expression_topic_name = "trollExpression";
std::string people_status_topic_name = "person_status";


bool greeted_person [6] {false, false, false, false ,false, false};

ros::Publisher greeting_publisher;

trollnode::Expression greeting_msg;
trollnode::Expression goodbye_msg;







void interest_listener(const trollnode::Interested::ConstPtr& interest_array)
{

	for (int i = 0; i < 6; i++)
	{
		//Interested person appears, send greeting
		if (interest_array->interested_person[i] && ! greeted_person[i])
		{
			ROS_INFO("person [%d] greeted", i);
			greeted_person[i] = true;
			greeting_publisher.publish(greeting_msg);

		}

		//greeting person stops being interested or leaves tracking. send goodbye message
		else if (greeted_person[i] && ! interest_array->interested_person[i])
		{

			ROS_INFO("person [%d] given goodbye", i);
			greeting_publisher.publish(goodbye_msg);
			greeted_person[i] = false;
		}

	}


}


void people_status_listener(const trollnode::PersonArray::ConstPtr& person_array)
{

	for (int i = 0; i < 6; i++)
	{

		//person is tracked at all		
		if(person_array->people[i].tracked)
		{


		}



	}


}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "publishExpression");

	ros::NodeHandle n;

	greeting_publisher = n.advertise<trollnode::Expression>(expression_topic_name, 10); 
	ros::Subscriber sub = n.subscribe(interest_topic_name, 10, interest_listener);

	goodbye_msg.speech = "See you later.";
	goodbye_msg.expression = "sad";
	greeting_msg.speech = "Hello, it is nice to meet you.";
	greeting_msg.expression = "smile";








	ros::spin();





	return 0;
}
