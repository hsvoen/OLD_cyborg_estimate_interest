#include "ros/ros.h"
#include "std_msgs/String.h"


#include "trollnode/Expression.h"

#include "trollnode/Interested.h"
#include "trollnode/PersonStatus.h"
#include "trollnode/PersonArray.h"
#include "geometry_msgs/Point.h"


std::string interest_topic_name = "person_interest";
std::string expression_topic_name = "trollExpression";
std::string people_status_topic_name = "person_status";


enum Status {none = -1, tracked = 0, stationary = 1, approaching = 2, interested = 3};

Status person_status = none;
int person_to_look_at = -1;





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
		look_at_person(person_array->people[i].position)
	}

}

void look_at_person(geometry_msgs::Point position)
{
	//look left, look right, look up
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "publishExpression");

	ros::NodeHandle n;

	greeting_publisher = n.advertise<trollnode::Expression>(expression_topic_name, 10); 
	ros::Subscriber sub = n.subscribe(interest_topic_name, 10, interest_listener);










	ros::spin();





	return 0;
}
