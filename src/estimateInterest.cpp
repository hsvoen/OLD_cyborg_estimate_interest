#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "k2_client/BodyArray.h"

#include <math.h>
#include <vector>

#include "trollnode/Person.h"
#include "trollnode/Interested.h"

std::string bodyTopicName = "/head/kinect2/bodyArray";
std::string interest_topic_name = "person_interest";





/*================ INIT ============*/


//=====================================
//Person person_array [6]{Person(0), Person(1), Person(2), Person(3), Person(4), Person(5)};

Person person_array [6]{Person(0), Person(1), Person(2), Person(3), Person(4), Person(5)};


void bodyListener(const k2_client::BodyArray::ConstPtr& body_array)
{
	int people_tracked = 0;
	for (int i = 0; i < 6; i++)
	{


		if(body_array->bodies[i].isTracked)
		{
			person_array[i].tracked();
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

			if(person_array[i].is_stationary())
			{
				ROS_INFO("Person is stationary");
			}
			else
				ROS_INFO("Person is moving");

			if(person_array[i].is_approaching())
				ROS_INFO("Approaching the Cyborg");
			else if (person_array[i].is_leaving())
				ROS_INFO("Leaving the Cyborg");
			else
				ROS_INFO("Keeping distance");


			//print position
			person_array[i].get_position().print_position();
			ROS_INFO("Distance from Cyborg:[%.3f]", person_array[i].get_distance_to_cyborg() );

			//ROS_INFO("Estimates:");
			ROS_INFO("Estimated future pos, 1 sec:");
			person_array[i].estimate_future_position(1).print_position();
			ROS_INFO("Distance to cyborg: [%.3f]\n",person_array[i].estimate_future_position(1).get_distance_to_cyborg());

		}
		else
			person_array[i].not_tracked();

	}
	if(people_tracked > 0)
		ROS_INFO("People tracked: %d \n",people_tracked);

/*//Joint numbers and meaning

{
						case 0: fieldName = "SpineBase";break;
						case 1: fieldName = "SpineMid";break;
						case 2: fieldName = "Neck";break;
						case 3: fieldName = "Head";break;
						case 4: fieldName = "ShoulderLeft";break;
						case 5: fieldName = "ElbowLeft";break;
						case 6: fieldName = "WristLeft";break;
						case 7: fieldName = "HandLeft";break;
						case 8: fieldName = "ShoulderRight";break;
						case 9: fieldName = "ElbowRight";break;
						case 10: fieldName = "WristRight";break;
						case 11: fieldName = "HandRight";break;
						case 12: fieldName = "HipLeft";break;
						case 13: fieldName = "KneeLeft";break;
						case 14: fieldName = "AnkleLeft";break;
						case 15: fieldName = "FootLeft";break;
						case 16: fieldName = "HipRight";break;
						case 17: fieldName = "KneeRight";break;
						case 18: fieldName = "AnkleRight";break;
						case 19: fieldName = "FootRight";break;
						case 20: fieldName = "SpineShoulder";break;
						case 21: fieldName = "HandTipLeft";break;
						case 22: fieldName = "ThumbLeft";break;
						case 23: fieldName = "HandTipRight";break;
						case 24: fieldName = "ThumbRight";break;
						*/



}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "interest");

	ros::NodeHandle n;


	ros::Subscriber sub = n.subscribe(bodyTopicName, 10, bodyListener);

	ros::Publisher interest_publisher = n.advertise<trollnode::Interested>(interest_topic_name, 10); 
	ros::Rate loop_rate(0.5);

	while(ros::ok())
	{
		trollnode::Interested interested_msg;


		
		for (int i = 0; i < 6; i++)
		{
			if (person_array[i].is_tracked() && person_array[i].is_interested())
			{
				
				interested_msg.interested_person.push_back(true);
			}
			else
				interested_msg.interested_person.push_back(false);
			
		}
		
	


    interest_publisher.publish(interested_msg);

    ros::spinOnce();

    loop_rate.sleep();

	}
	


	return 0;
}

