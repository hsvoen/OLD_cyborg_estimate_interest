#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "k2_client/BodyArray.h"

#include <math.h>
#include <vector>

#include "trollnode/Person.h"

std::string bodyTopicName = "/head/kinect2/bodyArray";

//Forward declaration
//class Position;
//class Person;




/*================ INIT ============*/
int kinect_height = 0.5; //height of the kinect in m


//=====================================
Person person_array [6]{Person(), Person(), Person(), Person(), Person(), Person()};


/*
class Position{
	geometry_msgs::Point point;
	std_msgs::Header header;

public:
	Position(geometry_msgs::Point pos, std_msgs::Header stamp)
	{
		point = pos;
		header = stamp;
	}
};



class Person 
{
	int number;
	bool isTracked;
	float speed;
	std::vector<Position> positions;

public:
	Person() {isTracked = false; speed = 0;}
	Person(bool tracked) {isTracked = tracked;}

	int add_position(Position pos){ positions.push_back(pos);}
	void is_tracked(){isTracked = true;}
};

*/




float distance_from_cyborg_calc(geometry_msgs::Point position)
{
	return sqrt( pow(position.x,2) + pow(position.y,2) + pow( abs(position.z - kinect_height ),2));
}

geometry_msgs::Point vector_between_points(geometry_msgs::Point pos1, geometry_msgs::Point pos2)
{
	geometry_msgs::Point vector;

	vector.x = pos2.x - pos1.x;
	vector.y = pos2.y - pos1.y;
	vector.z = pos2.z - pos1.z;

	return vector;

}



void bodyListener(const k2_client::BodyArray::ConstPtr& body_array)
{
	int people_tracked = 0;
	for (int i = 0; i < 6; i++)
	{


		if(body_array->bodies[i].isTracked)
		{
			person_array[i].is_tracked();
			person_array[i].add_position(Position(body_array->bodies[i].jointPositions[3].position, body_array->bodies[i].header));



			people_tracked++;
			ROS_INFO("Person: %d, time: [%d]",i, body_array->bodies[i].header.stamp.sec);

			if(person_array[i].is_stationary())
			{

				ROS_INFO("Person is not moving, speed: [%.3f]",person_array[i].get_speed());
			}
			else
				ROS_INFO("Person is moving, speed: [%.3f]",person_array[i].get_speed());


			ROS_INFO("Position");
			ROS_INFO("Head:[%.3f,%.3f,%.3f]",  body_array->bodies[i].jointPositions[3].position.x, body_array->bodies[i].jointPositions[3].position.y, body_array->bodies[i].jointPositions[3].position.z);
			
			ROS_INFO("Future position: [%.3f,%.3f,%.3f]", person_array[i].guess_future_position().point.x,person_array[i].guess_future_position().point.y, person_array[i].guess_future_position().point.z);
			//ROS_INFO("Left shoulder:[%.3f,%.3f,%.3f]",  body_array->bodies[i].jointPositions[4].position.x, body_array->bodies[i].jointPositions[4].position.y, body_array->bodies[i].jointPositions[4].position.z);
			//ROS_INFO("Right shoulder:[%.3f,%.3f,%.3f]", body_array->bodies[i].jointPositions[8].position.x, body_array->bodies[i].jointPositions[8].position.y, body_array->bodies[i].jointPositions[8].position.z);

			//ROS_INFO("Orientation:");
			//ROS_INFO("Head: [%.3f, %.3f, %.3f, %.3f]", body_array->bodies[i].jointOrientations[3].orientation.x, body_array->bodies[i].jointOrientations[3].orientation.y, body_array->bodies[i].jointOrientations[3].orientation.z, body_array->bodies[i].jointOrientations[3].orientation.w);
			//ROS_INFO("Spine base: [%.3f, %.3f, %.3f, %.3f]", body_array->bodies[i].jointOrientations[0].orientation.x, body_array->bodies[i].jointOrientations[0].orientation.y, body_array->bodies[i].jointOrientations[0].orientation.z, body_array->bodies[i].jointOrientations[0].orientation.w);
			//ROS_INFO("Neck: [%.3f, %.3f, %.3f, %.3f]", body_array->bodies[i].jointOrientations[2].orientation.x, body_array->bodies[i].jointOrientations[2].orientation.y, body_array->bodies[i].jointOrientations[2].orientation.z, body_array->bodies[i].jointOrientations[2].orientation.w);

			ROS_INFO("Distance from Cyborg:");
			ROS_INFO("Head: [%.3f]", distance_from_cyborg_calc(body_array->bodies[i].jointPositions[3].position));
			//ROS_INFO("Left Shoulder: [%.3f]", distance_from_cyborg_calc(body_array->bodies[i].jointPositions[4].position));
			//ROS_INFO("Right Shoulder: [%.3f]", distance_from_cyborg_calc(body_array->bodies[i].jointPositions[8].position));
			ROS_INFO("Spine base: [%.3f]", distance_from_cyborg_calc(body_array->bodies[i].jointPositions[0].position));
			//ROS_INFO("Spine mid: [%.3f]", distance_from_cyborg_calc(body_array->bodies[i].jointPositions[1].position));
		}

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

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe(bodyTopicName, 100, bodyListener);

  ros::spin();

  return 0;
}

