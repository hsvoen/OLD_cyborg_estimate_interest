
#ifndef __PERSON_H_INCLUDED__
#define __PERSON_H_INCLUDED__


//Forward declaration
class Position;
class Person;

//======================================
// dependencies

#include <math.h>
#include <vector>
#include <stdio.h>


//Message files
#include "geometry_msgs/Point.h"
#include "k2_client/BodyArray.h"

//===========================







// Stores point [x,y,z] information about the head and spine position of a person relative to the cyborg.
class Position{

public:
	geometry_msgs::Point head;
	geometry_msgs::Point spine_base;
	double time; //ROS time for message being published, in seconds.


	//Constructors
	Position(){}
	Position(geometry_msgs::Point head_pos, geometry_msgs::Point spine_base_pos , double seconds)
	{
		head = head_pos;
		spine_base = spine_base_pos;
		time = seconds;
	}
	Position(const Position &pos){head = pos.head; spine_base = pos.spine_base; time = pos.time;}

	//get functions
	double get_distance_to_position(Position);
	double get_distance_to_cyborg();

	//Print to terminal
	void print_position();
};


/* Person stores the position of a person gotten from the kinect.
If the Kinect looses tracking, all data is deleted since it's impossible to know what happened.
The Cyborg is assumed to be at the Origo [0,0,0]
 */
class Person 
{
	int 	number; 	//Number of person from kinect, randomly assigned. between 0-5. 
	bool 	isTracked;	//The person is tracked by the kinect. Always check if a person is tracked before doing something.

	//Variables storing interest
	bool	interested;
	bool	indecisive;
	bool 	hesitating;
	bool	not_interested;

	std::vector<Position> positions;

public:
	//Constructors
	Person(int person_number) {number = person_number; isTracked = false;}
	Person(int person_number, bool tracked) {number = person_number; isTracked = tracked;}

	void tracked(){isTracked = true;}
	void not_tracked(){isTracked = false; positions.clear(); interested = false; indecisive = false; hesitating = false; not_interested = false;} //deletes the position history

	int add_position(Position pos){ positions.push_back(pos);}
	

	//What the person is doing.
	bool is_stationary();
	bool is_stationary(double seconds);
	bool is_moving_closer(); //to the cyborg, [0,0,0]
	bool is_moving_away();

	void estimate_interest();
	bool is_interested(); //Function trying to guess if a person is interested in the cyborg from spatial relationship.
	bool is_indecisive();
	bool is_hesitating();
	bool is_not_interested();


	//Get functions
	Position get_position();
	Position get_earlier_position(float time_diff);

	bool get_tracked(){return isTracked;}
	double get_time_diff(Position, Position);
	double get_speed();
	double get_speed(Position, Position);
	double get_distance_to_cyborg();

	bool get_interested(){return interested;}
	bool get_indecisive(){return indecisive;}
	bool get_hesitating(){return hesitating;}
	bool get_not_interested(){return not_interested;}


};



#endif // __PERSON_H_INCLUDED__ 
