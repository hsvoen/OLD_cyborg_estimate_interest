
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

	//History of person
	double speed;
	double cyborg_distance;

	//add history to positions for easier tuning
	bool stationary;
	bool slowing_down;
	bool speeding_up;
	bool moving_closer;
	bool moving_away;

	//Constructors
	Position(){}
	Position(geometry_msgs::Point head_pos, geometry_msgs::Point spine_base_pos , double seconds)
	{
		head = head_pos;
		spine_base = spine_base_pos;
		time = seconds;
	}
	Position(const Position &pos){head = pos.head; spine_base = pos.spine_base; time = pos.time;}


	//Functions
	Position add_vector(Position vector);
	Position get_vector_to_pos(Position pos);
	Position multiply_vector_with_number(double number);

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

	std::vector<int> in_group_with;
	std::vector<Position> positions;

public:
	//Constructors
	Person(int person_number) {number = person_number; isTracked = false;}
	Person(int person_number, bool tracked) {number = person_number; isTracked = tracked;}

	void tracked(){isTracked = true;}
	void not_tracked(){isTracked = false; positions.clear(); } //deletes the position history

	int add_position(Position pos){ positions.push_back(pos); add_history_to_pos();}
	void add_history_to_pos();
	

	//Group functions
	void add_group_member(int member){in_group_with.push_back(member);} //need a remove group member at some point
	bool is_in_group(int member);
	int remove_member_from_group(int member);
	std::vector<int> get_group_members(){return in_group_with;}

	//What the person is doing.
	bool is_tracked(){return isTracked;}
	bool is_stationary();
	bool is_slowing_down();
	bool is_moving_faster();
	bool is_moving_closer(); //to the cyborg, [0,0,0]
	bool is_moving_away();



	bool is_interested(); //Function trying to guess if a person is interested in the cyborg from spatial relationship.
	//bool is_approaching(); 
	//bool is_leaving();
	//bool is_keeping_distance(); //True if person is not approaching and not leaving

	//Get functions
	Position get_position();
	Position get_earlier_position(float time_diff);

	double get_time_diff(Position, Position);
	double get_speed();
	double get_speed(Position, Position);
	double get_distance_to_cyborg();
	double get_distance_to_person(Person);


	Position guess_future_position();
	Position estimate_future_position(double seconds);

};



#endif // __PERSON_H_INCLUDED__ 
