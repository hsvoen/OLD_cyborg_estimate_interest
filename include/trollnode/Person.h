
#ifndef __PERSON_H_INCLUDED__
#define __PERSON_H_INCLUDED__


//Forward declaration
class Position;
class Person;

//======================================
// dependencies

#include "geometry_msgs/Point.h"
#include "k2_client/BodyArray.h"

#include <math.h>
#include <vector>
#include <stdio.h>
//===========================

class Position{

	//History of person at moment
	double speed;
	double cyborg_distance;

	//add history to positions for easier tuning
	bool stationary;
	bool slowing_down;
	bool speeding_up;
	bool moving_closer;
	bool moving_away;

public:
	geometry_msgs::Point head;
	geometry_msgs::Point spine_base;
	double time;


	Position(geometry_msgs::Point head_pos, geometry_msgs::Point spine_base_pos , double seconds)
	{
		head = head_pos;
		spine_base = spine_base_pos;
		time = seconds;
	}
	Position(){}
	Position(const Position &pos){head = pos.head; spine_base = pos.spine_base; time = pos.time;}

	Position add_vector(Position vector);
	Position get_vector_to_pos(Position pos);
	Position multiply_vector_with_number(double number);

	double get_distance_to_position(Position);
	double get_distance_to_cyborg();


	void print_position();
};



class Person 
{
	int number;
	bool isTracked;
	double speed;
	double distance_to_cyborg;

	std::vector<Position> positions;

public:
	Person(int person_number) {number = person_number; isTracked = false;}
	Person(int person_number, bool tracked) {number = person_number; isTracked = tracked;}

	void tracked(){isTracked = true;}
	void not_tracked(){isTracked = false; positions.clear(); }

	int add_position(Position pos){ positions.push_back(pos);}
	
	bool is_tracked(){return isTracked;}
	bool is_stationary();
	bool is_slowing_down();
	bool is_moving_faster();
	bool is_moving_closer();
	bool is_moving_away();
	bool is_moving_away(double starting_time);
	bool is_interested();
	bool is_approaching();
	bool is_leaving();

	Position get_position();
	Position get_earlier_position(float time_diff);

	double get_time_diff(Position, Position);
	//double distance_between_positions_head(Position, Position);
	//double distance_between_positions_spine(Position, Position);
	double get_speed();
	double get_speed(Position, Position);
	double get_distance_to_cyborg();

	Position guess_future_position();
	Position estimate_future_position(double seconds);
	//Position get_vector_between_points(Position,Position);
	//Position add_vector_to_position(Position pos, Position vector);

};



#endif // __PERSON_H_INCLUDED__ 
