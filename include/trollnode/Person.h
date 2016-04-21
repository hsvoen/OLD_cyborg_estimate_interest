
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


public:
	geometry_msgs::Point head;
	geometry_msgs::Point spine_base;
	//std_msgs::Header header;
	double time;

	Position(geometry_msgs::Point head_pos, geometry_msgs::Point spine_base_pos , double seconds)
	{
		head = head_pos;
		spine_base = spine_base_pos;
		time = seconds;
	}
	Position(){}
};



class Person 
{
	int number;
	bool isTracked;
	double speed;
	double distance_to_cyborg;

	std::vector<Position> positions;

public:
	Person() {isTracked = false; speed = 0;}
	Person(bool tracked) {isTracked = tracked;}

	int add_position(Position pos){ positions.push_back(pos);}
	void is_tracked(){isTracked = true;}
	bool is_stationary();
	bool is_slowing_down();
	bool is_approaching();
	bool is_leaving();


	Position get_position(){return positions.back();};
	Position get_earlier_position(float time_diff);

	double get_time_diff(Position, Position);
	double distance_between_positions_head(Position, Position);
	double distance_between_positions_spine(Position, Position);
	double get_speed();
	double get_speed(Position, Position);
	double get_distance_to_cyborg();
	double get_distance_to_cyborg(Position);
	
	Position guess_future_position();
	Position get_vector_between_points(Position,Position);
	Position add_vector_to_position(Position pos, Position vector);

};



#endif // __PERSON_H_INCLUDED__ 
