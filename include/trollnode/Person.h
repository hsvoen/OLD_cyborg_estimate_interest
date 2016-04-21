
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
	geometry_msgs::Point point;
	std_msgs::Header header;

	Position(geometry_msgs::Point pos, std_msgs::Header stamp)
	{
		point = pos;
		header = stamp;
	}
	Position(){}
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
	bool is_stationary();

	Position get_position(){return positions.back();};
	Position get_earlier_position();

	float distance_between_positions(Position, Position);
	float get_speed();

	Position guess_future_position();
	Position get_vector_between_points(Position,Position);
	Position add_vector_to_position(Position pos, Position vector);

};



#endif // __PERSON_H_INCLUDED__ 
