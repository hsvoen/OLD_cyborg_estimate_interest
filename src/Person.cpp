
#include "trollnode/Person.h"

Position Person::get_earlier_position()
{
	int time_diff= 1; //in seconds;
	int current_time = get_position().header.stamp.sec;

	std::vector<Position>::iterator it = positions.end();
	it--;

	while (time_diff > current_time - it->header.stamp.sec && it != positions.begin())
	{

		it--;
	}

	return *it;

}

float Person::distance_between_positions(Position pos1, Position pos2)
{
	return sqrt( pow(pos2.point.x - pos1.point.x ,2) + pow(pos2.point.y - pos1.point.y ,2) + pow(pos2.point.z - pos1.point.z ,2) );
}


bool Person::is_stationary()
{
	float threshold = 0.2; //in meters ?
	Position pos1 = get_earlier_position();
	Position pos2 = get_position();

	//printf("earlier position: [%f,%f,%f], new position:[%f,%f,%f]\n", pos1.point.x,pos1.point.y,pos1.point.z, pos2.point.x,pos2.point.y,pos2.point.z);

	//printf("%f",distance_between_positions(get_earlier_position(), get_position()));

	if(threshold > distance_between_positions(get_earlier_position(), get_position()))
	{
		return true;

	}
	else
		return false;
}


float Person::get_speed()
{
	return distance_between_positions(get_earlier_position(), get_position())/1;
}

Position Person::get_vector_between_points(Position pos1, Position pos2)
{
	Position vector = pos1;

	vector.point.x = pos2.point.x - pos1.point.x;
	vector.point.y = pos2.point.y - pos1.point.y;
	vector.point.z = pos2.point.z - pos1.point.z;

	return vector;
}

Position Person::add_vector_to_position(Position pos, Position vector)
{
	pos.point.x = pos.point.x + vector.point.x;
	pos.point.y = pos.point.y + vector.point.y;
	pos.point.z = pos.point.z + vector.point.z;

	return pos;
}


Position Person::guess_future_position()
{
	Position vector = get_vector_between_points(get_earlier_position(), get_position());

	return add_vector_to_position(get_position(), vector);

}