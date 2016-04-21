
#include "trollnode/Person.h"


double Person::get_time_diff(Position pos1, Position pos2)
{
	//printf("Get time diff: Time1: [%f], Time 2: [%f]\n", pos1.time, pos2.time);
	//printf("Time diff: [%f]\n", pos2.time - pos1.time );

	return pos2.time - pos1.time;

}

Position Person::get_earlier_position(float time_diff)
{
	//int time_diff= 1; //in seconds;
	int current_time = get_position().time;

	std::vector<Position>::iterator it = positions.end();
	it--;

	//printf("vector size: %i", positions.size());
	//printf("Time diff: %f \n", get_time_diff(*it, get_position()));

	//while (time_diff > current_time - it->header.stamp.sec && it != positions.begin())
	while (time_diff > get_time_diff(*it, get_position()) && it != positions.begin())
	{
		//printf("Time1: [%f], time2: [%f]\n",it->time, get_position().time);
		//printf("Time diff: %f \n", get_time_diff(*it, get_position()));
		it--;
	}

	return *it;

}

double Person::distance_between_positions_head(Position pos1, Position pos2)
{
	return sqrt( pow(pos2.head.x - pos1.head.x ,2) + pow(pos2.head.y - pos1.head.y ,2) + pow(pos2.head.z - pos1.head.z ,2) );
}

double Person::distance_between_positions_spine(Position pos1, Position pos2)
{
	return sqrt( pow(pos2.spine_base.x - pos1.spine_base.x ,2) + pow(pos2.spine_base.y - pos1.spine_base.y ,2) + pow(pos2.spine_base.z - pos1.spine_base.z ,2) );
}


bool Person::is_slowing_down()
{
	double speed = get_speed();

	double prev_speed = get_speed(get_earlier_position(1.5), get_earlier_position(1)); 

	if (speed < prev_speed)
	{
		return true;
	}
	else
		return false;

}


bool Person::is_stationary()
{
	double threshold = 0.1; 

	if(threshold > distance_between_positions_head(get_earlier_position(1), get_position()))
	{
		return true;

	}
	else
		return false;
}

bool Person::is_approaching()
{
	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_distance_to_cyborg(get_earlier_position(1));

	if(curr_dist < prev_dist)
		return true;
	else 
		return false;

}

bool Person::is_leaving()
{
	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_distance_to_cyborg(get_earlier_position(1));

	if(curr_dist > prev_dist)
		return true;
	else 
		return false;

}


double Person::get_speed()
{
	Position prev_pos = get_earlier_position(1);
	Position curr_pos = get_position();

	double speed_head = distance_between_positions_head(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);
	double speed_spine = distance_between_positions_spine(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);

	return (speed_spine + speed_head )/2;
}

double Person::get_speed(Position prev_pos, Position curr_pos)
{


	double speed_head = distance_between_positions_head(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);
	double speed_spine = distance_between_positions_spine(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);

	return (speed_spine + speed_head )/2;
}

Position Person::get_vector_between_points(Position pos1, Position pos2)
{
	Position vector = pos1;

	vector.head.x = pos2.head.x - pos1.head.x;
	vector.head.y = pos2.head.y - pos1.head.y;
	vector.head.z = pos2.head.z - pos1.head.z;
 
	return vector;
}

Position Person::add_vector_to_position(Position pos, Position vector)
{
	pos.head.x = pos.head.x + vector.head.x;
	pos.head.y = pos.head.y + vector.head.y;
	pos.head.z = pos.head.z + vector.head.z;

	return pos;
}


Position Person::guess_future_position()
{
	Position vector = get_vector_between_points(get_earlier_position(1), get_position());

	return add_vector_to_position(get_position(), vector);

}

double Person::get_distance_to_cyborg()
{
	Position pos = get_position();

	return sqrt( pow(pos.head.x,2) + pow(pos.head.y,2) + pow( pos.head.z,2));

}

double Person::get_distance_to_cyborg(Position pos)
{
	

	return sqrt( pow(pos.head.x,2) + pow(pos.head.y,2) + pow( pos.head.z,2));

}