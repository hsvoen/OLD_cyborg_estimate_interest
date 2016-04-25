
#include "trollnode/Person.h"




Position Position::add_vector(Position vector)
{
	Position pos(*this);

	pos.head.x = head.x + vector.head.x;
	pos.head.y = head.y + vector.head.y;
	pos.head.z = head.z + vector.head.z;

	pos.spine_base.x = spine_base.x + vector.spine_base.x;
	pos.spine_base.y = spine_base.y + vector.spine_base.y;
	pos.spine_base.z = spine_base.z + vector.spine_base.z;

	return pos;

}

Position Position::get_vector_to_pos(Position pos)
{
	Position vector(pos);

	vector.head.x = pos.head.x - head.x;
	vector.head.y = pos.head.y - head.y;
	vector.head.z = pos.head.z - head.z;

	vector.spine_base.x = pos.spine_base.x - spine_base.x;
	vector.spine_base.y = pos.spine_base.y - spine_base.y;
	vector.spine_base.z = pos.spine_base.z - spine_base.z;

 
	return vector;
}

Position Position::multiply_vector_with_number(double number)
{

	head.x =  head.x * number;
	head.y =  head.y * number;
	head.z =  head.z * number;

	spine_base.x =  spine_base.x * number;
	spine_base.y =  spine_base.y * number;
	spine_base.z =  spine_base.z * number;

 
	return *this;
}

// returns avg. distance of head and spine to position
double Position::get_distance_to_position(Position pos)
{

	double dist_head = sqrt( pow(pos.head.x - head.x ,2) + pow(pos.head.y - head.y ,2) + pow(pos.head.z - head.z ,2) );
	double dist_spine = sqrt( pow(pos.spine_base.x - spine_base.x ,2) + pow(pos.spine_base.y - spine_base.y ,2) + pow(pos.spine_base.z - spine_base.z ,2) );
	
	return (dist_head + dist_spine)/2;
}

/*Returns avg distance of head and spine to [0,0,0]*/
double Position::get_distance_to_cyborg()
{

	return (sqrt ( pow(head.x,2) + pow(head.y,2) + pow( head.z,2)) + sqrt(pow(spine_base.x,2) + pow(spine_base.y,2) + pow(spine_base.z,2) ) )/2 ;

}


void Position::print_position()
{
	//printf("time: [%f] seconds\n", time);
	printf("Position Head: [%.3f,%.3f,%.3f], spine: [%.3f,%.3f,%.3f] \n",head.x,head.y,head.z, spine_base.x, spine_base.y, spine_base.z);
}










/*===================== PERSON CLASS FUNCTIONS =============================================================*/



double Person::get_time_diff(Position pos1, Position pos2)
{
	//printf("Get time diff: Time1: [%f], Time 2: [%f]\n", pos1.time, pos2.time);
	//printf("Time diff: [%f]\n", pos2.time - pos1.time );

	return pos2.time - pos1.time;

}

Position Person::get_position()
{
	return positions.back();
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

double Person::get_speed()
{
	Position prev_pos = get_earlier_position(0.5);
	Position curr_pos = get_position();

	return prev_pos.get_distance_to_position(curr_pos)/get_time_diff(prev_pos,curr_pos);

/*
	double speed_head = distance_between_positions_head(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);
	double speed_spine = distance_between_positions_spine(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);

	return (speed_spine + speed_head )/2;
	*/
}

double Person::get_speed(Position prev_pos, Position curr_pos)
{
	return prev_pos.get_distance_to_position(curr_pos)/get_time_diff(prev_pos,curr_pos);

/*
	double speed_head = distance_between_positions_head(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);
	double speed_spine = distance_between_positions_spine(prev_pos,curr_pos)/get_time_diff(prev_pos, curr_pos);

	return (speed_spine + speed_head )/2;
	*/
}

double Person::get_distance_to_cyborg()
{
	
	return get_position().get_distance_to_cyborg();
	/*Position pos = get_position();
	return sqrt( pow(pos.head.x,2) + pow(pos.head.y,2) + pow( pos.head.z,2));*/
}


/*
double Person::distance_between_positions_head(Position pos1, Position pos2)
{
	return sqrt( pow(pos2.head.x - pos1.head.x ,2) + pow(pos2.head.y - pos1.head.y ,2) + pow(pos2.head.z - pos1.head.z ,2) );
}

double Person::distance_between_positions_spine(Position pos1, Position pos2)
{
	return sqrt( pow(pos2.spine_base.x - pos1.spine_base.x ,2) + pow(pos2.spine_base.y - pos1.spine_base.y ,2) + pow(pos2.spine_base.z - pos1.spine_base.z ,2) );
}
*/

bool Person::is_slowing_down()
{
	double threshold = 0.1;

	double speed = get_speed();
	double prev_speed = get_speed(get_earlier_position(1), get_earlier_position(0.5)); 

	if (speed + threshold < prev_speed )
	{
		return true;
	}
	else
		return false;
}

bool Person::is_moving_faster()
{
	double threshold = 0.1;

	double speed = get_speed();
	double prev_speed = get_speed(get_earlier_position(1), get_earlier_position(0.5)); 

	if (speed > prev_speed + threshold)
	{
		return true;
	}
	else
		return false;
}


bool Person::is_stationary()
{
	double threshold = 0.2; 

	if(threshold > get_speed())
	{
		return true;

	}
	else
		return false;

	/*
	if(threshold > get_earlier_position(1).get_distance_to_position(get_position()))
	{
		return true;

	}
	else
		return false;
		*/
}

bool Person::is_moving_closer()
{
	double threshold = 0.1;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.5).get_distance_to_cyborg();

	if(curr_dist + threshold < prev_dist)
		return true;
	else 
		return false;

}

bool Person::is_moving_away()
{
	double threshold = 0.05;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.5).get_distance_to_cyborg();

	if(curr_dist > prev_dist + threshold)
		return true;
	else 
		return false;
}

bool Person::is_moving_away(double starting_time)
{
	double threshold = 0.05;

	double curr_dist = get_earlier_position(starting_time).get_distance_to_cyborg();
	double prev_dist = get_earlier_position(starting_time + 1).get_distance_to_cyborg();

	if(curr_dist > prev_dist + threshold)
		return true;
	else 
		return false;

}

bool Person::is_interested()
{
	double social_distance = 1.5; //distance people approach the robot for interaction
	double hesitant_distance = 2;  // distance hesitant people will linger
	double interested_speed = 0.5; //walking speed for people who are interested

	//case: person is close and stationary

	if(social_distance > get_distance_to_cyborg() && is_stationary())
		return true;

	//case: person is approaching and slowing down or walking slownly
	else if( is_approaching() && (is_slowing_down() || get_speed() <= interested_speed) )
		return true;



	//case: person is leaving: not interested
	else if (is_leaving())
		return false; 

	//case: person is not approaching and not slowing down: not interested
	else if (!is_approaching() || ! is_slowing_down() )
		return false;

	else
		return false;


}

bool Person::is_approaching()
{
	double approach_threshold = 1.5;

	for(int i = 1; i < 8; i++)
	{
		if(approach_threshold > estimate_future_position(i).get_distance_to_cyborg())
		{
			return true;
		}
	}

	return false;
}

//Could also check if estimated future position is away from the cyborg.
bool Person::is_leaving()
{


	for(int i = 0; i < 2; i++)
	{
		if(!is_moving_away(i))
		{
			return false;
		}
	}

	return true;

}




/*
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
*/

Position Person::guess_future_position()
{
	Position vector = get_earlier_position(1).get_vector_to_pos(get_position());


	return get_position().add_vector(vector);

}

Position Person::estimate_future_position(double seconds)
{
	Position vector = get_earlier_position(1).get_vector_to_pos(get_position());

	//this should probably have some sanity check when it comes to movement up and down.

	return get_position().add_vector(vector.multiply_vector_with_number(seconds));

}


