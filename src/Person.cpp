
#include "trollnode/Person.h"


//============================== Position class functions =================

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

//For multiplying a speed vector with time to get estimated future position
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

/*
// returns avg. distance of head and spine to position
double Position::get_distance_to_position(Position pos)
{

	double dist_head = sqrt( pow(pos.head.x - head.x ,2) + pow(pos.head.y - head.y ,2) + pow(pos.head.z - head.z ,2) );
	double dist_spine = sqrt( pow(pos.spine_base.x - spine_base.x ,2) + pow(pos.spine_base.y - spine_base.y ,2) + pow(pos.spine_base.z - spine_base.z ,2) );
	
	return (dist_head + dist_spine)/2;
}
*/

//Ignores height distance!!!
// returns avg. distance of head and spine to position.
double Position::get_distance_to_position(Position pos)
{

	double dist_head = sqrt( pow(pos.head.x - head.x ,2)  + pow(pos.head.z - head.z ,2) );
	double dist_spine = sqrt( pow(pos.spine_base.x - spine_base.x ,2) + pow(pos.spine_base.z - spine_base.z ,2) );
	
	return (dist_head + dist_spine)/2;
}

/*
//Returns avg distance of head and spine to [0,0,0]
double Position::get_distance_to_cyborg()
{
	return (sqrt ( pow(head.x,2) + pow(head.y,2) + pow( head.z,2)) + sqrt(pow(spine_base.x,2) + pow(spine_base.y,2) + pow(spine_base.z,2) ) )/2 ;
}
*/

//IGNORES HEIGHT!!!
//Returns avg distance of head and spine to [0,0,0]
double Position::get_distance_to_cyborg()
{
	return (sqrt ( pow(head.x,2)  + pow( head.z,2)) + sqrt(pow(spine_base.x,2) + pow(spine_base.z,2) ) )/2 ;
}

void Position::print_position()
{
	//printf("time: [%f] seconds\n", time);
	printf("Position Head: [%.3f,%.3f,%.3f], spine: [%.3f,%.3f,%.3f] \n",head.x,head.y,head.z, spine_base.x, spine_base.y, spine_base.z);
}







/*===================== PERSON CLASS FUNCTIONS =======================================================*/



double Person::get_time_diff(Position pos1, Position pos2)
{
	return pos2.time - pos1.time;
}

Position Person::get_position()
{
	return positions.back();
}

//Returns the first position with a time stamp - current time > time diff
Position Person::get_earlier_position(float time_diff)
{
	int current_time = get_position().time;

	std::vector<Position>::iterator it = positions.end();
	it--;

	while (time_diff > get_time_diff(*it, get_position()) && it != positions.begin())
	{
		it--;
	}

	return *it;

}


double Person::get_speed()
{
	Position prev_pos = get_earlier_position(0.5);
	Position curr_pos = get_position();

	return prev_pos.get_distance_to_position(curr_pos)/get_time_diff(prev_pos,curr_pos);
}


double Person::get_speed(Position prev_pos, Position curr_pos)
{
	return prev_pos.get_distance_to_position(curr_pos)/get_time_diff(prev_pos,curr_pos);
}




double Person::get_distance_to_cyborg()
{
	return get_position().get_distance_to_cyborg();
}


double Person::get_distance_to_person(Person person)
{
	return get_position().get_distance_to_position(person.get_position());
}






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
	double threshold = 0.15; 

	if(threshold > get_speed())
	{
		return true;

	}
	else
		return false;

}


bool Person::is_moving_closer()
{
	double threshold = 0.15;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.5).get_distance_to_cyborg();

	if(curr_dist + threshold < prev_dist && !is_stationary())
		return true;
	else 
		return false;
}


bool Person::is_moving_away()
{
	double threshold = 0.15;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.5).get_distance_to_cyborg();


	if(curr_dist > prev_dist + threshold && !is_stationary())
		return true;
	else 
		return false;
	
}




bool Person::is_interested()
{
double social_distance = 2.5; //People standing inside this radius is classified as interested
double public_distance = 3.5; //distance people approach the robot for interaction
double fast_speed = 0.8;


	//case: person is close and stationary
	// Very likely the person is interested.
	if(social_distance > get_distance_to_cyborg() && is_stationary())
		return true;

	//Moving close within public distance
	else if (public_distance > get_distance_to_cyborg() && is_moving_closer() )
		return true;

	else
		return false;


}




bool Person::is_indecisive()
{
double social_distance = 2.5; //People standing inside this radius is classified as interested
double public_distance = 3.5; //distance people approach the robot for interaction
double fast_speed = 0.8;


	//Case: Within public distance, not approaching, not leaving and not walking fast
	if( public_distance > get_distance_to_cyborg() && ! is_moving_closer() && ! is_moving_away() && get_speed() < fast_speed)
		return true;
	//Case: outside of public distance and approaching
	else if (public_distance < get_distance_to_cyborg() && is_moving_closer() )
		return true;
	else
		return false;

}


bool Person::is_not_interested()
{
double social_distance = 2.5; //People standing inside this radius is classified as interested
double public_distance = 3.5; //distance people approach the robot for interaction
double fast_speed = 0.8;
	//Case: Is moving away from the cyborg
	if(is_leaving())
	{
		return true;
	}
	//Case: is far away and not approaching
	else if(public_distance < get_distance_to_cyborg() && !is_moving_closer())
	{

		return true;
	}
	//Case: Is close, passing and walking fast
	else if(public_distance < get_distance_to_cyborg() && !is_moving_closer() && !is_moving_away() && get_speed() < fast_speed)
		{
			return true;
		}
	else
		return false;



}


void Person::estimate_interest()
{
	if(is_interested())
	{
		interested = true;
		indecisive = false;
		not_interested = false;
	}
	else if(is_indecisive())
	{
		interested = false;
		indecisive = true;
		not_interested = false;
	}
	else if(is_not_interested())
	{
		interested = false;
		indecisive = false;
		not_interested = true;
	}
}



bool Person::is_approaching()
{
	//double approach_threshold = 1;

	if( !is_stationary() )
	{

		if(get_distance_to_cyborg() > estimate_future_position(1).get_distance_to_cyborg())
		{
			return true;
		}

	}

	return false;
}





//Could also check if estimated future position is away from the cyborg.
bool Person::is_leaving()
{

	double leave_treshold = 1;

	if( !is_stationary() )
	{

		if(get_distance_to_cyborg() < estimate_future_position(1).get_distance_to_cyborg())
		{
			return true;
		}

	}

	return false;
}




//updates position object with history states.
void Person::add_history_to_pos()
{

	positions.back().speed = get_speed();
	positions.back().cyborg_distance = get_distance_to_cyborg();
	positions.back().stationary = is_stationary();
	positions.back().slowing_down = is_slowing_down();
	positions.back().speeding_up = is_moving_faster();
	positions.back().moving_closer = is_moving_closer();
	positions.back().moving_away = is_moving_away();

}




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


