
#include "estimate_interest/Person.h"


//============================== Position class functions =================

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
	Position prev_pos = get_earlier_position(0.1);
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




bool Person::is_stationary()
{
	double threshold = 0.08; 



	if(threshold > get_earlier_position(0.2).get_distance_to_position(get_position()))
		return true;
	else
		return false;

}

bool Person::is_stationary(double seconds)
{
	double threshold = 0.2; 

	if(threshold > get_earlier_position(seconds).get_distance_to_position(get_position()))
		return true;
	else
		return false;
}




bool Person::is_moving_closer()
{
	double threshold = 0.08;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.2).get_distance_to_cyborg();

	if(curr_dist + threshold < prev_dist && !is_stationary())
		return true;
	else 
		return false;
}


bool Person::is_moving_away()
{
	double threshold = 0.08;

	double curr_dist = get_distance_to_cyborg();
	double prev_dist = get_earlier_position(0.2).get_distance_to_cyborg();


	if(curr_dist > prev_dist + threshold && !is_stationary())
		return true;
	else 
		return false;
	
}




bool Person::is_interested()
{
	double social_distance = 2.5; 
	double public_distance = 3.5; 
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
	double social_distance = 2.5; 
	double public_distance = 3.5; 
	double fast_speed = 0.8;

	//Case: Within public distance but not in interaction distance and stationary
	if(public_distance > get_distance_to_cyborg() && social_distance < get_distance_to_cyborg() && is_stationary() && ! is_stationary(3) )
		return true;
	//Case: Within public distance, not approaching, not leaving and not walking fast
	else if( public_distance > get_distance_to_cyborg() && ! is_moving_closer() && ! is_moving_away() && get_speed() < fast_speed && ! is_stationary())
		return true;
	//Case: outside of public distance and approaching
	else if (public_distance < get_distance_to_cyborg() && is_moving_closer() )
		return true;
	else
		return false;
}


bool Person::is_hesitating()
{
	double social_distance = 2.5; 
	double public_distance = 3.5; 
	double fast_speed = 0.8;

	//Case: Within public distance, not within social distance, and have been stationary for 3 seconds.
	if(public_distance > get_distance_to_cyborg() && social_distance < get_distance_to_cyborg() && is_stationary(3 ))
		return true;
	else
		return false;
}


bool Person::is_not_interested()
{
	double social_distance = 2.5; 
	double public_distance = 3.5; 
	double fast_speed = 0.8;
	//Case: Is moving away from the cyborg
	if(is_moving_away())
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
		hesitating = false;
		not_interested = false;
	}
	else if(is_indecisive())
	{
		interested = false;
		indecisive = true;
		hesitating = false;
		not_interested = false;
	}
	else if(is_hesitating())
	{
		interested = false;
		indecisive = false;
		hesitating = true;
		not_interested = false;
	}
	else if(is_not_interested())
	{
		interested = false;
		indecisive = false;
		hesitating = false;
		not_interested = true;
	}
}








