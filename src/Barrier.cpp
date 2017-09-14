/*
 * Barrier.cpp
 *
 *  Created on: Aug 7, 2016
 *      Author: Alex's
 */




class Barrier
{
	//Sets class features as public
public :
	int Y_start_location;
	int X_start_location;
	int Y_end_location;
	int X_end_location;
	int traverse_Speed;

	bool Avoid;
	bool Moving;
	bool Collide;

	//Default constructor
	Barrier()
	{
		Y_start_location = 0;
		Y_end_location = 0;
		X_start_location = 0;
		X_end_location = 0;
		traverse_Speed = 0;

		Avoid = false;
		Moving = false;
		Collide = false;

	}

	//Sets a barrier
	Barrier(int Y_S , int Y_E , int X_S, int X_E , int Speed)
		{
			Y_start_location = Y_S;
			Y_end_location = Y_E;
			X_start_location = X_S;
			X_end_location = X_E;
			traverse_Speed = Speed;


			Avoid = false;
			Moving = false;
			Collide = false;

		}

	//Returns the starting Y Corner
	int GetYStart()
	{
		return Y_start_location;
	}

	//Returns the Starting X Corner
	int GetXStart()
	{
		return X_start_location;
	}

	//Returns the Ending Y Corner
	int GetYEnd()
	{
		return Y_end_location;
	}

	//Returns the Ending X Corner
	int GetXEnd()
	{
		return X_end_location;
	}

	//Returns speed for travel
	int GetTraverseSpeed()
	{
		return traverse_Speed;
	}

	//Returns Sets Y Start
	void SetYStart(int Y_S)
		{
			Y_start_location = Y_S;
		}

	//Sets X Start
	void SetXStart( int X_S)
		{
			X_start_location = X_S;
		}

	//Set Y End
	void SetYEnd(int Y_E)
		{
			Y_end_location = Y_E;
		}

	//Set X End
	void SetXEnd(int X_E)
		{
			X_end_location = X_E;
		}

	//Set Traverse Speed
	void SetTraverseSpeed( int SPEED)
		{
		traverse_Speed = SPEED;
		}




};
