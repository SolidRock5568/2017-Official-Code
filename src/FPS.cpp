//Included code imports
#include "math.h"
#include "Barrier.cpp"



class FPS
{

	//Calls are public
public:

	int ROW; // X = ROW
	int COL; // Y = COL
	double SPEED; // Speed for the Postion
	int XLimit; // X limit for Robot postion
	int YLimit; // Y limit for Robot Postion

	Barrier barrier; // Creates a Barrier for the postion

	//Default constructor
FPS()
	{
	ROW = 1;
	COL = 1;
	SPEED = .5;
	barrier = Barrier(0,0,0,0,0);
	XLimit = 0;
	YLimit = 0;
	}

// Constructor with ROW, COL, Speed, X Limit and Y Limit
FPS(int Row, int Col, double Speed, int X, int Y)
		{
	ROW = Row;
	COL = Col;
	SPEED = Speed;
	XLimit = X;
	YLimit = Y;
		}

//Returns Speed of postion
double GetSpeed()
{
	return SPEED;

}

//Sets Speed for current Postion
void SetSpeed(double Speed)
{
	SPEED = Speed;
}

//Returns COL
int GetCol()
{
	return COL;
}

//Returns ROW
int GetRow()
{
	return ROW;
}


// Returns Angle Difference of current Postion and Target Postion
double GetAngleDifference(FPS temp)
{

	int y1 = this->ROW; //Sets y1 to ROW
	int x1 = this->COL;  //Sets x1 to COL

	int x2 = temp.GetCol(); //Sets y2 to Target COL
	int y2 = temp.GetRow(); //Sets x2 to Target ROW

	//Test to see if the Two Postions are in a Line

	//Test for DOWN
	if (x1 == x2 && y1 > y2)
	{
		return 270;
	}
	//Test for UP
	else if (x1 == x2 && y1 < y2)
		{
			return 90;
		}
	//Test for LEFT
	else if (y1 == y2 && x1 < x2)
			{
				return 180;
			}
	//Test for RIGHT
	else if (y1 == y2 && x1 < x2)
				{
					return 360;
				}

	//Test for SAME POSTION
	else if (y1 == y2 && x1 == x2)
	{
		return 0;
	}

	//Begin angle math

	//Test to see if target is Left or Right of Current
	//Tests Right
	if ( x2 - x1 > 0)
	{
		//Test to see if target is up or down of target
		//Tests Up
		if (y2 - y1 > 0)
		{
			//Returns angle of first Quad
		double sideA = x2 - x1;
		double sideB = y2 - y1;

		return (atan(sideB/sideA) * 180/3.1415);
		}

		//Test DOWN
		else if (y2 - y1 < 0)
		{

			//Returns angle of First Quad plus 360 for 4th Quad
			double sideA = x2 - x1;
			double sideB = y2 - y1;
			double sideC = (atan(sideB/sideA) * 180/3.1415);

			return 360 + sideC;
		}
	}

	//Test LEFT
	else
	{
		//Test UP
		if (y2 - y1 > 0)
				{
			//Returns First Quad plus 180 for 2nd Quad
			double sideA = x2 - x1;
			double sideB = y2 - y1;
			double sideC = (atan(sideB/sideA) * 180/3.1415);

			return 90 + sideC + 90;

				}
		//Tests DOWN
		else
		{
			//Returns 1st Quad plus 180 for 3rd Quad
			double sideA = x2 - x1;
			double sideB = y2 - y1;
			double sideC = (atan(sideB/sideA) * 180/3.1415);

			return 180 + sideC;

				}

	}
//Default Return if all else fails
	return 0.0;
}

//Returns Distance between Current postion and Target Postion
double GetDistance(FPS temp)
{

	int y1 = this->ROW; //Sets y1 to ROW
	int x1 = this->COL; //Sets x1 to COL

	int x2 = temp.GetCol(); //Sets x2 to target COL
	int y2 = temp.GetRow(); //Sets y2 to target ROW

	double sideA = x2 - x1; // Gets side A Distance
	double sideB = y2 - y1; // Gets side B Distance

	//Returns Distance between Current and Target Point (Triangle HYP)
	return sqrt((sideA * sideA) + (sideB*sideB));

}

// Creates a Barrier for current point
void CreateBarrier(int Y_S,int Y_E,int X_S,int X_E)
{
	barrier = Barrier(Y_S,Y_E,X_S,X_E,SPEED);
}

//Check to see if barrier exists at current location
bool CheckSpot()
{
	if (barrier.GetXStart() > 0 && barrier.GetYStart() > 0)
	{
		return true;
	}
	return false;

}

//Returns point to move to, Avoiding Barriers. (DOES NOT WORK, IN PROGRESS)
int BarrierAvoid(FPS EndPostion)
{
	int x1 = ROW;
	int y1 = COL;
	int x2 = EndPostion.GetRow();
	int y2 = EndPostion.GetCol();

if ( x1 > x2 && y1 > y2)
{
	return 0;
}

return 0;

}


};
