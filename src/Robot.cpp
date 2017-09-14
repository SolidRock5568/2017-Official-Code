#include <thread>
#include "WPILib.h"
#include <CameraServer.h>
#include <IterativeRobot.h>


#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cameraData.cpp"
#include "AHRS.h"
#include "FPS.cpp"
#include "Math.h"





class Robot: public frc::IterativeRobot {


public:




	Joystick mainJoystick;
	Joystick xboxJoystick;
	Victor leftDriveWheel_1;
	Victor leftDriveWheel_2;
	Victor rightDriveWheel_1;
	Victor rightDriveWheel_2;
	Victor shooterMotor;
	Victor shooterInfeedMotor;
	Victor climbMotor;
	Victor intakeMotor;

	Solenoid targetLight;

	Servo rightGear;
	Servo leftGear;

	AnalogInput ultrasonicLeft;

	AHRS *driveGyro;

	Encoder leftEncoder;
	Encoder rightEncoder;

	FPS Field[324][324];





	cs::UsbCamera cameraFront;
	cs::UsbCamera cameraBack;
	cs::CvSink cvSinkFront;
	cs::CvSink cvSinkBack;
	cs::CvSource outputStream;
	cv::Mat cameraFeed;
	cv::Mat HSV;
	cv::Mat threshold;
	cv::Mat erodeElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(10,10));
	cv::Mat dilateElement = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(30,30));
    cv::Mat temp;

	float xIn;
	float yIn;
	double robotLength;
	int ultrasonicMax;
	int ultrasonicMin;
	bool hit;
	bool OuterRange;
	bool InnerRange;
	static const int cameraFeed_WIDTH = 640;
	static const int cameraFeed_HEIGHT = 480;
	static const int MAX_NUM_OBJECTS=10;
	static const int MIN_OBJECT_AREA = 20*20;
	const int MAX_OBJECT_AREA = cameraFeed_HEIGHT*cameraFeed_WIDTH/1.5;

	int targetXPos;
	int targetYPos;
	int distance;
	double area;

	bool openGear;



		int robotBaseHeading;
		int robotCurrentX;
		int robotCurrentY;
		int time;
		double rightSpeed;
		double leftSpeed;
		double targetDistance;
		double currentDistance;
		double startDistance;
		int currentHeading;
		int targetHeading;
		double distancePerSecond;
		double howFarOffCourse;
		double timeOffCourse;
		bool slowStop;
		bool slowStart;
		bool reachedTarget;
		bool areOffCourse;
		bool areObjects;

		bool turnLeft;
		bool turnRight;
		bool hasntRun;

		int robotAngle;

	int field;
	int zone;
	int action;

	int gearX;
	int gearY;

	bool intake;
	bool flipFront;
	int xDistance;
	int turns;

	void advancedMove(FPS target)
	{
	targetDistance = Field[robotCurrentX][robotCurrentY].GetDistance(target);
	basicMove(targetDistance);
	robotCurrentX = target.GetRow();
	robotCurrentY = target.GetCol();
	}
	void advancedTurn(FPS target)
	{
		targetHeading = Field[robotCurrentX][robotCurrentY].GetAngleDifference(target);
		currentHeading = driveGyro->GetAngle();
		if((currentHeading/360) > 1 || (currentHeading/360) < -1)
		{
			currentHeading = abs(currentHeading) % 360;

			if(driveGyro->GetAngle() < 0)
			{
				currentHeading = -currentHeading;
			}
		}
		if( currentHeading < 0)
		{
			currentHeading = currentHeading + 360;
		}
		if(currentHeading - targetHeading > 180)
		{
			targetHeading = targetHeading - 360;
		}

		targetHeading = currentHeading+targetHeading;

		basicTurn(targetHeading);
	}
	void basicMove(double inches)
	{
		leftEncoder.Reset();
		rightEncoder.Reset();
		startDistance = leftEncoder.GetDistance();
		currentDistance = leftEncoder.GetDistance();
		targetDistance = inches;
		int stuckOffset = 0;
		int count = 0;
		int stuckDistance = currentDistance;

		rightSpeed = 0;
		leftSpeed = 0;

		currentHeading = driveGyro->GetAngle();
		targetHeading = currentHeading;

		SmartDashboard::PutNumber("TargetDistance" , targetDistance);
		SmartDashboard::PutNumber("CurrentDistance" , currentDistance);
		SmartDashboard::PutNumber("StartDistance" , startDistance);
		SmartDashboard::PutNumber("currentHeading" , currentHeading);
		SmartDashboard::PutNumber("TargetHeading" , targetHeading);
		if(targetDistance > 0)
		{

		while (targetDistance - .3 > currentDistance)
		{
			currentHeading = driveGyro->GetAngle();

			if(currentHeading < targetHeading + .5 && currentHeading > targetHeading - .5)
			{

				rightSpeed = 0;
				leftSpeed = 0;
			}

			else if (currentHeading > targetHeading)
			{
				leftSpeed = .2;
			}
			else
			{
				rightSpeed = .2;
			}

			if(currentDistance == stuckDistance)
			{
				count++;
			}
			else
			{
				stuckDistance = currentDistance;
			}
			if(count > 30)
			{
				count = 0;
				stuckOffset = stuckOffset + .05;
			}


			rightDriveWheel_1.Set( .5 + rightSpeed + stuckOffset);
			rightDriveWheel_2.Set( .5 + rightSpeed + stuckOffset);
			leftDriveWheel_1.Set(-.5 - leftSpeed  - stuckOffset);
			leftDriveWheel_2.Set( -.5 - leftSpeed - stuckOffset);


			if(leftEncoder.GetDistance() > rightEncoder.GetDistance())
			{
			currentDistance = leftEncoder.GetDistance();
			}
			else
			{
				currentDistance = rightEncoder.GetDistance();
			}

			SmartDashboard::PutNumber("TargetDistance" , targetDistance);
			SmartDashboard::PutNumber("CurrentDistance" , currentDistance);
			SmartDashboard::PutNumber("StartDistance" , startDistance);
			SmartDashboard::PutNumber("currentHeading" , currentHeading);
			SmartDashboard::PutNumber("TargetHeading" , targetHeading);

		}
		rightDriveWheel_1.Set(-.3);
		rightDriveWheel_2.Set(-.3);
		leftDriveWheel_1.Set(.3);
		leftDriveWheel_2.Set(.3);

	}
		else if(targetDistance < 0)
				{

				while (targetDistance + .3 < currentDistance )
				{
					currentHeading = driveGyro->GetAngle();

					if(currentHeading < targetHeading + .5 && currentHeading > targetHeading - .5)
					{

						rightSpeed = 0;
						leftSpeed = 0;
					}

					else if (currentHeading < targetHeading)
					{
						leftSpeed = .2;
					}
					else
					{
						rightSpeed = .2;
					}
					if(currentDistance == stuckDistance)
								{
									count++;
								}
								else
								{
									stuckDistance = currentDistance;
								}
								if(count > 30)
								{
									count = 0;
									stuckOffset = stuckOffset + .05;
								}



					rightDriveWheel_1.Set( -.5 - rightSpeed - stuckOffset);
					rightDriveWheel_2.Set( -.5 - rightSpeed - stuckOffset);
					leftDriveWheel_1.Set(.5 + leftSpeed + stuckOffset);
					leftDriveWheel_2.Set( .5 + leftSpeed + stuckOffset);


					if(leftEncoder.GetDistance() > rightEncoder.GetDistance())
								{
								currentDistance = leftEncoder.GetDistance();
								}
								else
								{
									currentDistance = rightEncoder.GetDistance();
								}

					SmartDashboard::PutNumber("TargetDistance" , targetDistance);
					SmartDashboard::PutNumber("CurrentDistance" , currentDistance);
					SmartDashboard::PutNumber("StartDistance" , startDistance);
					SmartDashboard::PutNumber("currentHeading" , currentHeading);
					SmartDashboard::PutNumber("TargetHeading" , targetHeading);

				}
				rightDriveWheel_1.Set(.3);
				rightDriveWheel_2.Set(.3);
				leftDriveWheel_1.Set(-.3);
				leftDriveWheel_2.Set(-.3);
			}

		Wait(.1);
		rightDriveWheel_1.Set(0);
		rightDriveWheel_2.Set(0);
		leftDriveWheel_1.Set(0);
		leftDriveWheel_2.Set(0);
	}
    void basicTurn(double angle)
	{
		bool turnDirection;
		int count = 0;
		double speedLeft = 0;
		double speedRight = 0;
		//false is left; true is right
		driveGyro -> Reset();
		targetHeading = angle;
		currentHeading = driveGyro -> GetAngle();
		double stuckHeading = currentHeading;
		rightSpeed = 0;
		leftSpeed = 0;


		if (angle < 0)
		{
			turnDirection = false;
			if (angle < -180)
			{
				turnDirection = true;
				targetHeading = angle + 360;
			}
		}
		else
		{
			turnDirection = true;
			if(angle > 180)
			{
				turnDirection = false;
				targetHeading = angle - 360;
			}
		}

		SmartDashboard::PutBoolean("TurnDirection" , turnDirection);
		if (turnDirection == false) // turn left
		{
			while(currentHeading > targetHeading  + .3)
			{
				rightDriveWheel_1.Set(-.35 - speedLeft);
				rightDriveWheel_2.Set(-.35 - speedLeft);
				leftDriveWheel_1.Set(-.35 - speedRight);
				leftDriveWheel_2.Set(-.35 - speedRight);
				currentHeading = driveGyro-> GetAngle();

				SmartDashboard::PutNumber("CurrentHeading1", currentHeading);
				SmartDashboard::PutNumber("TargetHeading1", targetHeading);
			}

		}
		else // Turn Right
		{
			while(currentHeading < targetHeading - .3 )
			{
				leftDriveWheel_1.Set(.35 + speedLeft);
				leftDriveWheel_2.Set(.35 + speedLeft);
				rightDriveWheel_1.Set(.35+ speedRight);
			    rightDriveWheel_2.Set(.35+ speedRight);
			    currentHeading = driveGyro-> GetAngle();

			}

		}
		rightDriveWheel_1.Set(0);
		rightDriveWheel_2.Set(0);
		leftDriveWheel_1.Set(0);
		leftDriveWheel_2.Set(0);
		Wait(.3);

		SmartDashboard::PutNumber("CurrentHeading1", currentHeading);
		SmartDashboard::PutNumber("TargetHeading1", targetHeading);
		currentHeading = driveGyro->GetAngle();

		SmartDashboard::PutNumber("CurrentHeading1", currentHeading);
		SmartDashboard::PutNumber("TargetHeading1", targetHeading);
		if(turnDirection)
		{
			if(targetHeading < currentHeading +.2 )
			{
				while(currentHeading > targetHeading + .1)
				{
					rightDriveWheel_1.Set(-.30 - speedLeft);
					rightDriveWheel_2.Set(-.30 - speedLeft);
					leftDriveWheel_1.Set(-.30 -  speedRight);
					leftDriveWheel_2.Set(-.30 - speedRight);
					currentHeading = driveGyro-> GetAngle();

					SmartDashboard::PutNumber("CurrentHeading1", currentHeading);
					SmartDashboard::PutNumber("TargetHeading1", targetHeading);
				}
				rightDriveWheel_1.Set(.5);
				rightDriveWheel_2.Set(.5);
				leftDriveWheel_1.Set(.5);
				leftDriveWheel_2.Set(.5);
			}
		}
		else
		{
			if(targetHeading  > currentHeading - .2 )
					{

			while(currentHeading < targetHeading - .1)
			{
				leftDriveWheel_1.Set(.30+ speedLeft);
				leftDriveWheel_2.Set(.30 + speedLeft);
				rightDriveWheel_1.Set(.30+ speedRight);
				rightDriveWheel_2.Set(.30+ speedRight);
				currentHeading = driveGyro-> GetAngle();

			}

		}

	}
		rightDriveWheel_1.Set(0);
		rightDriveWheel_2.Set(0);
		leftDriveWheel_1.Set(0);
		leftDriveWheel_2.Set(0);
}


	void advancedMovement(double distance, double angle)
	{
			leftEncoder.Reset();
			startDistance = leftEncoder.GetDistance();
			currentDistance = startDistance;
			targetDistance = distance;
			double powerChange = 0.0;


			rightSpeed = 0;
			leftSpeed = 0;

			currentHeading = driveGyro->GetAngle();
			targetHeading = angle;
			if(angle < 0)
			{
				angle = angle - 1.5;
			}
			else
			{
				angle = angle + 3.5;
			}

			SmartDashboard::PutNumber("TargetDistance AdvancedMove" , targetDistance);
			SmartDashboard::PutNumber("CurrentDistance AdvancedMove" , currentDistance);
			SmartDashboard::PutNumber("StartDistance AdvancedMove" , startDistance);
			SmartDashboard::PutNumber("currentHeading AdvancedMove" , currentHeading);
			SmartDashboard::PutNumber("TargetHeading AdvancedMove" , targetHeading);

			if(targetDistance > 0)
					{

					while (targetDistance - 2 > currentDistance)
					{
						currentHeading = driveGyro->GetAngle();
						powerChange = ((currentHeading - targetHeading) * .04);
						if(powerChange < 0)
						{
							powerChange = -powerChange;
						}

						if(currentHeading < targetHeading + .1 && currentHeading > targetHeading - .1)
						{

							rightSpeed = 0;
							leftSpeed = 0;
						}

						else if (currentHeading > targetHeading)
						{
							//leftSpeed = .15;
							//rightSpeed = -.1;
							leftSpeed = powerChange;
							//rightSpeed = 0;
						}
						else
						{
							//rightSpeed = .15;
							//leftSpeed = -.1;
							rightSpeed = powerChange;
							//leftSpeed = 0;
						}


						if(mainJoystick.GetRawButton(1))
								{
							rightDriveWheel_1.Set(0);
							rightDriveWheel_2.Set(0);
							leftDriveWheel_1.Set(0);
							leftDriveWheel_2.Set(0);

									return;
								}


						rightDriveWheel_1.Set( .35 + rightSpeed);
						rightDriveWheel_2.Set( .35 + rightSpeed);
						leftDriveWheel_1.Set(-.35 - leftSpeed);
						leftDriveWheel_2.Set( -.35 - leftSpeed);


						if(leftEncoder.GetDistance() > rightEncoder.GetDistance())
									{
									currentDistance = leftEncoder.GetDistance();
									}
									else
									{
										currentDistance = rightEncoder.GetDistance();
									}

						SmartDashboard::PutNumber("TargetDistance" , targetDistance);
						SmartDashboard::PutNumber("CurrentDistance" , currentDistance);
						SmartDashboard::PutNumber("StartDistance" , startDistance);
						SmartDashboard::PutNumber("currentHeading" , currentHeading);
						SmartDashboard::PutNumber("TargetHeading" , targetHeading);

					}
					rightDriveWheel_1.Set(-.1);
					rightDriveWheel_2.Set(-.1);
					leftDriveWheel_1.Set(.1);
					leftDriveWheel_2.Set(.1);

				}
					else if(targetDistance < 0)
							{

							while (targetDistance + 2 < currentDistance )
							{
								currentHeading = driveGyro->GetAngle();
								powerChange = ((currentHeading - targetHeading) * .04);
								if(powerChange < 0)
								{
									powerChange = -powerChange;
								}

								if(currentHeading < targetHeading + .2 && currentHeading > targetHeading - .2)
								{

									rightSpeed = 0;
									leftSpeed = 0;
								}

								else if (currentHeading < targetHeading)
								{
									leftSpeed = .15;
									rightSpeed = -.1;
								}
								else
								{
									rightSpeed = .15;
									leftSpeed = -.1;
								}

											if(mainJoystick.GetRawButton(1))
																			{
																		rightDriveWheel_1.Set(0);
																		rightDriveWheel_2.Set(0);
																		leftDriveWheel_1.Set(0);
																		leftDriveWheel_2.Set(0);

																				return;
																			}



								rightDriveWheel_1.Set( -.35 - rightSpeed );
								rightDriveWheel_2.Set( -.35 - rightSpeed);
								leftDriveWheel_1.Set(.35 + leftSpeed);
								leftDriveWheel_2.Set( .35 + leftSpeed);


								currentDistance = leftEncoder.GetDistance();

								SmartDashboard::PutNumber("TargetDistance" , targetDistance);
								SmartDashboard::PutNumber("CurrentDistance" , currentDistance);
								SmartDashboard::PutNumber("StartDistance" , startDistance);
								SmartDashboard::PutNumber("currentHeading" , currentHeading);
								SmartDashboard::PutNumber("TargetHeading" , targetHeading);

							}

						}
					rightDriveWheel_1.Set(.3);
					rightDriveWheel_2.Set(.3);
					leftDriveWheel_1.Set(-.3);
					leftDriveWheel_2.Set(-.3);
					Wait(.1);

					rightDriveWheel_1.Set(0);
					rightDriveWheel_2.Set(0);
					leftDriveWheel_1.Set(0);
					leftDriveWheel_2.Set(0);


	}

	void setRobotOnCamera()
	{
		currentHeading = driveGyro->GetAngle();
		targetHeading = driveGyro->GetAngle();
		VisionThread();
		xDistance = 0;
		if(mainJoystick.GetRawButton(1))
		{
			return;
		}



		if(targetXPos > 0)
		{
			xDistance = getUltrasonicModeLeft() - 13;
			int turn;
			turn = targetXPos - 320;
			targetHeading = currentHeading + (turn/14);
			SmartDashboard::PutNumber("Needtoturnto" ,targetHeading);
			SmartDashboard::PutNumber("AlreadyAt" , currentHeading);
			SmartDashboard::PutNumber("Turnbypixels" , turn);
			if(targetXPos + 5 > 320 && targetXPos - 5 < 320)
			{
				xDistance = getUltrasonicModeLeft() -13;
				SmartDashboard::PutNumber("UltraLeftAuto" , xDistance);
				Wait(.1);
				//picturePost();
				VisionThread();
				turn = targetXPos - 320;
				targetHeading = currentHeading + (turn/14);
				if(targetXPos + 5 > 320 && targetXPos - 5 < 320)
				{

					turns = 1;
					if(targetXPos > 320)
					{
						turnLeft = true;
						turnRight = false;
					}
					else if(targetXPos < 320)
					{
						turnLeft = false;
						turnRight = true;
					}
					if(xDistance < 30)
					{
						setRobotOnCamera();
					}
					SmartDashboard::PutBoolean("TestMoveFroward1" , true);
					advancedMovement(xDistance,targetHeading);


				}
				else
				{

					if(xDistance < 30)
										{
											setRobotOnCamera();
										}
					SmartDashboard::PutBoolean("TestMoveFroward2" , true);
					advancedMovement(xDistance,targetHeading);


				}
			}
			else
			{

				SmartDashboard::PutBoolean("TestMoveFroward3" , false);
				SmartDashboard::PutNumber("XDISTANCEFINAL" , xDistance);
				SmartDashboard::PutNumber("TargetHeadingFINAL" , targetHeading);
				SmartDashboard::PutNumber("XPOSFINAL" , targetXPos);
				if(xDistance < 30)
									{
										setRobotOnCamera();
									}
				advancedMovement(xDistance,targetHeading);
			}

		}
			else if(targetXPos == 0)
			{

				setRobotOnCamera();
			}

		leftDriveWheel_1.Set(0);
		leftDriveWheel_2.Set(0);
		rightDriveWheel_1.Set(0);
		rightDriveWheel_2.Set(0);




	}



	/*void picturePost()
	{
		makeImage();
		editImage();
		getDataFromImage();

	}
	void makeImage()
	{

	if (cvSinkFront.GrabFrame(cameraFeed) == 0) {
		// Send the output the error.
		outputStream.NotifyError(cvSinkFront.GetError());
		// skip the rest of the current iteration
		//continue;
	}
	else
	{
		SmartDashboard::PutBoolean("Camera Found" , true);
	}

	}

	void editImage()
	{
		cv::cvtColor(cameraFeed,HSV,cv::COLOR_BGR2HSV);
		cv::inRange(HSV,cv::Scalar(47,50,100),cv::Scalar(58,255,255),threshold);


		cv::dilate(threshold,threshold,dilateElement);
		cv::dilate(threshold,threshold,dilateElement);
		cv::erode(threshold,threshold,erodeElement);
		cv::erode(threshold,threshold,erodeElement);



	}
	void getDataFromImage()
	{
		std::vector <cameraData> Object;
							std::vector< std::vector<cv::Point> > contours;
							std::vector<cv::Vec4i> hierarchy;

							threshold.copyTo(temp);

							//find contours of filtered image using openCV findContours function
								findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
								//use moments method to find our filtered object


								bool objectFound = false;

								if (hierarchy.size() > 0) {
									int numObjects = hierarchy.size();
									//SmartDashboard::PutNumber("Amount of Objects" , numObjects);
									//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
									if(numObjects<MAX_NUM_OBJECTS){
										for (int index = 0; index >= 0; index = hierarchy[index][0]) {



											cv::Moments moment = moments((cv::Mat)contours[index]);
											double area = moment.m00;

											//if the area is less than 20 px by 20px then it is probably just noise
											//if the area is the same as the 3/2 of the image size, probably just a bad filter
											//we only want the object with the largest area so we safe a reference area each
											//iteration and compare it to the area in the next iteration.
											if(area>MIN_OBJECT_AREA){

												cameraData object;





												object.setXPos(moment.m10/area);
												object.setYPos(moment.m01/area);

												Object.push_back(object);

												objectFound = true;

											}else objectFound = false;


										}
										//let user know you found an object
										if(objectFound ==true){


											OuterRange = true;
											SmartDashboard::PutNumber("Number of Objects" , Object.size());



											if(Object.size() > 1)
											{
												int x = (Object.at(0).getXPos() + Object.at(1).getXPos())/2;
												int y = (Object.at(0).getYPos() + Object.at(1).getYPos())/2;
												SmartDashboard::PutNumber("TargetXPos" ,x);
												SmartDashboard::PutNumber("TargetYPos" ,y);
												targetYPos = y;
												targetXPos = x;


											}
											else if (Object.size() == 1)
											{
												int x = Object.at(0).getXPos();

												int y = Object.at(0).getYPos();

												targetYPos = y;
												targetXPos = x;
											}




											SmartDashboard::PutNumber("TargetXPos" ,targetXPos);
											SmartDashboard::PutNumber("TargetYPos" ,targetYPos);
											if(targetXPos + 200 >= 320 || targetXPos - 200 <= 320)
											{

												hit = true;
												OuterRange = false;

												SmartDashboard::PutBoolean("Hit?" , hit);


											}


										}
										SmartDashboard::PutBoolean("OuterRange", OuterRange);
										SmartDashboard::PutBoolean("Hit", hit);
									}
								}



	}*/



	void VisionThread() {

					// Tell the CvSink to grab a cameraFeed from the camera and put it
					// in the source mat.  If there is an error notify the output.
					if (cvSinkFront.GrabFrame(cameraFeed) == 0) {
						// Send the output the error.
						outputStream.NotifyError(cvSinkFront.GetError());
						// skip the rest of the current iteration

					}
					SmartDashboard::PutNumber("TargetXPos" ,0);
					SmartDashboard::PutNumber("TargetYPos" ,0);
					cv::cvtColor(cameraFeed,HSV,cv::COLOR_BGR2HSV);
					cv::inRange(HSV,cv::Scalar(47,50,75),cv::Scalar(58,255,255),threshold);


					cv::dilate(threshold,threshold,dilateElement);
					cv::dilate(threshold,threshold,dilateElement);
					cv::erode(threshold,threshold,erodeElement);
					cv::erode(threshold,threshold,erodeElement);
					cv::dilate(threshold,threshold,dilateElement);
					cv::dilate(threshold,threshold,dilateElement);


					std::vector <cameraData> Object;
					std::vector< std::vector<cv::Point> > contours;
					std::vector<cv::Vec4i> hierarchy;

					threshold.copyTo(temp);

					//find contours of filtered image using openCV findContours function
						findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
						//use moments method to find our filtered object
						//double refArea = 0;
						bool objectFound = false;

						if (hierarchy.size() > 0) {
							int numObjects = hierarchy.size();
							SmartDashboard::PutNumber("Amount of Objects" , numObjects);
							//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
							if(numObjects<MAX_NUM_OBJECTS){
								for (int index = 0; index >= 0; index = hierarchy[index][0]) {

									cv::Moments moment = moments((cv::Mat)contours[index]);
									area = moment.m00;

									//if the area is less than 20 px by 20px then it is probably just noise
									//if the area is the same as the 3/2 of the image size, probably just a bad filter
									//we only want the object with the largest area so we safe a reference area each
									//iteration and compare it to the area in the next iteration.
									if(area>MIN_OBJECT_AREA){

										cameraData object;



										object.setXPos(moment.m10/area);
										object.setYPos(moment.m01/area);

										Object.push_back(object);

										objectFound = true;

									}else objectFound = false;


								}
								//let user know you found an object
								if(objectFound ==true){
									//draw object location on screen
									/*for(int i =0; i<Object.size(); i++){



											circle(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),20,cv::Scalar(0,255,0),2);
										    if(Object.at(i).getYPos()-25>0)
										    line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()-25),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos(),0),cv::Scalar(0,255,0),2);
										    if(Object.at(i).getYPos()+25<cameraFeed_HEIGHT)
										    line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()+25),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos(),cameraFeed_HEIGHT),cv::Scalar(0,255,0),2);
										    if(Object.at(i).getXPos()-25>0)
										    line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos()-25,Object.at(i).getYPos()),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(0,Object.at(i).getYPos()),cv::Scalar(0,255,0),2);
										    if(Object.at(i).getXPos()+25<cameraFeed_WIDTH)
										    line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(Object.at(i).getXPos()+25,Object.at(i).getYPos()),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(Object.at(i).getXPos(),Object.at(i).getYPos()),cv::Point(cameraFeed_WIDTH,Object.at(i).getYPos()),cv::Scalar(0,255,0),2);


							}*/
									if(Object.size() > 1)
									{
										int x = (Object.at(0).getXPos() + Object.at(1).getXPos())/2;
										int y = (Object.at(0).getYPos() + Object.at(1).getYPos())/2;
										SmartDashboard::PutNumber("TargetXPos" ,x);
										SmartDashboard::PutNumber("TargetYPos" ,y);
										targetYPos = y;
										targetXPos = x;



											/*circle(cameraFeed,cv::Point(x,y),20,cv::Scalar(0,255,0),2);
										    if(y-25>0)
										    line(cameraFeed,cv::Point(x,y),cv::Point(x,y-25),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(x,y),cv::Point(x,0),cv::Scalar(0,255,0),2);
										    if(y+25<cameraFeed_HEIGHT)
										    line(cameraFeed,cv::Point(x,y),cv::Point(x,y+25),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(x,y),cv::Point(x,cameraFeed_HEIGHT),cv::Scalar(0,255,0),2);
										    if(x-25>0)
										    line(cameraFeed,cv::Point(x,y),cv::Point(x-25,y),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(x,y),cv::Point(0,y),cv::Scalar(0,255,0),2);
										    if(x+25<cameraFeed_WIDTH)
										    line(cameraFeed,cv::Point(x,y),cv::Point(x+25,y),cv::Scalar(0,255,0),2);
										    else line(cameraFeed,cv::Point(x,y),cv::Point(cameraFeed_WIDTH,y),cv::Scalar(0,255,0),2);
										circle(cameraFeed,cv::Point(x,y),20,cv::Scalar(0,255,0),2);*/

									}
									else if (Object.size() == 1)
								{
									int x = Object.at(0).getXPos();
									int y = Object.at(0).getYPos();
									SmartDashboard::PutNumber("TargetXPos" ,x);
									SmartDashboard::PutNumber("TargetYPos" ,y);
									targetYPos = y;
									targetXPos = x;

								}

						}
					}



				}
						outputStream.PutFrame(threshold);

			}


	int getUltrasonicModeLeft()
	{
		int array[100];
		int mostCommon = 0;
		int testedCommon = 0;
		int mostCommonAmount = 0;
		int testedCommonAmount = 0;


		for( int x = 0; x  <= 99; x = x + 1 )
				   {
				      array[x] = ultrasonicLeft.GetValue()/19.16;
				   }
		  int i = 1;
		  int j = 1;
		  int flag = 1;    // set flag to 1 to start first pass
		      int temp;             // holding variable
		      int numLength = 99;
		      for(i = 1; (i <= numLength) && flag; i++)
		     {
		          flag = 0;
		          for (j=0; j < (numLength -1); j++)
		         {
		               if (array[j+1] > array[j])      // ascending order simply changes to <
		              {
		                    temp = array[j];             // swap elements
		                    array[j] = array[j+1];
		                    array[j+1] = temp;
		                    flag = 1;               // indicates that a swap occurred.
		               }
		          }
		     }

		for( int x = 0; x  <= 99; x = x + 1 )
		   {
		    if(array[x] == testedCommon)
		    {
		    	testedCommonAmount++;
		    	if(testedCommonAmount >= mostCommonAmount)
		    	{
		    		mostCommonAmount = testedCommonAmount;
		    		mostCommon = testedCommon;
		    	}
		    }
		    else
		    {
		    	testedCommonAmount = 1;
		    	testedCommon = array[x];
		    }


		   }
		SmartDashboard::PutNumber("Left Ultrasonic Count" ,mostCommonAmount);
		SmartDashboard::PutNumber("Left Ultrasonic Mode Value" ,mostCommon);
		return mostCommon;
	}



	float squareDrive(float val)
		{
			if(val > 0.05)
			{
				val = val*val;
				return val;
			}
			else if(val < -0.05)
			{
				val = -(val*val);
				return val;
			}
			else
			{
				return 0;
			}
		}

	void shoot()
	{
		basicMove(-48);
		basicTurn(175);
		basicTurn(30);


		shooterMotor.Set(.9);

		shooterInfeedMotor.Set(.5);
		Wait(3);
		shooterMotor.Set(0);
		shooterInfeedMotor.Set(0);
	}
	void placeGear()
	{

		rightGear.Set(180);
		leftGear.Set(0);
		Wait(1);
		leftEncoder.Reset();
		startDistance = leftEncoder.GetDistance();
		currentDistance = leftEncoder.GetDistance();
		targetDistance = -35;
		int count = 0;

		while(targetDistance < currentDistance)
		{
			currentDistance = leftEncoder.GetDistance();
			if(mainJoystick.GetRawButton(1))
											{
										rightDriveWheel_1.Set(0);
										rightDriveWheel_2.Set(0);
										leftDriveWheel_1.Set(0);
										leftDriveWheel_2.Set(0);

												return;
											}
			rightDriveWheel_1.Set(- .25);
			rightDriveWheel_2.Set(- .25);
			leftDriveWheel_1.Set(.25);
			leftDriveWheel_2.Set( .25);


		}

		rightGear.Set(0);
		leftGear.Set(180);

		rightDriveWheel_1.Set(0);
		rightDriveWheel_2.Set(0);
		leftDriveWheel_1.Set(0);
		leftDriveWheel_2.Set(0);



	}



	Robot() :

	mainJoystick(0),
	xboxJoystick(1),

	/*leftDriveWheel_1(3),
	leftDriveWheel_2(2),
	rightDriveWheel_1(1), // Good
	rightDriveWheel_2(0), // Bad
	shooterMotor(4),
	climbMotor(6),
	intakeMotor(5),
	rightGear(7),
	leftGear(8),
	shooterInfeedMotor(9),*/

	leftDriveWheel_1(0),
	leftDriveWheel_2(5),
		rightDriveWheel_1(1), // Good
		rightDriveWheel_2(3), // Bad
		shooterMotor(2),
		climbMotor(4),
		intakeMotor(6),
		targetLight(0),
		rightGear(7),
		leftGear(8),
		shooterInfeedMotor(9),

	ultrasonicLeft(0),
	//ultrasonicRight(1),

	leftEncoder(6,7, false, Encoder::k4X),
	rightEncoder(8,9,false, Encoder::k4X)

	//0 is left Motor
				// 1 is right Motor (?)
				//2 is Shooter Motor
				//3 is Right motor(?)
				//4 is climber
				//5 is left Motor(?)
				//6 is Intake Motor(Ball intake)






	{


		robotLength = 25;
				ultrasonicMax = 180;
				ultrasonicMin = 15;
				targetXPos = 0;
				targetYPos = 0;
				// Get the USB camera from CameraServer
				cameraFront = CameraServer::GetInstance()->StartAutomaticCapture(1);
				cameraFront.SetResolution(640, 480);
				cameraFront.SetFPS(30);

				cameraBack = CameraServer::GetInstance()->StartAutomaticCapture(0);
				cameraBack.SetResolution(640, 480);
				cameraBack.SetFPS(30);

				cvSinkFront = CameraServer::GetInstance()->GetVideo(cameraFront);
				cvSinkBack = CameraServer::GetInstance()->GetVideo(cameraBack);


				//camera2 = CameraServer::GetInstance()->StartAutomaticCapture();
				// Set the resolution


				//camera2.SetResolution(640, 480);

				// Get a CvSink. This will capture Mats from the Camera

				// Setup a CvSource. This will send images back to the Dashboard
				outputStream = CameraServer::GetInstance()->
						PutVideo("Switching", 640, 480);







				//camera2.SetResolution(640, 480);

				hit = false;
				distance = 0;
				area = 0.0;

				driveGyro = new AHRS(SPI::Port::kMXP);




				leftEncoder.SetSamplesToAverage(5);
				leftEncoder.SetDistancePerPulse((1.0 / 360.0 * 2.0 * 3.1415 * 3.0 ));  //22.95
				leftEncoder.SetMinRate(1.0);

				rightEncoder.SetSamplesToAverage(5);
				rightEncoder.SetDistancePerPulse((1.0 / 360.0 * 2.0 * 3.1415 * 1.5));
				rightEncoder.SetMinRate(1.0);

				turns = 1;
				time = 0;
				rightSpeed = 0.0;
				leftSpeed = 0.0;
				targetDistance = 0.0;
				currentDistance = 0.0;
				startDistance = 0.0;
				currentHeading = 90;
				targetHeading = 0;
				distancePerSecond = 0.0;
				howFarOffCourse = 0.0;
				timeOffCourse = 0.0;
				slowStop = false;
				slowStart = false;
				reachedTarget = false;
				areOffCourse = false;
				areObjects = false;

				robotCurrentX = 60;
				robotCurrentY = 1;
				xDistance = 0;

				robotBaseHeading = 90;

				turnLeft = false;
				turnRight = false;

				robotAngle = 0;

				field = 1;
				zone = 1;
				action = 1;

				gearX = 0;
				gearY = 0;

				openGear = false;

				intake = false;
				flipFront = true;

				InnerRange = false;
				OuterRange = false;
				hasntRun = false;




				for (int tempY = 0; tempY <= 323 ; tempY++)
								{
									for (int tempX = 0; tempX <= 323; tempX++)
									{
										Field[tempX][tempY] = FPS(tempX , tempY , .50, 20 , 20);
									}
								}




	}

	void RobotInit() {



	}

	void AutonomousInit() override {



	}

	void AutonomousPeriodic() {

		//Red Field
		if(field == 1)
				{
					if(zone == 1)
					{

							turnRight = true;
							turnLeft = false;


							if(!hasntRun)
							{
							basicMove(48);
							Wait(.1);
							basicTurn(-47);
							Wait(.1);
							setRobotOnCamera();
							Wait(.1);
							placeGear();
							Wait(.1);
							basicTurn(-(driveGyro->GetAngle()));
							//shoot();
							//gearX = 148;
							//gearY = 118;
							hasntRun = true;
							}
					}
					else if(zone == 2)
					{

							turnRight = false;
							turnLeft = false;

							if(!hasntRun)
							{


							setRobotOnCamera();
							Wait(1);
							placeGear();
							hasntRun = true;
							}
							//gearX = 162;
							//gearY = 93;




					}
					else if(zone == 3)
					{
							turnRight = true;
							turnLeft = false;
							if(!hasntRun)
							{
							basicMove(66);
							Wait(.2);
							basicTurn(60);
							Wait(.2);
							setRobotOnCamera();
							Wait(.2);
							placeGear();
							hasntRun = true;


									}
							//gearX = 176;
							//gearY = 118;

					}

				}

		//Field 2
				else if(field == 2)
				{
					if(zone == 1)
					{
						if(!hasntRun)
										{
										basicMove(48);
										Wait(.1);
										basicTurn(47);
										Wait(.1);
										setRobotOnCamera();
										Wait(.1);
										placeGear();
										Wait(.1);
										basicTurn(-(driveGyro->GetAngle()));
										//shoot();
										//gearX = 148;
										//gearY = 118;
										hasntRun = true;
										}


					}
					else if(zone == 2)
					{
						if(!hasntRun)
											{


											setRobotOnCamera();
											Wait(1);
											placeGear();
											hasntRun = true;
											}

					}
					else if(zone == 3)
					{
						if(!hasntRun)
											{
											basicMove(66);
											Wait(.2);
											basicTurn(-60);
											Wait(.2);
											setRobotOnCamera();
											Wait(.2);
											placeGear();
											hasntRun = true;


													}
					}


				}
				else
				{
					basicMove(100);

				}

	}

	void TeleopInit() {
		if(!openGear)
		{
			openGear = false;

			rightGear.Set(0);
			leftGear.Set(180);
		}

	}

	void TeleopPeriodic() {
		//VisionThread();


		SmartDashboard::PutNumber("GyroAngleCurrent" , driveGyro->GetAngle());
		SmartDashboard::PutNumber("LeftEncoder" , leftEncoder.GetDistance());
		SmartDashboard::PutNumber("RightEncoder" , rightEncoder.GetDistance());
		SmartDashboard::PutNumber("UltraSonic Data" , getUltrasonicModeLeft() );
		if(mainJoystick.GetRawButton(11) || mainJoystick.GetRawButton(6))
		{
			flipFront = true;
		}
		else if(mainJoystick.GetRawButton(10) || mainJoystick.GetRawButton(7))
		{
			flipFront = false;
		}
		if(!flipFront)
		{
		yIn = (-mainJoystick.GetRawAxis(1));
		xIn = (mainJoystick.GetRawAxis(0));
		cvSinkFront.SetEnabled(false);
				cvSinkBack.SetEnabled(true);
				cvSinkBack.GrabFrame(cameraFeed);
		}
		else
		{
			yIn = (mainJoystick.GetRawAxis(1));
			xIn = (mainJoystick.GetRawAxis(0));
			cvSinkBack.SetEnabled(false);
						cvSinkFront.SetEnabled(true);
						cvSinkFront.GrabFrame(cameraFeed);
		}

		outputStream.PutFrame(cameraFeed);



				yIn = yIn/1.1;
				xIn = xIn/1.5;

				if(mainJoystick.GetRawButton(1))
				{
					yIn = yIn/1.5;
				}





		rightDriveWheel_1.Set(-(squareDrive(yIn) - squareDrive(xIn)));
		rightDriveWheel_2.Set(-(squareDrive(yIn) - squareDrive(xIn)));
		leftDriveWheel_1.Set((squareDrive(yIn) + squareDrive(xIn)));
		leftDriveWheel_2.Set((squareDrive(yIn) + squareDrive(xIn)));

		double test = ultrasonicLeft.GetValue();
		SmartDashboard::PutNumber("Ultrasonic range converted left", test/19.16);
		SmartDashboard::PutNumber("Ultrasonic range raw left", test);


		if(xboxJoystick.GetRawButton(6))
		{
			shooterMotor.Set(.85);

		}
		else if (xboxJoystick.GetRawButton(5))
		{
			shooterMotor.Set(0);

		}
		if(xboxJoystick.GetRawButton(1))
		{
			intakeMotor.Set(-.75);
		}
		else if(xboxJoystick.GetRawButton(2))
		{
			intakeMotor.Set(0);
		}

		else if(xboxJoystick.GetRawButton(4))
		{
			intakeMotor.Set(.5);
		}
		if(xboxJoystick.GetRawButton(3))
		{
			climbMotor.Set(1);
		}
		else
		{
			climbMotor.Set(0);

		if(xboxJoystick.GetRawButton(7))
					{
						rightGear.Set(180);
						leftGear.Set(0);
					}
		if(xboxJoystick.GetRawButton(8))
		{


			rightGear.Set(0);
			leftGear.Set(180);

		}
		if(xboxJoystick.GetRawAxis(3) > .5)
		{
			shooterInfeedMotor.Set(.7);
		}
		else
		{

			shooterInfeedMotor.Set(-.2);
		}

		if(mainJoystick.GetRawButton(8))
		{
			//targetLight.Set(true);

			currentHeading = driveGyro->GetAngle();
			xDistance = getUltrasonicModeLeft() - 20;
					if((currentHeading/360) > 1 || (currentHeading/360) < -1)
					{
						currentHeading = abs(currentHeading) % 360;
					}
					if(currentHeading > 0)
					{
						currentHeading = 45;
					}
					else
					{
						currentHeading = -45;
					}


						advancedMovement(xDistance , currentHeading);

		}
		else if(mainJoystick.GetRawButton(9))
		{
			//targetLight.Set(false);
			setRobotOnCamera();
		}









	}







	}

	void TestPeriodic() {



	}


};

START_ROBOT_CLASS(Robot)


