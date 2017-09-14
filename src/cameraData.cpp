class cameraData {
	int xPos;
	int yPos;
public :
	cameraData()
	{
		xPos = 0;
		yPos = 0;
	}
	cameraData(int x,int y)
		{
			xPos = x;
			yPos = y;
		}

	void setXPos(int X)
	{
		xPos = X;
	}
	void setYPos(int X)
	{
		yPos = X;

	}

	int getXPos()
		{
			return xPos;
		}
		int getYPos()
		{
			return yPos;
		}
};
