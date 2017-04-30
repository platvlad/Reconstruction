#include <iostream>
//#include "stdafx.h"
//#include "ppl.h"
//#include "Kinect.h"
//#include "NuiKinectFusionApi.h"

#include "KinectSensor.h"
//#include "DepthStream.h"

using namespace std;



int main()
{
	Kinect sensor;
	if (SUCCEEDED(sensor.Initialize()))
	{
		sensor.Run();
	}
	else
	{
		cout << "Initialization failed" << endl;
	}
}