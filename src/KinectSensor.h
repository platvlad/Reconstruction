#include "Kinect.h"

#include "DepthStream.h"
#include "DepthCamera.h"

class Kinect
{
public:
	Kinect();
	~Kinect();
	HRESULT Initialize();
	void Run();
	void CreateMesh();
private:
	IKinectSensor* m_pNuiSensor;
	ICoordinateMapper* m_pMapper;
	IDepthFrameReader* m_pDepthFrameReader;
    IColorFrameReader* m_pColorFrameReader;
	UINT16* m_pDepthRawPixelBuffer;
	UINT16* m_pDepthUndistortedPixelBuffer;
	DepthStream* depthStream;
	DepthCamera* depthCamera;
	UINT * depthDistortionLT;
	
};