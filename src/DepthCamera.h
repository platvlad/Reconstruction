#include "Kinect.h"
#include "NuiKinectFusionApi.h"

class DepthCamera
{
private:
	NUI_FUSION_CAMERA_PARAMETERS m_cameraParameters;
	UINT* m_pDepthDistortionLT;
	DepthSpacePoint* m_pDepthDistortionMap;
	ICoordinateMapper* m_pMapper;

	HRESULT SetupUndistortion();
	
public:
	DepthCamera(ICoordinateMapper* mapper);
	~DepthCamera();

	void getDepthDistortionLT(UINT* distortionLT);
};