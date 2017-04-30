#include <iostream>

#include "DepthCamera.h"

using namespace std;

DepthCamera :: DepthCamera(ICoordinateMapper* mapper) : 
	m_pMapper(mapper)
{
	m_cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
    m_cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
    m_cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
    m_cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
	const UINT width = NUI_DEPTH_RAW_WIDTH;
    const UINT height = NUI_DEPTH_RAW_HEIGHT;
	const UINT depthBufferSize = width * height;
	m_pDepthDistortionLT = new UINT[depthBufferSize];
	m_pDepthDistortionMap = new DepthSpacePoint[depthBufferSize];
	SetupUndistortion();
}

HRESULT DepthCamera :: SetupUndistortion()
{
	 HRESULT hr = E_UNEXPECTED;

    if (m_cameraParameters.principalPointX != 0)
    {

        const UINT width = NUI_DEPTH_RAW_WIDTH;
        const UINT height = NUI_DEPTH_RAW_HEIGHT;
        const UINT depthBufferSize = width * height;

        CameraSpacePoint cameraFrameCorners[4] = //at 1 meter distance. Take into account that depth frame is mirrored
        {
            /*LT*/ { -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f }, 
            /*RT*/ { (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, m_cameraParameters.principalPointY / m_cameraParameters.focalLengthY, 1.f }, 
            /*LB*/ { -m_cameraParameters.principalPointX / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }, 
            /*RB*/ { (1.f - m_cameraParameters.principalPointX) / m_cameraParameters.focalLengthX, (m_cameraParameters.principalPointY - 1.f) / m_cameraParameters.focalLengthY, 1.f }
        };

        for(UINT rowID = 0; rowID < height; rowID++)
        {
            const float rowFactor = float(rowID) / float(height - 1);
            const CameraSpacePoint rowStart = 
            {
                cameraFrameCorners[0].X + (cameraFrameCorners[2].X - cameraFrameCorners[0].X) * rowFactor,
                cameraFrameCorners[0].Y + (cameraFrameCorners[2].Y - cameraFrameCorners[0].Y) * rowFactor,
                1.f
            };

            const CameraSpacePoint rowEnd = 
            {
                cameraFrameCorners[1].X + (cameraFrameCorners[3].X - cameraFrameCorners[1].X) * rowFactor,
                cameraFrameCorners[1].Y + (cameraFrameCorners[3].Y - cameraFrameCorners[1].Y) * rowFactor,
                1.f
            };

            const float stepFactor = 1.f / float(width - 1);
            const CameraSpacePoint rowDelta = 
            {
                (rowEnd.X - rowStart.X) * stepFactor,
                (rowEnd.Y - rowStart.Y) * stepFactor,
                0
            };

            CameraSpacePoint cameraCoordsRow[NUI_DEPTH_RAW_WIDTH];

            CameraSpacePoint currentPoint = rowStart;
            for(UINT i = 0; i < width; i++)
            {
                cameraCoordsRow[i] = currentPoint;
                currentPoint.X += rowDelta.X;
                currentPoint.Y += rowDelta.Y;
            }

            hr = m_pMapper->MapCameraPointsToDepthSpace(width, cameraCoordsRow, width, &m_pDepthDistortionMap[rowID * width]);
            if(FAILED(hr))
            {
                return hr;
            }
        }

        if (nullptr == m_pDepthDistortionLT)
        {
            return E_OUTOFMEMORY;
        }
		if (m_pDepthDistortionLT)
		{
			cout << "m_pDepthDistortionLT in SetupUndistortion method is not null" << endl;
		}
		else
		{
			cout << "m_pDepthDistortion in SetupUndistortion method is null" << endl;
		}

        UINT* pLT = m_pDepthDistortionLT;
        for(UINT i = 0; i < depthBufferSize; i++, pLT++)
        {
            //nearest neighbor depth lookup table 
            UINT x = UINT(m_pDepthDistortionMap[i].X + 0.5f);
            UINT y = UINT(m_pDepthDistortionMap[i].Y + 0.5f);

            *pLT = (x < width && y < height)? x + y * width : UINT_MAX; 
        } 
    }
    return S_OK;
}

DepthCamera :: ~DepthCamera()
{
	delete [] m_pDepthDistortionMap;
	delete [] m_pDepthDistortionLT;
}

void DepthCamera :: getDepthDistortionLT(UINT* distortionLT)
{
	cout << 15.1 << endl;
	const UINT width = NUI_DEPTH_RAW_WIDTH;
    const UINT height = NUI_DEPTH_RAW_HEIGHT;
    const UINT depthBufferSize = width * height;
	UINT* depthDistortion = m_pDepthDistortionLT;
	
	for (int i = 0; i < depthBufferSize; ++i, ++depthDistortion)
	{
		distortionLT[i] = *depthDistortion;
	}
}