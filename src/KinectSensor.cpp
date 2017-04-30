#include <iostream>

#include "NuiKinectFusionApi.h"

#include "KinectSensor.h"

using namespace std;

Kinect::Kinect():
	m_pNuiSensor(NULL),
	m_pDepthFrameReader(NULL),
	m_pColorFrameReader(NULL),
	m_pDepthRawPixelBuffer(NULL),
	m_pDepthUndistortedPixelBuffer(NULL),
	depthStream(NULL),
	depthCamera(NULL),
	depthDistortionLT(NULL)
{

}

Kinect:: ~Kinect()
{
	//here must be code: if (nullptr != m_pMapper) m_pMapper->UnsubscribeCoordinateMappingChanged(m_coordinateMappingChangedEvent);

    SafeRelease(m_pMapper);

	if (m_pNuiSensor != nullptr)
    {
        m_pNuiSensor->Close();
        SafeRelease(m_pNuiSensor);
    }

	SafeRelease(m_pDepthFrameReader);
    SafeRelease(m_pColorFrameReader);

	if (m_pDepthRawPixelBuffer)
	{
		delete [] m_pDepthRawPixelBuffer;
	}

	if (m_pDepthUndistortedPixelBuffer)
	{
		delete [] m_pDepthUndistortedPixelBuffer;
	}

	if (depthStream)
	{
		delete depthStream;
	}

	if (depthCamera)
	{
		delete depthCamera;
	}

	if (depthDistortionLT)
	{
		delete [] depthDistortionLT;
	}

}

HRESULT Kinect::Initialize()
{
	HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pNuiSensor);

    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pNuiSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;
        IColorFrameSource* pColorFrameSource = NULL;

        hr = m_pNuiSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pNuiSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
        }

        if (SUCCEEDED(hr))
        {
            hr = m_pNuiSensor->get_ColorFrameSource(&pColorFrameSource);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
        }

		if (SUCCEEDED(hr))
        {
            hr = m_pNuiSensor->get_CoordinateMapper(&m_pMapper);
        }

		SafeRelease(pDepthFrameSource);
        SafeRelease(pColorFrameSource);

		m_pDepthRawPixelBuffer = new UINT16[NUI_DEPTH_RAW_HEIGHT * NUI_DEPTH_RAW_WIDTH];
		depthCamera = new DepthCamera(m_pMapper);
		depthStream = new DepthStream(NUI_DEPTH_RAW_HEIGHT, NUI_DEPTH_RAW_WIDTH, 10);

		depthDistortionLT = new UINT[NUI_DEPTH_RAW_HEIGHT * NUI_DEPTH_RAW_WIDTH];
		depthCamera -> getDepthDistortionLT(depthDistortionLT);
	}
	return hr;
}

void Kinect::Run()
{
	while (true)
	{
		HRESULT hr;
		IDepthFrame* pDepthFrame = NULL;
		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if (FAILED(hr))
		{
			continue;
		}
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

		const UINT bufferLength =  NUI_DEPTH_RAW_HEIGHT * NUI_DEPTH_RAW_WIDTH;
		UINT16 * pRawDepth = m_pDepthRawPixelBuffer;
		UINT16 * pDepth = m_pDepthUndistortedPixelBuffer;

		for(UINT i = 0; i < bufferLength; i++, pRawDepth++, pDepth++)
		{
            *pRawDepth = pBuffer[i];
			//const UINT id = depthDistortionLT[i];
			//*pDepth = id < bufferLength? pBuffer[id] : 0;
        }

		depthStream->pushFrame(pBuffer);
		SafeRelease(pDepthFrame);
	}
}

void Kinect::CreateMesh()
{

}