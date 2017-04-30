#include "stdafx.h"
#include <opencv2/opencv.hpp>

class DepthStream
{
private:
	int m_height;
	int m_width;
	int m_size;
	int m_capacity;
	int m_counter;
	int m_bigCounter;
	UINT16** m_pBuffer;
	void ToJPG(UINT16* frame);
	void ToFile(UINT16* frame);
	void Sobel(cv::Mat &frame);
public:
	DepthStream(int height = 512, int width = 424, int capacity = 10);
	void pushFrame(UINT16* depthFrame);
	UINT16* getAverageFrame();
	~DepthStream();
};