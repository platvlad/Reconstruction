#include <iostream>
#include <fstream>

#include <string>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv/highgui.h"

#include "DepthStream.h"

using namespace std;

DepthStream :: DepthStream(int height, int width, int capacity):
	m_height(height),
	m_width(width),
	m_size(0),
	m_capacity(capacity),
	m_counter(0),
	m_bigCounter(0)
{
	m_pBuffer = new UINT16*[m_capacity];
}

DepthStream :: ~DepthStream()
{
	for (int i = 0; i < m_size; ++i)
	{
		delete [] m_pBuffer[i];
	}
	delete [] m_pBuffer;
}

void DepthStream :: pushFrame(UINT16* depthFrame)
{
	UINT16* frame = depthFrame;
	int bufSize = m_height * m_width;
	if (m_size < m_capacity)
	{
		m_pBuffer[m_counter] = new UINT16[bufSize];
	}

	for (int i = 0; i < bufSize; ++i, ++frame)
	{
		m_pBuffer[m_counter][i] = *frame;
	}
	
	ToJPG(m_pBuffer[m_counter]);
	m_counter = (m_counter + 1) % m_capacity;
	m_size = min(m_size + 1, m_capacity);
	++m_bigCounter;
}

UINT16* DepthStream :: getAverageFrame()
{
	int frameSize = m_height * m_width;
	UINT16* averageFrame = new UINT16[frameSize];
	for (int i = 0; i < frameSize; ++i)
	{
		averageFrame[i] = 0;
		for (int j = 0; j < m_size; ++j)
		{
			averageFrame[i] += m_pBuffer[j][i];
		}
		averageFrame[i] /= m_capacity;
	}
	return averageFrame;
}

void DepthStream:: ToJPG(UINT16* frame)
{
	cv::Mat mat(m_height, m_width, CV_8UC1);
	for (int i = 0; i < m_height; ++i)
	{
		for (int j = 0; j < m_width; ++j)
		{
			mat.at<uchar>(i, j) = min(max(frame[i * m_width + j] - 500, 0) * 0.51, 255); //black if < 0.5 m and white if > 1 m
		}
	}
	
	std::string str = std::to_string(m_bigCounter);
	cv::imwrite("Frames/" + str + ".jpg", mat);
	Sobel(mat);
}

void DepthStream :: ToFile(UINT16* frame)
{
	string str = std::to_string(m_bigCounter);
	ofstream fout;
	fout.open("Text frames/" + str + ".txt");
	for (int i = 0; i < m_height; ++i)
	{
		for (int j = 0; j < m_width; ++j)
		{
			fout << frame[i * m_height + j] << ' ';
		}
		fout << endl;
	}
	fout.close();
}

void DepthStream :: Sobel(cv::Mat &frame)
{
	int ddepth = CV_16S;

	cv::Mat SobelX(m_height, m_width, CV_8UC1);
	cv::Mat SobelY(m_height, m_width, CV_8UC1);

	cv::Sobel(frame, SobelX, ddepth, 1, 0);
	cv::Sobel(frame, SobelY, ddepth, 0, 1);
	cv::Mat SobelSum = SobelX + SobelY;

	std::string str = std::to_string(m_bigCounter);
	
	cv::imwrite("Frames/SobelX/" + str + ".jpg", SobelX);
	cv::imwrite("Frames/SobelY/" + str + ".jpg", SobelY);
	cv::imwrite("Frames/SobelSum/" + str + ".jpg", SobelSum);

	std::cout << m_bigCounter << std::endl;
}