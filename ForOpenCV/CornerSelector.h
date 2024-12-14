#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <fstream>
#include <map>
#include <filesystem>

#define WIN_NAME "Corner Selector"

class CornerSelector
{
public:
	static std::vector<cv::Mat> loadFrames(std::string folderName);

	void startCornerSelectorWindow(std::vector<cv::Mat> frames);

private:
	int WIDTH;
	int HEIGHT;
	cv::Mat windowImage;
	int currentFrame=0;
	int totalFrames;
	std::vector<cv::Mat> corners;
	int currentCorner = 0;
	
	bool handleInput(int key);
	void cornerSelectorMainLoop(std::vector<cv::Mat> frames);

	void onMouse(int event, int x, int y, int flags);
	static void onMouse(int event, int x, int y, int flags, void* userdata);

	void redraw() {
		cv::Mat displayImage;
		cv::resize(windowImage, displayImage, cv::Size((int)((double)windowImage.cols / windowImage.rows * 600.0), 600));
		if (cv::countNonZero(corners[currentFrame]) != 0) {
			cv::Mat outImage; displayImage.copyTo(outImage);
			drawPoints(corners[currentFrame], outImage);
			displayImage = outImage;
		}
		cv::imshow(WIN_NAME, displayImage);
	}

	void drawPoints(cv::Mat points, cv::Mat& outImage) {
		for (int i = 0; i < points.rows; i++) {
			cv::circle(outImage, 
				cv::Point2i((int)points.at<double>(i, 0), (int)points.at<double>(i, 1)), 
				4, cv::Scalar(0, 0, 255), -1);
		}
	}

	void saveSelectedCoordinatesIntoFile(std::string filename) {
		/*Save these coordinates into a text file (2 points). */
		std::stringstream strstr;
		for (int i = 0; i < corners.size(); i++) {
			cv::Mat c = corners[i];
			strstr << c.at<double>(0, 0) << " " << c.at<double>(0, 1) << ";" <<
					  c.at<double>(1, 0) << " " << c.at<double>(1, 1) << ";" <<
					  c.at<double>(2, 0) << " " << c.at<double>(2, 1) << ";" <<
					  c.at<double>(3, 0) << " " <<  c.at<double>(3, 1) << "\n";
		}
		std::ofstream outFile;
		outFile.open(filename);
		outFile << strstr.rdbuf();
	}
};

