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


class CloudImprinter
{
public:
    static void loadCornersImagesClouds(std::string cornerFile, std::string imagesFolder, std::string cloudsFolder,
        std::vector<cv::Mat>& corners, std::vector<cv::Mat>& images, std::vector<cv::Mat>& clouds);

	std::vector<cv::Mat> mapImagesIntoPoster(std::vector<cv::Mat> images, std::vector<cv::Mat> corners);

	std::vector<cv::Mat> projectCloudsIntoImages(std::vector<cv::Mat> mappedImages, std::vector<cv::Mat> clouds) {
		/*Repeat the drawing for all the frames. (2 points) */
		for (int i = 0; i < mappedImages.size(); i++) 
			projectCloudIntoImage(mappedImages[i], clouds[i]);
        return mappedImages;
	}

	void makeVideoAndSave(std::vector<cv::Mat> finalImages, std::string fileName) {
		/*Make a video from the 20 resulting pictures using OpenCV’s method(s) and save it (5 points). */

		cv::VideoWriter video(fileName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 4, cv::Size(finalImages[0].cols, finalImages[0].rows), true);
		for (int i = 0; i < finalImages.size(); i++) {
			cv::Mat frame = finalImages[i];
			video.write(frame);
			char c = (char)cv::waitKey(0);
			if (c == 27) break;
			cv::imshow("Frame", frame);
		}
	}
private:
	static bool compare(std::pair<int, int>& a, std::pair<int, int>& b);
	static std::vector<std::pair<int, int>> sort(std::map<int, int>& mp);

    cv::Mat mapImageIntoPoster(cv::Mat image, cv::Mat cornerPoints);

	cv::Mat projectCloudIntoImage(cv::Mat mappedImage, cv::Mat cloud) { // make the parameters here as they are already uploaded
		/*Read the document of the applied sensor kit, 
		take the calibration parameters and project the point cloud into the images. 
		Draw them by filled blue circles with radius 3 pixels (4 points) 
		Check the visibility of the points, draw only the 3D LiDAR points 
		that are in front of the camera plane. (2 points) */

		/* We know that we work with DEV0 and only DEV0, so we don't really need to parse the CalibrationParameters file
		DEV0
		Radial distortion: [0.022535963351356,-0.021289955281333]
		K: [1.296017692307357e+03,0,9.407268031732034e+02;
			0,1.294832210476451e+03,5.837191315595016e+02;
			0,0,1]
		FocalLength: [1.296017692307357e+03,1.294832210476451e+03]
		PrincipalPoint: [9.407268031732034e+02,5.837191315595016e+02]

		DEV0->LiDAR
		R: [0.951099839323371,0.023671034661576,0.307975287575331;
			-0.308201037834387,0.006476788999821,0.951299201871871;
			0.020523545426254,-0.999698821306849,0.013455510426372]
		t: [-0.002222602868500,-0.079224643637628,-0.115061940654467]
		*/

     	double kdata[9]{ 1.296017692307357e+03,0,9.407268031732034e+02,
						0,1.294832210476451e+03,5.837191315595016e+02,
						0,0,1 };
		double rdata[9]{ 0.951099839323371,0.023671034661576,0.307975287575331,
						-0.308201037834387,0.006476788999821,0.951299201871871,
						 0.020523545426254,-0.999698821306849,0.013455510426372 };
		double tdata[3]{ -0.002222602868500,-0.079224643637628,-0.115061940654467 };
		cv::Mat K(3, 3, CV_64F, kdata);
		cv::Mat R(3, 3, CV_64F, rdata);
		cv::Mat T(3, 1, CV_64F, tdata);

		cv::Mat lookDir = R * T;

		cv::Mat angles = cloud * lookDir; // a x b = |a||b|cos(a,b)

		std::vector<cv::Point2f> projectedPoints; std::vector<cv::Point3f> pp;
		for (int i = 0; i < cloud.rows; i++) {
			if (angles.at<double>(i,0) <    0) {
				pp.push_back(cv::Point3f(cloud.at<double>(i, 0), cloud.at<double>(i, 1), cloud.at<double>(i, 2)));
			}
		}

		cv::Mat R_vector(3, 1, CV_64F);
		cv::Rodrigues(R, R_vector);
		double data[4]{ 0,0,0,0 };
		cv::Mat dstr = cv::Mat(4, 1, CV_64F, data);
		cv::projectPoints(pp, R.inv(), -T, K, dstr, projectedPoints);
		for (int i = 0; i < projectedPoints.size(); i++)
			cv::circle(mappedImage, cv::Point((int)projectedPoints.at(i).x, (int)projectedPoints.at(i).y), 6, cv::Scalar(255, 0, 0), cv::FILLED);

		//cv::resize(mappedImage, mappedImage, cv::Size((int)((double)mappedImage.cols / mappedImage.rows * 600.0), 600));
		//cv::imshow("Imprinter", mappedImage);
		//cv::waitKey(0);

		return mappedImage;
	}
};

