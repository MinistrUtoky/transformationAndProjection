#include "CornerSelector.h"


void CornerSelector::onMouse(int event, int x, int y, int flags) {
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		corners[currentFrame].at<double>(currentCorner, 0) = x;
		corners[currentFrame].at<double>(currentCorner, 1) = y;
		std::cout << corners[currentFrame] << std::endl;
		currentCorner = (currentCorner + 1) % 4;
	}
	if (event == cv::EVENT_RBUTTONDOWN) {
		if (x > windowImage.cols / 4) {
			currentCorner = 0;
			currentFrame = (currentFrame + 1) % totalFrames;
		}
		else {
			currentCorner = 0;
			currentFrame = (currentFrame + totalFrames - 1) % totalFrames;
		}
	}
	if (event == cv::EVENT_MOUSEWHEEL) {
		float delta = cv::getMouseWheelDelta(flags);
		if (delta > 0) {
			currentCorner = 0;
			currentFrame = (currentFrame + 1) % totalFrames;
		}
		else {
			currentCorner = 0;
			currentFrame = (currentFrame + totalFrames - 1) % totalFrames;
		}
	} 
}

void CornerSelector::onMouse(int event, int x, int y, int flags, void* userdata) {
	CornerSelector* cs = reinterpret_cast<CornerSelector*>(userdata);
	cs->onMouse(event, x, y, flags);
}

std::vector<cv::Mat> CornerSelector::loadFrames(std::string folderName) {
	/*Select 20 consecutive frames of a camera on which a billboard (poster) is visible. (2 points)
	(Only one camera’s stream is required, other camera frames can be deleted.)*/
	std::string path = "T:\\task2\\" + folderName + "\\*.jpg";

	std::vector<std::string> imageNames;
	cv::glob(path, imageNames, false);

	std::vector<cv::Mat> images;
	for (int i = 0; i < imageNames.size() - 1; i++) {
		cv::Mat img = cv::imread(imageNames.at(i));
		images.push_back(img);
	}
	return images;
}


void CornerSelector::startCornerSelectorWindow(std::vector<cv::Mat> frames) {
	/*Write a GUI (graphical user interface) that loads the images one by one,
	and the user can select the four corner points of the poster manually by mouse clicking. (6 points). */
	currentFrame = 0;
	currentCorner = 0;
	totalFrames = frames.size();
	for (int i = 0; i < frames.size(); i++) {
		cv::Mat ps = cv::Mat::zeros(4, 2, CV_64F);
		corners.push_back(ps);
	}
	windowImage = frames[currentFrame];
	WIDTH = windowImage.cols;
	HEIGHT = windowImage.rows;

	redraw();

	cv::setMouseCallback(WIN_NAME, onMouse, this);
	cornerSelectorMainLoop(frames);
}

bool CornerSelector::handleInput(int key) {
	if (key == 27) return false;
	if (key == 32) {
		saveSelectedCoordinatesIntoFile("cornerCoordinates.txt");
		return false;
	}
	/*
	switch (key) {
	case 'a':
		currentCorner = 0;
		currentFrame = (currentFrame + totalFrames - 1) % totalFrames;
		break;
	case 'd':
		currentCorner = 0;
		currentFrame = (currentFrame + 1) % totalFrames;
		break;
	}*/
	return true;
}
void CornerSelector::cornerSelectorMainLoop(std::vector<cv::Mat> frames) {
	// not to forget that input person may be a monkey (make all four point selection mandatory, show points with circles, 
	// show reminder and error boards, restart if not all selected, not letting into the main part, make toggle for selection skip)
	while (true) {
		int key = cv::pollKey();
		if (!handleInput(key)) return;
		windowImage = frames[currentFrame];
		redraw();
	}
}