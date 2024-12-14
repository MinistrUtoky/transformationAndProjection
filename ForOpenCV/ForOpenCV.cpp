#include "CornerSelector.h"
#include "CloudImprinter.h"

void startCornerSelector() {
	std::vector<cv::Mat> images = CornerSelector::loadFrames("sample");
	CornerSelector cs;
	cs.startCornerSelectorWindow(images);
}

void startCloudImprinter() {
	std::vector<cv::Mat> corners, images, clouds;
	CloudImprinter::loadCornersImagesClouds("cornerCoordinates.txt", "sample", "sample_clouds", 
											corners, images, clouds);
	CloudImprinter ci;
	images = ci.mapImagesIntoPoster(images, corners);
	images = ci.projectCloudsIntoImages(images, clouds);
	ci.makeVideoAndSave(images, "p2ptNpcp.avi");
}

int main()
{
	//startCornerSelector();
	startCloudImprinter();
    return 0;
}
