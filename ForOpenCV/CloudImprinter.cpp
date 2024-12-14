#include "CloudImprinter.h"

void CloudImprinter::loadCornersImagesClouds(std::string cornerFile, std::string imagesFolder, std::string cloudsFolder,
    std::vector<cv::Mat>& corners, std::vector<cv::Mat>& images, std::vector<cv::Mat>& clouds) {
    /*Write another application which loads the text file with the corners
    and loads the corresponding images and point clouds. (2 points) */
    std::ifstream file;
    file.open(cornerFile);
    while (!file.eof()) {
        cv::Mat points(4, 2, CV_64F);
        std::string ss;
        std::getline(file, ss);
        if (ss.size() != 0) {
            for (int i = 0; i < 4; i++) {
                std::string s = ss.substr(0, ss.find(";"));
                points.at<double>(i, 0) = std::stoi(s.substr(0, s.find(" ")));
                s.erase(0, s.find(" ") + 1);
                points.at<double>(i, 1) = std::stoi(s);
                ss.erase(0, ss.find(";") + 1);
            }
        }
        corners.push_back(points);
    }
    file.close();

    std::vector<std::string> imageNames;
    cv::glob("T:\\task2\\" + imagesFolder + "\\*.jpg", imageNames, false);
    for (int i = 0; i < imageNames.size() - 1; i++) {
        images.push_back(cv::imread(imageNames.at(i)));
    }

    std::vector<std::string> cloudNames;
    cv::glob("T:\\task2\\" + cloudsFolder + "\\*.xyz", cloudNames, false);
    // polar coords https://www.hackademix.hu/wp-content/uploads/2023/06/Sensor_pack_summary_2023.pdf
    // + rgb colors (don't matter)
    for (int i = 0; i < cloudNames.size() - 1; i++) {
        std::ifstream cloudFile;
        cloudFile.open(cloudNames[i]);
        std::vector<cv::Point3d> cloud;
        while (!cloudFile.eof()) {
            double RCosWSinA, RCosWCosA, RSinW; int R, G, B;
            cloudFile >> RCosWSinA; cloudFile >> RCosWCosA; cloudFile >> RSinW;
            cloudFile >> R; cloudFile >> G; cloudFile >> B;
            if (RCosWSinA != 0 || RCosWCosA != 0 || RSinW != 0)
                cloud.push_back(cv::Point3d(RCosWSinA, RCosWCosA, RSinW));
        }

        cv::Mat cloudMat(cloud.size(), 3, CV_64F);
        for (int j = 0; j < cloud.size(); j++) {
            cloudMat.at<double>(j, 0) = cloud.at(j).x;
            cloudMat.at<double>(j, 1) = cloud.at(j).y;
            cloudMat.at<double>(j, 2) = cloud.at(j).z;
        }
        clouds.push_back(cloudMat);
    }
}


std::vector<cv::Mat> CloudImprinter::mapImagesIntoPoster(std::vector<cv::Mat> images, std::vector<cv::Mat> corners) {
    /*Compute the homographies for each frame that map the whole image of the current frame into the poster.
    You can use both OpenCV’s built-in method or the code from the 3d vision class. (5 points)*/
    std::vector<cv::Mat> mappedImages;
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat mappedImage;
        //cv::resize(images[i], mappedImage, cv::Size((int)((double)images[i].cols / images[i].rows * 600.0), 600));
        //mappedImage = mapImageIntoPoster(mappedImage, corners[i]);
        mappedImage = mapImageIntoPoster(images[i], corners[i]);
        mappedImages.push_back(mappedImage);
    }
    return mappedImages;
}


bool CloudImprinter::compare(std::pair<int, int>& a, std::pair<int, int>& b) { return a.second < b.second; }
std::vector<std::pair<int, int>> CloudImprinter::sort(std::map<int, int>& mp)
{
    std::vector<std::pair<int, int>> pairs;
    for (auto& it : mp) pairs.push_back(it);
    std::sort(pairs.begin(), pairs.end(), compare);
    return pairs;
}

cv::Mat CloudImprinter::mapImageIntoPoster(cv::Mat image, cv::Mat cornerPoints) {
    /*Compute the homographies for each frame that map the whole image of the current frame into the poster.
    You can use both OpenCV’s built-in method or the code from the 3d vision class. (5 points)  */
    // sort out corner points
    

    std::vector<cv::Point2d> points;
    std::map<int, int> indexOrdering;//from left upper corner clockwise
    double uLCornerSqDistance = 1e63, lRCornerSqDistance = 1e63, uRCornerSqDistance = 1e63;
    for (int i = 0; i < 4; i++) {
        // I saved corners for a halved size image but the clouds are for full sized images so making corners back full sized is faster
        cv::Point2d p = cv::Point2d(cornerPoints.at<double>(i, 0)*2, cornerPoints.at<double>(i, 1)*2);
        double ulsqdistance = p.x * p.x + p.y * p.y,
            lrsqdistance = (image.cols - p.x) * (image.cols - p.x) + (image.rows - p.y) * (image.rows - p.y),
            ursqdistance = (image.cols - p.x) * (image.cols - p.x) + p.y * p.y;
        if (ulsqdistance < uLCornerSqDistance) {
            indexOrdering[0] = i;
            uLCornerSqDistance = ulsqdistance;
        }
        if (ursqdistance < uRCornerSqDistance) {
            indexOrdering[1] = i;
            uRCornerSqDistance = ursqdistance;
        }
        if (lrsqdistance < lRCornerSqDistance) {
            indexOrdering[2] = i;
            lRCornerSqDistance = lrsqdistance;
        }
        points.push_back(p);
    }

    std::vector<cv::Point2d> imageCorners{
        cv::Point2d(0,0),
        cv::Point2d(image.cols, 0),
        cv::Point2d(image.cols, image.rows),
        cv::Point2d(0, image.rows)
    };

    // since sum of 0+1+2+3=6 we can find the last index by o[3]=6-(o[0]+o[1]+o[2])
    indexOrdering[3] = 6 - indexOrdering[0] - indexOrdering[1] - indexOrdering[2];

    std::vector<cv::Point2d> posterCorners{
        points[indexOrdering[0]],
        points[indexOrdering[1]],
        points[indexOrdering[2]],
        points[indexOrdering[3]],
    };

    cv::Mat H = cv::findHomography(imageCorners, posterCorners);

    cv::Mat stitched_img;
    cv::Mat img_transformed;
    cv::warpPerspective(image, img_transformed, H, cv::Size(image.cols, image.rows));
    stitched_img = image.clone();
    cv::Mat mask;
    cv::inRange(img_transformed, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0), mask);
    img_transformed.copyTo(stitched_img(cv::Rect(0, 0, img_transformed.cols, img_transformed.rows)), 255 - mask);

    //cv::resize(stitched_img, stitched_img, cv::Size((int)((double)stitched_img.cols / stitched_img.rows * 600.0), 600));
    //cv::imshow("Imprinter Intermediate", stitched_img);
    //cv::waitKey(0);

    return stitched_img;
}