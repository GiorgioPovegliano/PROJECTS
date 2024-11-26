

#include <iostream>
#include <opencv2/opencv.hpp>
#include <filter.h>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace cv;

// starting point for the application
int main(int argc, char** argv)
{
	String path = "pratodellavalle";						// change this string to operate on different images
	// load an image and place it in the img variable
	Mat img = imread(path + "\\image_to_complete.jpg");     // you can load any .jpg image, select an image of your choice and place in the project folder
	Mat result = img.clone();								// image to show the result of the computation
	int num_patches = 3;									// WARNING: when working with different datasets you also need to change this variable
	bool transformed = 0;									// set to 1 if you want to work with transformed patches
	bool orb = 0;											// set to 1 if you want to work with ORB, 0 if you want to use SIFT
	imshow("img", img);

	//compute SIFT features of the image
	Ptr<Feature2D> siftobj = SIFT::create(0, 8, 0.04, 5, 1.6);
	Ptr<ORB> orbobj = ORB::create(100000);
	vector<KeyPoint> keypoints;				// vector of keypoints of the original image
	Mat descriptors;
	if(orb == 1) {
		orbobj->detectAndCompute(img, noArray(), keypoints, descriptors, false);
	}
	else {
		siftobj->detectAndCompute(img, noArray(), keypoints, descriptors, false);
	}
	//do the same with patches
	vector<vector<KeyPoint>> keypoints_p;	// vector of vectors of keypoints, each element contains the keypoints of a patch
	vector<Mat> descriptors_p;				// vector of descriptors, each element contains the descriptor of a patch
	vector<Mat> patches;
	//also for flipped patches (in case transformed = 1)
	vector<vector<KeyPoint>> keypoints_p_flipped;
	vector<Mat> descriptors_p_flipped;
	vector<Mat> patches_flipped;
	if(transformed == 0) {
		for (int i = 0; i < num_patches; i++) {
			// read in patch image
			string patch_path = path + "\\patch_" + to_string(i) + ".jpg";
			Mat patch_temp = imread(patch_path);
			patches.push_back(patch_temp);		// append the current patch to the patches image vector
			// detect keypoints and compute descriptors for patch
			vector<KeyPoint> keypoints_temp;
			Mat descriptors_temp;
			if (orb == 1) {
				orbobj->detectAndCompute(patch_temp, noArray(), keypoints_temp, descriptors_temp, false);
			}
			else {
				siftobj->detectAndCompute(patch_temp, noArray(), keypoints_temp, descriptors_temp, false);
			}
			// add keypoints and descriptors to vectors
			keypoints_p.push_back(keypoints_temp);
			descriptors_p.push_back(descriptors_temp);
		}
	}
	else {
		// do the same for the transformed patches but compute also the descriptors for the flipped image
		for (int i = 0; i < num_patches; i++) {
			// read in patch image
			string patch_path = path + "\\patch_t_" + to_string(i) + ".jpg";
			Mat patch_temp = imread(patch_path);
			patches.push_back(patch_temp);					// append the current patch to the patches image vector
			Mat patch_flipped;
			flip(patch_temp, patch_flipped, 1);
			patches_flipped.push_back(patch_flipped);		// append the flipped current patch to the flipped patches image vector
			// detect keypoints and compute descriptors for patch
			vector<KeyPoint> keypoints_temp;
			Mat descriptors_temp;
			if(orb == 1) {
				orbobj->detectAndCompute(patch_temp, noArray(), keypoints_temp, descriptors_temp, false);
			}
			else {
				siftobj->detectAndCompute(patch_temp, noArray(), keypoints_temp, descriptors_temp, false);
			}
			// add keypoints and descriptors to vectors
			keypoints_p.push_back(keypoints_temp);
			descriptors_p.push_back(descriptors_temp);
			// do the same for flipped 
			if(orb == 1) {
				orbobj->detectAndCompute(patch_flipped, noArray(), keypoints_temp, descriptors_temp, false);
			}
			else {
				siftobj->detectAndCompute(patch_flipped, noArray(), keypoints_temp, descriptors_temp, false);
			}
			// add keypoints and descriptors to vectors
			keypoints_p_flipped.push_back(keypoints_temp);
			descriptors_p_flipped.push_back(descriptors_temp);
		}
	}
	// match descriptors using Brute-Force matcher
	BFMatcher matcher;
	if(orb == 1) {
		matcher = BFMatcher(NORM_HAMMING, true);
	}
	else {
		matcher = BFMatcher(NORM_L2, true);
	}
	// variable where we store the matches
	vector<vector<DMatch>> matches(num_patches);	// initialize the matches vector 
	vector<vector<DMatch>> matches_flipped(num_patches);
	// user-defined ratio 
	float ratio = 2;
	// variable that controls how many distances to confront in order to check if the image is flipped
	int count = 5;
	vector<float> min_distance;
	vector<float> threshold;
	vector<float> average_distance;
	if(transformed == 0) {
		for (int i = 0; i < num_patches; i++) {
			matcher.match(descriptors, descriptors_p[i], matches[i]);
			// sort matches by distance
			sort(matches[i].begin(), matches[i].end());
			// pick the first element as the minimum distance
			min_distance.push_back(matches[i][0].distance);
			if(orb == 1) {
				threshold.push_back(3);						  // for orb I need to set a fixed threshold (otherwise threshold would be 0)
			}
			else {
				threshold.push_back(ratio * min_distance[i]); // for sift we can compute the threshold as explained in the slides
			}
		}
	}
	else {
		for (int i = 0; i < num_patches; i++) {
			matcher.match(descriptors, descriptors_p[i], matches[i]);
			matcher.match(descriptors, descriptors_p_flipped[i], matches_flipped[i]);
			// sort matches by distance
			sort(matches[i].begin(), matches[i].end());
			sort(matches_flipped[i].begin(), matches_flipped[i].end());
			// check if the average of the distances is less for the flipped image
			float sum_distance = 0;
			float sum_distance_flipped = 0;
			for (int j = 0; j < count; j++) {
				sum_distance += matches[i][j].distance;
				sum_distance_flipped += matches_flipped[i][j].distance;
			}
			if((sum_distance_flipped/count) < (sum_distance/count)) {
				matches[i] = matches_flipped[i];
				patches[i] = patches_flipped[i];
				descriptors_p[i] = descriptors_p_flipped[i];
				keypoints_p[i] = keypoints_p_flipped[i];
			}
			// pick the first element as the minimum distance
			min_distance.push_back(matches[i][0].distance);
			if (orb == 1) {
				threshold.push_back(10);
			}
			else {
				threshold.push_back(ratio * min_distance[i]);
			}
		}
	}

	// create a mask of the elements with distance < threshold
	vector<vector<bool>> mask(matches.size()); 
	// vector of 2 integers representing the coordinates of the pixels of refined matches in the source and destination image
	vector<vector<Point2f>> srcPoints(matches.size()), dstPoints(matches.size());
	vector<vector<DMatch>> matches_def(matches.size());
	for (int j = 0; j < matches.size(); j++) {
		for (int i = 0; i < matches[j].size(); i++) {
			float distance = matches[j][i].distance;
			if (distance < threshold[j]) {
				mask[j].push_back(true);
				srcPoints[j].push_back(keypoints[matches[j][i].queryIdx].pt);
				dstPoints[j].push_back(keypoints_p[j][matches[j][i].trainIdx].pt);
				matches_def[j].push_back(matches[j][i]);
			}
			else {
				mask[j].push_back(false);
			}
		}
	}
	// output the matches
	Mat output;
	Mat homography;
	Mat patchedImage = img.clone();  // copy of image2
	for (int i = 0; i < num_patches; i++) {
		drawMatches(img, keypoints, patches[i], keypoints_p[i], matches_def[i], output);
		namedWindow("Matching", WINDOW_NORMAL);
		imshow("Matching", output);
		waitKey(0);
		// compute homography
		homography = findHomography(dstPoints[i], srcPoints[i], RANSAC);


		// compute the corners of the patch in order to create a mask (Rect) to paste the patch in the original image
		vector<Point2f> patchCorners(4);
		patchCorners[0] = Point2f(0, 0);
		patchCorners[1] = Point2f(patches[i].cols, 0);
		patchCorners[2] = Point2f(patches[i].cols, patches[i].rows);
		patchCorners[3] = Point2f(0, patches[i].rows);
		// transform the corners of the patch using the homography matrix
		vector<Point2f> warpedCorners;
		perspectiveTransform(patchCorners, warpedCorners, homography);

		// compute the top-left and bottom-right corners
		Size imgSize = img.size();
		Point2f topLeft(imgSize.width, imgSize.height);
		Point2f bottomRight(0, 0);

		for (Point2f corner : warpedCorners) {
			topLeft.x = min(topLeft.x, corner.x)+2;
			topLeft.y = min(topLeft.y, corner.y)+2;
			bottomRight.x = max(bottomRight.x, corner.x)-1;
			bottomRight.y = max(bottomRight.y, corner.y)-1;
		}
		int warpedHeight = bottomRight.y - topLeft.y;
		int warpedWidth = bottomRight.x - topLeft.x;
		// create a rectangle representing the size of the warped image
		Rect warpedRect(topLeft, Size(warpedWidth, warpedHeight));
		// create a new matrix to hold the warped patch
		Mat warpedPatch;
		// apply perspective transformation to the patch
		warpPerspective(patches[i], warpedPatch, homography, img.size());
		warpedPatch(warpedRect).copyTo(result(warpedRect));
		namedWindow("Result",WINDOW_NORMAL);
		imshow("Result", result);
		waitKey(0);
		
	}
	// another technique for matching: TEMPLATE MATCHING
	Mat result_2 = img.clone();
	for (int i = 0; i < num_patches; i++) {
		// find the best match using template matching
		Mat resultImage;
		matchTemplate(img, patches[i], resultImage, TM_CCOEFF_NORMED);

		// find the location of the best match
		Point minLoc, maxLoc;
		double minVal, maxVal;
		minMaxLoc(resultImage, &minVal, &maxVal, &minLoc, &maxLoc);

		// draw a rectangle around the best match region
		Rect patchRect(maxLoc, Point(maxLoc.x + patches[i].cols, maxLoc.y + patches[i].rows));
		rectangle(result_2, patchRect, Scalar(0, 255, 0), 2);
		// paste the matched patch onto the input image
		patches[i].copyTo(result_2(patchRect));
	}
	// display the result
	namedWindow("Result with matchTemplate", WINDOW_NORMAL);
	imshow("Result with matchTemplate", result_2);
	waitKey(0);
 

	
	return 0;
}
