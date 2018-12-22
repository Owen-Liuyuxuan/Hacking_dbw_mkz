//Eigen Libraries, opencv, std Libraries
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include <vector>

class imageProcessor
{
public:
    imageProcessor(){;}
    void process_image(cv::Mat raw_image)
    {
        
        cv::Mat ROI_image = get_ROI_image(raw_image);
        _yellow_mask = get_yellow_mask(ROI_image);
        _white_mask = get_white_mask(ROI_image);
        cv::Mat road_mask = get_black_mask(ROI_image);
        cv::bitwise_and(road_mask, _white_mask, _white_mask);
        //cv::bitwise_and(road_mask, _yellow_mask, _yellow_mask);
    }
    //Methods for getting picture we want
    cv::Mat get_ROI_image(const cv::Mat original_image)
    {
        cv::Mat output_image;
        output_image = original_image(cv::Rect(0, 80, 800, (230 - 80))).clone();
        return output_image;
    }
    //Methods for thresholding
    cv::Mat get_yellow_mask(const cv::Mat original_image)
    {
        cv::Mat output_image;
        cv::Mat hls_image;
        cv::cvtColor(original_image, hls_image, cv::COLOR_BGR2HLS);
        std::vector<cv::Mat> hls_channels;
        cv::split(hls_image, hls_channels);
        cv::Mat yellow_mask;
        cv::inRange(hls_channels.at(0), 20, 30, yellow_mask);
        cv::Mat colorful_mask;
        cv::inRange(hls_channels.at(2), 128, 255, colorful_mask);
        cv::bitwise_and(yellow_mask, colorful_mask, yellow_mask);
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        for (int i = 0; i < 1; i++)
        {
            cv::dilate(yellow_mask, yellow_mask, element);
        }
        cv::Mat edged_mask;
        cv::Canny(yellow_mask, edged_mask, 100, 150);
        return edged_mask;
    }
    cv::Mat get_white_mask(const cv::Mat original_image)
    {
        cv::Mat hls_image;
        cv::cvtColor(original_image, hls_image, cv::COLOR_BGR2HLS);
        std::vector<cv::Mat> hls_channels;
        cv::split(hls_image, hls_channels);

        cv::Mat bright_mask, plain_mask;
        cv::inRange(hls_channels.at(1), 60, 255, bright_mask);
        cv::inRange(hls_channels.at(2), 0, 10, plain_mask);

        cv::Mat white_mask;
        cv::bitwise_and(bright_mask, plain_mask, white_mask);

        cv::Mat edged_mask;
        cv::Canny(white_mask, edged_mask, 100, 150);
        return edged_mask;
    }
    cv::Mat get_black_mask(const cv::Mat original_image)
    {
        cv::Mat hls_image;
        cv::cvtColor(original_image, hls_image, cv::COLOR_BGR2HLS);
        std::vector<cv::Mat> hls_channels;
        cv::split(hls_image, hls_channels);

        cv::Mat dark_mask, plain_mask;
        cv::inRange(hls_channels.at(1), 0, 30, dark_mask);
        cv::inRange(hls_channels.at(2), 0, 20, plain_mask);

        cv::Mat black_mask;
        cv::bitwise_and(dark_mask, plain_mask, black_mask);

        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        for (int i = 0; i < 1; i++)
        {
            cv::dilate(black_mask, black_mask, element);
            //erode(black_mask, black_mask, element, Point(-1, -1));
        }

        int largest_area = 0;
        int largest_contour_index = 0;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i++) // iterate through each contour.
        {
            double a = cv::contourArea(contours[i], false); //  Find the area of contour
            if (a > largest_area)
            {
                largest_area = a;
                largest_contour_index = i; //Store the index of largest contour
            }
        }
        black_mask = 0;
        cv::drawContours(black_mask, contours, largest_contour_index, cv::Scalar(255), -1);
        return black_mask;
    }
    cv::Mat _white_mask;
    cv::Mat _yellow_mask;
};