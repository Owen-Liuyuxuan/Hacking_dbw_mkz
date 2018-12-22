#pragma once
#include <vector>
#include <iostream>
#include <random>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "my_numpy.hpp"

const int ORDER = 2;
class RANSACMachine
{
  public:
    RANSACMachine(const int order_polynomial = 2) : _order_polynomial(order_polynomial)
    {
        ;
    }
    double sample_once(const std::vector<cv::Point2d> non_zero_point_list, Eigen::VectorXd &src_coefficient_vector, int number_points = ORDER + 1, double bearing = 1)
    {
        Eigen::VectorXd x_list(number_points);
        Eigen::VectorXd y_list(number_points);

        np::get_different_x_y_list(non_zero_point_list, x_list, y_list, number_points);
        src_coefficient_vector = np::polyfit(x_list, y_list, _order_polynomial);

        return _get_agree_rate(src_coefficient_vector, non_zero_point_list, bearing);
    }
    double _get_agree_rate(const Eigen::VectorXd coeffs, const std::vector<cv::Point2d> non_zero_point_list, double bearing)
    {
        double total_agreement = 0;

        for (int i = 0; i < non_zero_point_list.size(); i++)
        {
            double x = non_zero_point_list[i].x;
            double true_y = non_zero_point_list[i].y;
            double predicted_y = np::polyeval(coeffs, x);
            if (abs(true_y - predicted_y) < bearing)
                total_agreement += 1;
        }

        return total_agreement / non_zero_point_list.size();
    }
    int _order_polynomial;
};

class lane
{
  public:
    lane(const int order_polynomial = 3, double filter_lambda = 0.7) : _order_polynomial(order_polynomial), _filter_lambda(filter_lambda)
    {
        is_init = false;
    }
    void reinit()
    {
        is_init = false;
    }
    void update(const Eigen::VectorXd coeffs)
    {
        if (is_init)
        {
            //_coefficients = _coefficients * _filter_lambda + coeffs * (1 - _filter_lambda);
            _coefficients = coeffs;
        }
        else
        {
            is_init = true;
            _coefficients = coeffs;
        }
    }

    Eigen::VectorXd _coefficients;
    bool is_init;
    int _order_polynomial;
    double _filter_lambda;
};

class roadTracker
{

  public:
    roadTracker()
    {

        polyfitter = RANSACMachine(ORDER);
        _white_lane = lane(ORDER);
        _yellow_lane = lane(ORDER);
        _yellow_lane._filter_lambda = 0.1;
        _white_lane._filter_lambda = 0.1;
        _yellow_weight = 0.5;
        _white_weight = 0.5;
    }
    void get_target_road(std::vector<cv::Point2d> &result_target);
    void update_white_lane(const std::vector<cv::Point2d> white_point_list, int number_points = ORDER + 8, int max_iter = 60)
    {
        if (white_point_list.size() < 2 * number_points)
        {
            _white_weight *= 0.8;
            return;
        }
        _white_weight = 0.5;

        Eigen::VectorXd best_coeffs = _white_lane._coefficients;
        double best_agree_rate = 0;
        for (int i = 0; i < max_iter; i++)
        {
            Eigen::VectorXd temp_coeffs;
            double agree_rate = polyfitter.sample_once(white_point_list, temp_coeffs, number_points, 0.25);
            if (agree_rate > best_agree_rate)
            {
                best_agree_rate = agree_rate;
                best_coeffs = temp_coeffs;
            }
        }
        _white_lane.update(best_coeffs);
    }
    void update_yellow_lane(const std::vector<cv::Point2d> yellow_point_list, int number_points = ORDER + 8, int max_iter = 60)
    {
        double best_agree_rate = 0;
        Eigen::VectorXd best_coeffs = _yellow_lane._coefficients;
        std::vector<cv::Point2d> points_on_right;
        for (auto point : yellow_point_list)
        {
            double white_y_here = np::polyeval(_white_lane._coefficients, point.x);
            if (point.y < white_y_here)
            {
                points_on_right.push_back(point);
            }
        }

        if (points_on_right.size() < 2 * number_points)
        {
            _yellow_weight *= 0.8;
            return;
        }
        else
        {
            _yellow_weight = 0.5;

            for (int i = 0; i < max_iter; i++)
            {
                Eigen::VectorXd temp_coeffs;
                double agree_rate = polyfitter.sample_once(points_on_right, temp_coeffs, number_points, 0.25);
                if (agree_rate > best_agree_rate)
                {
                    best_agree_rate = agree_rate;
                    best_coeffs = temp_coeffs;
                }
            }

            _yellow_lane.update(best_coeffs);
        }
    }

    RANSACMachine polyfitter;
    lane _white_lane;
    lane _yellow_lane;
    double _yellow_weight;
    double _white_weight;
};