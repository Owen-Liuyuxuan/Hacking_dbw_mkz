#pragma once
#include <vector>
#include <iostream>
#include <random>
#include  <time.h>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
namespace np
{
// Evaluate a polynomial.

double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

double get_distance(const Eigen::VectorXd coeffs1, const Eigen::VectorXd coeffs2)
{
    assert(coeffs1.size() == coeffs2.size());
    double mse = 0;
    for (int i = 0, x = 0; i < 10; i++, x += 5)
    {
        double difference = polyeval(coeffs1, x) - polyeval(coeffs2, x);
        mse = (mse * i + difference * difference) / (i + 1.0);
    }
    return mse;
}

void get_different_x_y_list(const std::vector<cv::Point2d> non_zero_point_list, Eigen::VectorXd &x_list, Eigen::VectorXd &y_list, const int number_points = 3)
{
    int i = 0;
    srand((unsigned)time(0));
    while (i < number_points)
    {
        int random_index = rand() % (non_zero_point_list.size());
        double x = non_zero_point_list[random_index].x;
        double y = non_zero_point_list[random_index].y;
        
        bool is_found_same_x = false;
        for (int j = 0; j < i; j++)
        {
            if ((x-x_list[j]) < 0.001 && x-x_list[j] > -0.001)
            {

                is_found_same_x = true;
                break;
            }
        }
            
        if (!is_found_same_x)
        {
            x_list[i] = x;
            y_list[i] = y;
            i = i + 1;
        }
    }
}

} // namespace np
