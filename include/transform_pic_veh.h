#pragma once
#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

class imageVehicleTransformer
{
  public:
    imageVehicleTransformer(){;}
    imageVehicleTransformer(Eigen::Matrix3d in_R, Eigen::Vector3d in_T, Eigen::Matrix3d in_intrinsicMatrix) : R(in_R), T(in_T), intrinsicMatrix(in_intrinsicMatrix)
    {
        ;
    }

    void setR(const Eigen::Matrix3d in_R) { R = in_R; }
    void getR(Eigen::Matrix3d &get_R) { get_R = R; }

    void setT(const Eigen::Vector3d in_T) { T = in_T;}
    void getT(Eigen::Vector3d &get_T) { get_T = T; }

    void setK(const Eigen::Matrix3d in_K) { intrinsicMatrix = in_K; }
    void getK(Eigen::Matrix3d &get_K) { get_K = intrinsicMatrix; }

    void initTransforms()
    {
        Eigen::MatrixXd CamMatrix(3, 4);
        CamMatrix.block<3, 3>(0, 0) = R;
        CamMatrix.block<3, 1>(0, 3) = T;
        Eigen::Matrix3d CamMatrixNoZ(3, 3);
        CamMatrixNoZ.block<3, 2>(0, 0) = CamMatrix.block<3, 2>(0, 0);
        CamMatrixNoZ.block<3, 1>(0, 2) = CamMatrix.block<3, 1>(0, 3);

        tformToImage = intrinsicMatrix * CamMatrixNoZ;
        // tformToImage.block<3, 2>(0, 0) = tformToImage3D.block<3, 2>(0, 0);
        // tformToImage.block<3, 1>(0, 2) = tformToImage3D.block<3, 1>(0, 3);

        tformToVehicle = tformToImage.inverse();
    }
    template <typename T>
    void transformToImage(const std::vector<T> vehicle_path, std::vector<cv::Point2d> &image_path)
    {
        Eigen::Vector3d input_point, output_point;
        image_path.resize(vehicle_path.size());
        for (int i = 0; i < vehicle_path.size(); i++)
        {
            input_point << double(vehicle_path[i].x), double(vehicle_path[i].y), 1.0;
            output_point = tformToImage * input_point;
            image_path[i] = cv::Point2d(output_point(0)/output_point(2), output_point(1)/output_point(2));
        }
    }
    template <typename T>
    void transformToVehicle(const std::vector<T> image_path, std::vector<cv::Point2d> &vehicle_path, const int x_offset = 0, const int y_offset = 0)
    {
        Eigen::Vector3d image_point, output_point;
        vehicle_path.resize(image_path.size());
        for (int i = 0; i < image_path.size(); i++)
        {
            image_point << double(image_path[i].x) + x_offset, double(image_path[i].y) + y_offset, 1.0;
            output_point = tformToVehicle * image_point;
            vehicle_path[i] = cv::Point2d(output_point(0)/output_point(2), output_point(1)/output_point(2));
        }
    }

  private:
    Eigen::Matrix3d R;
    Eigen::Vector3d T; // Vector is 3*1 shape
    Eigen::Matrix3d intrinsicMatrix;
    Eigen::Matrix3d tformToVehicle;
    Eigen::Matrix3d tformToImage;
};