#include "../include/laneDetectorNode.hpp"
#include "../include/my_numpy.hpp"
using namespace std;
using namespace cv;

#include <ctime>
// Commonly used methods for utils
void my_imshow(const char *name, Mat image)
{
    imshow(name, image);
    cvMoveWindow(name, 100, 100);
}
void show_until_shutdown()
{
    char a = cv::waitKey(0);
    cv::destroyAllWindows();
}
template <typename T>
void print_vector(const vector<T> vec)
{
    if (vec.size() < 1)
    {
        return;
    }
    cout << "[";
    for (int i = 0; i < vec.size(); i++)
    {
        cout << vec[i];
        if (i != vec.size() - 1)
        {
            cout << ",";
        }
        else
        {
            cout << "]" << endl;
        }
    }
}
template <typename T>
void print_vector_point(const vector<T> vec)
{
    if (vec.size() < 1)
    {
        return;
    }
    cout << "[";
    for (int i = 0; i < vec.size(); i++)
    {
        cout << "(";
        cout << vec[i].x << "," << vec[i].y << ")";
        if (i != vec.size() - 1)
        {
            cout << ",";
        }
        else
        {
            cout << "]" << endl;
        }
    }
}
int main(int argc, char **argv)
{
    Mat image = imread("/home/owen/Desktop/play_ground/src/my_launch_car_sim/scripts/pictures/test_image8.png");
    laneDetectorNode lane_detector;
    clock_t start = clock();
    lane_detector._image_processor.process_image(image);
//transform to point lists;
    std::vector<Point> white_point_list_image, yellow_point_list_image;
    cv::findNonZero(lane_detector._image_processor._white_mask, white_point_list_image);
    cv::findNonZero(lane_detector._image_processor._yellow_mask, yellow_point_list_image);
    std::vector<Point2d> white_point_list, yellow_point_list;
    lane_detector._transformer.transformToVehicle(white_point_list_image, white_point_list, 0, 80);
    lane_detector._transformer.transformToVehicle(yellow_point_list_image, yellow_point_list, 0, 80);

    // std::vector<Point2d> test_array;
    // lane_detector._transformer.transformToImage(white_point_list, test_array);
    // Mat caveat = image.clone();
    // for (int i = 0; i < test_array.size(); i++)
    // {
    //     cv::circle(caveat, test_array[i], 2, Scalar(0, 255,0));
    // }
    // test_array.clear();
    // lane_detector._transformer.transformToImage(yellow_point_list, test_array);
    // for (int i = 0; i < test_array.size(); i++)
    // {
    //     cv::circle(caveat, test_array[i], 2, Scalar(0, 0, 255));
    // }
    // my_imshow("output", caveat);
    // show_until_shutdown();


//lane tracker update
    vector<Point2d> white_lane;
    vector<Point2d> yellow_lane;
    vector<Point2d> target_lane;
    
    lane_detector._lane_tracker.update_white_lane(white_point_list);
    lane_detector._lane_tracker.update_yellow_lane(yellow_point_list);
    const int PATH_LENGTH = 30;
    
    for (int i = 0; i < PATH_LENGTH; i++)
    {
        double x = i / (PATH_LENGTH / 20.0) + 8;
        double y_yellow = np::polyeval(lane_detector._lane_tracker._yellow_lane._coefficients, x);
        double y_white = np::polyeval(lane_detector._lane_tracker._white_lane._coefficients, x);
        double y_target = 0.5 * (y_yellow + y_white);
        white_lane.push_back(Point2d(x, y_white));
        yellow_lane.push_back(Point2d(x, y_yellow));
        target_lane.push_back(Point2d(x, y_target));
    }

    cout<< (clock() - start)/double(CLOCKS_PER_SEC) << endl;

    Mat caveat;
    vector<Point2d> white_image;
    lane_detector._transformer.transformToImage(white_lane, white_image);

    vector<Point2d> yellow_image;
    lane_detector._transformer.transformToImage(yellow_lane, yellow_image);

    vector<Point2d> target_image;
    lane_detector._transformer.transformToImage(target_lane, target_image);

    caveat = image.clone();
    for(int i = 0;i<PATH_LENGTH; i ++)
    {
        cv::circle(caveat, white_image[i], 2, Scalar(0, 255, 0));
        cv::circle(caveat, yellow_image[i], 2, Scalar(0, 0, 255));
        cv::circle(caveat, target_image[i], 2, Scalar(255, 0, 255));
    }
    my_imshow("output", caveat);
    show_until_shutdown();
}