#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#define POINTS_COUNT 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < POINTS_COUNT
    ) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// lhn
cv::Point2f calculate_point(const cv::Point2f &p1, const cv::Point2f &p2, float t) {
    return ((1.0-t)*p1 + t*p2);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int points_num = control_points.size();
    if(points_num < 1) {
        perror("Error control points number!\n");
        fflush(stderr);
        exit(-1);
    }
    if(points_num == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_points;
    for(int i = 0; i < points_num - 1; ++i) {
        new_points.emplace_back(calculate_point(control_points[i], control_points[i+1], t));
    }

    return recursive_bezier(new_points, t);
}

// lhn
const int d[3] = {-1, 0, 1};

float cal_distance(const cv::Point2f &p1, const cv::Point2f &p2) {
    float distance;  
    distance = powf((p1.x - p2.x),2) + powf((p1.y - p2.y),2);  
    distance = sqrtf(distance);  
    return distance; 
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    const int N = 1000;
    for(int i = 1; i <= N; ++i) {
        float t = (float)i / (float)N;
        auto point = recursive_bezier(control_points, t);
        // if(i % 10 == 0)
        //     printf("Point at [%f, %f], t=%f \n", point.y, point.x, t);
        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        // lhn Antialiasing: one point to one pixel ==> one point to nine pixels
        // find neighbour points
        std::vector<std::vector<float>> distance_points(3,std::vector<float>(3));
        float distance_max = 0.0;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                cv::Point2f curr_pixel(point.x + d[i], point.y + d[j]);
                float distance = cal_distance(curr_pixel, point);
                distance_max = std::max(distance, distance_max);
                distance_points[i][j] = distance;
            }
        }
        // render
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                window.at<cv::Vec3b>(point.y + d[j], point.x + d[i])[1] = 255.0 * distance_points[i][j] / distance_max;
            }
        }

    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == POINTS_COUNT) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
