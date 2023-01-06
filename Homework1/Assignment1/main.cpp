#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <math.h>

constexpr double MY_PI = 3.1415926;
using namespace std;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle_z, float rotation_angle_x = 0, float rotation_angle_y = 0)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f translate_z;
    rotation_angle_z = rotation_angle_z / 180.0 * MY_PI; 
    double cos_delta_z = std::cos(rotation_angle_z), sin_delta_z = std::sin(rotation_angle_z);
    translate_z << cos_delta_z, -sin_delta_z, 0, 0,
                 sin_delta_z, cos_delta_z, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

    Eigen::Matrix4f translate_y;
    rotation_angle_y = rotation_angle_y / 180.0 * MY_PI; 
    double cos_delta_y = std::cos(rotation_angle_y), sin_delta_y = std::sin(rotation_angle_y);
    translate_y << cos_delta_y, 0, sin_delta_y, 0,
                   0, 1, 0, 0,
                   -sin_delta_y, 0, cos_delta_y, 0,
                   0, 0, 0, 1;

    Eigen::Matrix4f translate_x;
    rotation_angle_x = rotation_angle_x / 180.0 * MY_PI; 
    double cos_delta_x = std::cos(rotation_angle_x), sin_delta_x = std::sin(rotation_angle_x);
    translate_x << 1, 0, 0, 0,
                 0, cos_delta_x, -sin_delta_x, 0,
                 0, sin_delta_x, cos_delta_x, 0,
                 0, 0, 0, 1;

    model = translate_x * translate_y * translate_z * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // Calculate n f l r t b
    float xLeft, xRight, yTop, yBottom;
    float fov = eye_fov / 180.0 * MY_PI;
    yTop = -zNear * std::tan(fov/2.0);
    yBottom = -yTop;
    xRight = aspect_ratio * yTop;
    xLeft = -xRight;

    // Three Steps: 1.Orthographic to Perspective 2.Translation 3.Scale
    
    Eigen::Matrix4f ortho2perspect;
    ortho2perspect << zNear, 0, 0, 0,
                      0, zNear, 0, 0,
                      0, 0, zNear + zFar, -zNear * zFar,
                      0, 0, 1, 0;

    Eigen::Matrix4f translation;
    translation << 1, 0, 0, -(xRight + xLeft)/2.0,
                   0, 1, 0, -(yTop + yBottom)/2.0,
                   0, 0, 1, -(zFar + zNear)/2.0,
                   0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.0/(xRight - xLeft), 0, 0, 0,
                   0, 2.0/(yTop - yBottom), 0, 0,
                   0, 0, 2.0/(zNear - zFar), 0,
                   0, 0, 0, 1;

    projection = scale * translation * ortho2perspect;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle_x = 0, angle_y = 0, angle_z = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 5) {
        command_line = true;
        angle_x = std::stof(argv[2]); // -r by default
        angle_y = std::stof(argv[3]); // -r by default
        angle_z = std::stof(argv[4]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[5]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle_z, angle_x, angle_y));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        cout << "angles: " << angle_x << " " << angle_y << " " << angle_z << endl;
        r.set_model(get_model_matrix(angle_z, angle_x, angle_y));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle_z += 10;
        }
        else if (key == 'd') {
            angle_z -= 10;
        } 
        else if (key == 'w') {
            angle_x += 10;
        }
        else if (key == 's') {
            angle_x -= 10;
        }
        else if (key == 'u') {
            angle_y += 10;
        }
        else if (key == 'p') {
            angle_y -= 10;
        }
    }

    return 0;
}
