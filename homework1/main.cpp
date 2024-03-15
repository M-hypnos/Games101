#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float angle = rotation_angle * M_PI / 180;
    Eigen::Matrix4f translate;
    translate << 
        cos(angle), -sin(angle), 0,0, 
        sin(angle), cos(angle), 0, 0, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

    model = translate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float n = -zNear;
    float f = -zFar;

    float t = zNear * tan(eye_fov / 2);
    float r = t * aspect_ratio;
    Eigen::Matrix4f translate;
    translate <<
        1 / r, 0, 0, 0,
        0, 1 / r, 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    Eigen::Matrix4f translate1;
    translate1 <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    Eigen::Matrix4f translate2;
    translate2 <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    projection = translate * translate1 * translate2 * projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f translate;
    float a = angle * M_PI / 180;
    /* Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    translate <<
        0, -axis[2], axis[1], 0,
        axis[2], 0, -axis[0], 0,
        -axis[1], axis[0], 0, 0,
        0, 0, 0, 1;
    Vector4f axisEx;
    axisEx << axis, 0;
    Eigen::Matrix4f i;
    i = (1 - cos(a)) * axisEx * axisEx.transpose();
    translate = cos(a) * I + (1 - cos(a)) * axisEx * axisEx.transpose() + sin(a) * translate;*/
    //上面写法 cos(a) * I 之后 会导致 w不位1，因为单位矩阵第4行第4列的1，乘了cos(a) 后不再为1

    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f translate3;
    translate3 <<
        0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;
    translate3 = cos(a) * I3 + (1 - cos(a)) * axis * axis.transpose() + sin(a) * translate3;
    translate <<
        translate3(0, 0), translate3(0, 1), translate3(0, 2), 0,
        translate3(1, 0), translate3(1, 1), translate3(1, 2), 0,
        translate3(2, 0), translate3(2, 1), translate3(2, 2), 0,
        0, 0, 0, 1;

    return translate;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
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

        r.set_model(get_model_matrix(angle));
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

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(Eigen::Vector3f(0,0, -1), angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
