#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
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

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) return control_points[0];
    std::vector<cv::Point2f> points;
    for (int i = 0; i < control_points.size()-1; i++) {
        auto pS = control_points[i];
        auto pE = control_points[i + 1];
        points.push_back((1 - t) * pS + t * pE);
    }

    return recursive_bezier(points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        //window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        int x = point.x - std::floor(point.x) < .5 ? std::floor(point.x) : std::ceil(point.x);
        int y = point.y - std::floor(point.y) < .5 ? std::floor(point.y) : std::ceil(point.y);
        auto p1 = cv::Point2i(x, y);
        
        /*float max = sqrt(2);
        for (float offsetX = -0.5f; offsetX < 1; offsetX++) {
            for (float offsetY = -0.5f; offsetY < 1; offsetY++) {
                float d = sqrt(std::pow((point.x - p1.x - offsetX), 2) + std::pow((point.y - p1.y - offsetY), 2));
                window.at<cv::Vec3b>(p1.y + offsetY, p1.x + offsetX)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y + offsetY, p1.x + offsetX)[1] + 255.f * (max - d) / max);
            }
        }*/

        auto tY = p1.y + .5f - point.y;
        auto bY = point.y - p1.y + .5f;
        auto tColorY = 255 * bY;
        auto bColorY = 255 * tY;
        
        auto rX = p1.x + .5f - point.x;
        auto lX = point.x - p1.x + .5f;
        auto rtColor = tColorY * lX;
        auto ltColor = tColorY * rX;
        auto rbColor = bColorY * lX;
        auto lbColor = bColorY * rX;

        window.at<cv::Vec3b>(p1.y - .5f, p1.x - .5f)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y - .5f, p1.x - .5f)[1] + lbColor);
        window.at<cv::Vec3b>(p1.y + .5f, p1.x - .5f)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y + .5f, p1.x - .5f)[1] + ltColor);
        window.at<cv::Vec3b>(p1.y - .5f, p1.x + .5f)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y - .5f, p1.x + .5f)[1] + rbColor);
        window.at<cv::Vec3b>(p1.y + .5f, p1.x + .5f)[1] = std::min(255.f, window.at<cv::Vec3b>(p1.y + .5f, p1.x + .5f)[1] + rtColor);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    control_points = { cv::Point2f(82, 422), cv::Point2f(184, 187), cv::Point2f(441, 133), cv::Point2f(593, 382) };
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
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
