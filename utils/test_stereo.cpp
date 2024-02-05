#include "../include/StereoCamera.hpp"
#include <opencv4/opencv2/opencv.hpp>

using namespace std;

int main(int argc, char** argv)
{
    StereoCamera camera;

    if (!camera.isOpened())
    {
        cout << "Camera failed to open" << endl;
        return -1;
    }

    while (1)
    {
        cv::Mat left = camera.getLeftFrame();
        cv::Mat right = camera.getRightFrame();

        if (left.empty() && right.empty())
        {
            cout << "Frame is empty" << endl;
            break;
        }

        cv::Mat img;
        cv::hconcat(left, right, img);

        cv::imshow("Frame", img);
        int key = cv::waitKey(1);

        if (key == 'q')
        {
            break;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}