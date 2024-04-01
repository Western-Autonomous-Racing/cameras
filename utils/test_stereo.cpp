#include "../include/StereoCamera.hpp"
#include <opencv4/opencv2/opencv.hpp>

using namespace std;

int main(int argc, char** argv)
{
    StereoCamera camera(1, 1000);

    if (!camera.isOpened())
    {
        cout << "Camera failed to open" << endl;
        return -1;
    }

    while (1)
    {
        StereoFrame left, right, depth;
        camera.getFrames(&left, &right, &depth);

        if (left.frame.empty() && right.frame.empty())
        {
            cout << "Frame is empty" << endl;
            break;
        }

        cv::Mat img;
        cv::hconcat(left.frame, right.frame, img);

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