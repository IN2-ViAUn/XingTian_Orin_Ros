#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define IMAGE_TOPIC "/rgb/image_raw"

using namespace std;

void GrabImage(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgb");
    ros::start();


    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe(IMAGE_TOPIC, 1, GrabImage);

    ros::spin();

    ros::shutdown();

    return 0;
}

void GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
        printf("Image received\n");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // 检查图像是否为空
    if(cv_ptr->image.empty())
    {
        cerr << endl << "Failed to load image from topic " << IMAGE_TOPIC << endl;
        return;
    }
    printf("Image received2\n");

    // 打印图像的shape
    cout << "Image size: " << cv_ptr->image.size() << endl;
    int channels = cv_ptr->image.channels();
    std::cout << "Number of channels: " << channels << std::endl;

    // 打印image的数据类型，打印成CV_8UC3或者其他的
    std::cout << "Image type: " << cv_ptr->image.type() << std::endl;
    printf("Expect Type: %d\n", CV_8UC3);

    // // 打印所有图像数据
    // for (int i = 0; i < cv_ptr->image.rows; i++)
    // {
    //     for (int j = 0; j < cv_ptr->image.cols; j++)
    //     {
    //         for (int k = 0; k < channels; k++)
    //         {
    //             printf("%d ", cv_ptr->image.at<cv::Vec3b>(i, j)[k]);
    //         }
    //     }
    // }

    // 读取成cv::Mat
    cv::Mat image = cv_ptr->image;

    // 随机创建一个cv::Mat的数据类型，大小为1280*720*3，数据类型为CV_8UC3，值为0，然后用imshow显示
    // cv::Size size(1280, 720);
    // cv::Mat image2 = cv::Mat::zeros(size, CV_8UC3);
    // // 打印image2的数据类型
    // cv::imshow("image2", image2);
    // cv::waitKey(1);

    printf("Image received3\n");

    // 图像数据类型转换，转换为CV_8UC3

    // 显示图像
    cv::imshow("image", image);
    printf("Image received4\n");
    cv::waitKey(1);
    

    printf("Image received3\n");

}


