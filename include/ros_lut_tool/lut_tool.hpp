#ifndef __ROS_LUT_TOOL_H_
#define __ROS_LUT_TOOL_H_

#include <iostream>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <mutex>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <yaml-cpp/yaml.h>
#include <npuvision/npuvision.h>
#include "npuvision/LUT.h"

const int frame_width{640};
const int frame_height{360};

const int hue_range{256};
const int sat_range{256};

struct BallParam
{
    int min_val{0};
    int max_val{255};
    int min_sat{0};
    int max_sat{255};
} ;

struct WhiteParam
{
    int min_hue{0};
    int max_hue{255};
    int min_sat{0};
    int max_sat{255};
    int min_val{0};
    int max_val{255};
} ;

struct MouseStuff
{
    bool left_down{false};
    bool left_up{false};

    cv::Point coor1;
    cv::Point coor2;
} ;

class Lut_tool
{
private:
    /* data */

public:
    Lut_tool(/* args */);
    ~Lut_tool();

    void process();

    ros::NodeHandle nh_;

    cv_bridge::CvImagePtr cv_img_ptr_subs_;
    cv_bridge::CvImage cv_img_pubs_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber it_subs_;
    image_transport::Publisher it_pubs_;

    cv::Mat all_in_one;
    cv::MatND hist_reference[3];
    cv::Mat color_table;
    cv::Mat input;
    cv::Mat input_hsv;
    cv::Mat src_img;

    std::string lut_path = "../data/config.yaml";
    std::string video_path = "/home/wujs/dataset/robocup/cup18.avi";
    std::string param_winname = "[lut_tool] Parameters config";
    std::string winname = "[lut_tool] Calibrator";

    unsigned int img_encoding_;

    // ros::Time stamp_;
    // std::string frame_id_;

    BallParam ball_param;
    WhiteParam white_param;
    MouseStuff mouse_stuff;

    std::mutex lock_;

    int cycle_ = 0;

    void loadConfig();
    void setupTrackbar();
    void modifyTable(const cv::Mat &data, int flag);
    void modifyTableRange(int flag);
    void zeroTable();
    void getHist(const cv::Mat &ref, cv::MatND hist[3]);
    cv::Mat segmentField(const cv::Mat &_in_hsv);
    cv::Mat segmentBall(const cv::Mat &_in_hsv);
    cv::Mat segmentOutput(cv::Mat &_in);
    cv::Mat segmentWhite(const cv::Mat &_in_hsv);
    cv::Mat makeAll(const cv::Mat tl, const cv::Mat tr, const cv::Mat bl, const cv::Mat br);
    void drawText();
    void getTrackbar();

    // 
    void imageCallback(const sensor_msgs::ImageConstPtr &_msg);
    
};

void mouse_call(int event, int x, int y, int flags, void *data);


#endif // __ROS_LUT_TOOL__