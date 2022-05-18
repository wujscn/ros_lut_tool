
#include "ros_lut_tool/lut_tool.hpp"

// Lut_tool *Lut_tool::instance_ = NULL;
void Lut_tool::loadConfig()
{
    cv::FileStorage fs(lut_path, cv::FileStorage::READ);
    fs["color_table"] >> color_table;
    // cv::resize(color_table, color_table, cv::Size(256, 256));
    // fs["min_val"] >> ball_param.min_val;
    // fs["max_val"] >> ball_param.max_val;
    // fs["min_sat"] >> ball_param.min_sat;
    // fs["max_sat"] >> ball_param.max_sat;
    fs["min_val"] >> white_param.min_val;
    fs["max_val"] >> white_param.max_val;
    fs["min_sat"] >> white_param.min_sat;
    fs["max_sat"] >> white_param.max_sat;
    fs["min_hue"] >> white_param.min_hue;
    fs["max_hue"] >> white_param.max_hue;
    fs.release();
}

void Lut_tool::setupTrackbar()
{

    cv::namedWindow(param_winname, cv::WINDOW_NORMAL);

    // cv::createTrackbar("Ball Min. Val", param_winname, &ball_param.min_val, 255);
    // cv::createTrackbar("Ball Max. Val", param_winname, &ball_param.max_val, 255);
    // cv::createTrackbar("Ball Min. Sat", param_winname, &ball_param.min_sat, 255);
    // cv::createTrackbar("Ball Max. Sat", param_winname, &ball_param.max_sat, 255);
    //
    // cv::createTrackbar("White Min. Val", param_winname, &white_param.min_val, 255);
    // cv::createTrackbar("White Max. Val", param_winname, &white_param.max_val, 255);
    cv::createTrackbar("White Min. Sat", param_winname, &white_param.min_sat, 255);
    cv::createTrackbar("White Max. Sat", param_winname, &white_param.max_sat, 255);
    cv::createTrackbar("White Min. Hue", param_winname, &white_param.min_hue, 255);
    cv::createTrackbar("White Max. Hue", param_winname, &white_param.max_hue, 255);
}

Lut_tool::Lut_tool(/* args */) : nh_(ros::this_node::getName()),
                                 it_(this->nh_), it_subs_(it_.subscribe("image_in", 1, &Lut_tool::imageCallback, this))
{
    std::cout << "init" << std::endl;
    color_table = cv::Mat(hue_range, sat_range, CV_8UC1);

    cv::setMouseCallback(winname, mouse_call, 0);
    nh_.param<std::string>("ball_config_path", lut_path,
                           ros::package::getPath("ros_lut_tool") + "/config/config.yaml");

    std::cout << "config path" << lut_path << std::endl;
    std::cout << color_table << std::endl;
    loadConfig();
    setupTrackbar();
}

Lut_tool::~Lut_tool()
{
}

// Lut_tool* Lut_tool::getInstance() {
//   if (!instance_) instance_ = new Lut_tool();
//   return instance_;
// }

void Lut_tool::modifyTable(const cv::Mat &data, int flag)
{
    cv::Mat temp;
    cv::FileStorage fs(lut_path, cv::FileStorage::READ);
    fs["color_table"] >> temp;
    fs.release();

    if (!temp.data)
        temp = cv::Mat::zeros(256, 256, CV_8UC1);

    for (int i(0); i < data.rows; i++)
    {
        const cv::Vec3b *data_ptr = data.ptr<cv::Vec3b>(i);
        for (int j(0); j < data.cols; j++)
        {
            int hue = data_ptr[j][0];
            int sat = data_ptr[j][1];
            temp.at<uchar>(hue, sat) = flag;
            color_table.at<uchar>(hue, sat) = flag;
        }
    }

    fs.open(lut_path, cv::FileStorage::WRITE);
    fs << "color_table" << temp;
    fs << "min_val" << white_param.min_val;
    fs << "max_val" << white_param.max_val;
    fs << "min_sat" << white_param.min_sat;
    fs << "max_sat" << white_param.max_sat;
    fs << "min_hue" << white_param.min_hue;
    fs << "max_hue" << white_param.max_hue;
    fs.release();
}

void Lut_tool::modifyTableRange(int flag)
{
    cv::Mat temp;
    cv::FileStorage fs(lut_path, cv::FileStorage::READ);
    fs["color_table"] >> temp;
    fs.release();

    if (!temp.data)
        temp = cv::Mat::zeros(256, 256, CV_8UC1);

    for (int hue = white_param.min_hue; hue <= white_param.max_hue; hue++)
    {
        for (int sat = white_param.min_sat; sat <= white_param.max_sat; sat++)
        {
            temp.at<uchar>(hue, sat) = flag;
            color_table.at<uchar>(hue, sat) = flag;
        }
    }

    fs.open(lut_path, cv::FileStorage::WRITE);
    fs << "color_table" << temp;
    fs << "min_val" << white_param.min_val;
    fs << "max_val" << white_param.max_val;
    fs << "min_sat" << white_param.min_sat;
    fs << "max_sat" << white_param.max_sat;
    fs << "min_hue" << white_param.min_hue;
    fs << "max_hue" << white_param.max_hue;
    fs.release();
}


void Lut_tool::zeroTable()
{
    cv::Mat temp;
    cv::FileStorage fs(lut_path, cv::FileStorage::READ);
    fs["color_table"] >> temp;
    fs.release();

    if (!temp.data)
        temp = cv::Mat::zeros(256, 256, CV_8UC1);

    for (int i(0); i < hue_range; i++)
    {
        for (int j(0); j < sat_range; j++)
        {
            color_table.at<uchar>(i, j) = 0;
        }
    }

    fs.open(lut_path, cv::FileStorage::WRITE);
    fs << "color_table" << temp;
    fs << "min_val" << ball_param.min_val;
    fs << "max_val" << ball_param.max_val;
    fs << "min_sat" << ball_param.min_sat;
    fs << "max_sat" << ball_param.max_sat;
    fs.release();
}

void Lut_tool::getHist(const cv::Mat &ref, cv::MatND hist[3])
{

    cv::Mat ball_mask(cv::Mat::zeros(ref.size(), CV_8UC1));
    cv::Point center(ball_mask.cols / 2, ball_mask.rows / 2);
    cv::circle(ball_mask, center,
               (center.y < center.x) ? center.y : center.x, cv::Scalar(1), cv::FILLED);

    int hist_bin[1] = {32};
    float ranges_hsv[3][2] = {{0., 180.}, {0., 256.}, {0., 256.}};
    for (int i(0); i < 3; i++)
    {
        int chn[1] = {i};
        const float *range[1] = {ranges_hsv[i]};
        cv::calcHist(&ref, 1, chn, ball_mask,
                     hist[i], 1, hist_bin, range, true, false);
        cv::normalize(hist[i], hist[i], 0, 1, cv::NORM_MINMAX);
    }
}

cv::Mat Lut_tool::segmentField(const cv::Mat &_in_hsv)
{
    cv::Mat field_color(cv::Mat::zeros(_in_hsv.size(), CV_8UC1));

    for (auto i(0); i < _in_hsv.total(); i++)
    {
        int hue = _in_hsv.at<cv::Vec3b>(i)[0];
        int sat = _in_hsv.at<cv::Vec3b>(i)[1];
        int val = _in_hsv.at<cv::Vec3b>(i)[2];

        if (color_table.at<uchar>(hue * sat_range + sat) == 1)
            field_color.at<uchar>(i) = 255;
    }

    return field_color;
}

cv::Mat Lut_tool::segmentBall(const cv::Mat &_in_hsv)
{
    cv::Mat ball_color(cv::Mat::zeros(_in_hsv.size(), CV_8UC1));

    for (auto i(0); i < _in_hsv.total(); i++)
    {
        int sat = _in_hsv.at<cv::Vec3b>(i)[1];
        int val = _in_hsv.at<cv::Vec3b>(i)[2];

        if (val >= ball_param.min_val & sat >= ball_param.min_sat & val <= ball_param.max_val & sat <= ball_param.max_sat)
            ball_color.at<uchar>(i) = 255;
    }

    return ball_color;
}

cv::Mat Lut_tool::segmentOutput(cv::Mat &_in)
{
    cv::Mat _out = cv::Mat::zeros(frame_height, frame_width, CV_8UC3);
    for (int i = 0; i < frame_height; i++)
    {
        cv::Vec3b *in_hsv_ptr = _in.ptr<cv::Vec3b>(i);
        cv::Vec3b *out_segment_ptr = _out.ptr<cv::Vec3b>(i);
        for (int j = 0; j < frame_width; j++)
        {
            uchar pres_class = color_table.at<uchar>(in_hsv_ptr[j][0], in_hsv_ptr[j][1]);
            if (pres_class == 1)
            {
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 200;
                out_segment_ptr[j][2] = 0;
            }
            else if (pres_class == 2)
            {
                out_segment_ptr[j][0] = 0;
                out_segment_ptr[j][1] = 140;
                out_segment_ptr[j][2] = 255;
            }
            else if (pres_class == 3)
            {
                out_segment_ptr[j][0] = 255;
                out_segment_ptr[j][1] = 255;
                out_segment_ptr[j][2] = 255;
            }
        }
    }

    return _out;
}

cv::Mat Lut_tool::segmentWhite(const cv::Mat &_in_hsv)
{
    cv::Mat white_color(cv::Mat::zeros(_in_hsv.size(), CV_8UC1));

    for (auto i(0); i < _in_hsv.total(); i++)
    {
        int hue = _in_hsv.at<cv::Vec3b>(i)[0];
        int sat = _in_hsv.at<cv::Vec3b>(i)[1];
        int val = _in_hsv.at<cv::Vec3b>(i)[2];

        if (color_table.at<uchar>(hue * sat_range + sat) == 3)
            white_color.at<uchar>(i) = 255;
    }

    return white_color;
}

cv::Mat Lut_tool::makeAll(const cv::Mat tl, const cv::Mat tr, const cv::Mat bl, const cv::Mat br)
{
    cv::Mat all, temp;
    cv::hconcat(tl, tr, all);
    cv::hconcat(bl, br, temp);
    cv::vconcat(all, temp, all);
    return all;
}

void Lut_tool::drawText()
{
    cv::putText(all_in_one, "RGB", cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 0, 255), 2);
    cv::putText(all_in_one, "HSV", cv::Point(frame_width + 5, 15), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(255, 0, 0), 2);
    cv::putText(all_in_one, "OUTPUT", cv::Point(5, frame_height + 15), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 0, 255), 2);
    cv::putText(all_in_one, "RANGE", cv::Point(frame_width + 5, frame_height + 15), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 0, 255), 2);
}

void Lut_tool::getTrackbar()
{
    white_param.min_hue = cv::getTrackbarPos("White Min. Hue", param_winname);
    white_param.max_hue = cv::getTrackbarPos("White Max. Hue", param_winname);
    white_param.min_sat = cv::getTrackbarPos("White Min. Sat", param_winname);
    white_param.max_sat = cv::getTrackbarPos("White Max. Sat", param_winname);
}

void Lut_tool::imageCallback(const sensor_msgs::ImageConstPtr &_msg)
{
    try
    {
        if (_msg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0)
            this->img_encoding_ = npuvision::IMG_MONO;
        if (_msg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0)
            this->img_encoding_ = npuvision::IMG_BGR8;
        if (_msg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0)
            this->img_encoding_ = npuvision::IMG_RGB8;
        this->cv_img_ptr_subs_ = cv_bridge::toCvCopy(_msg, _msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("[robocup_detector] cv bridge exception: %s", e.what());
        return;
    }

    // this->stamp_ = _msg->header.stamp;
    // this->frame_id_ = _msg->header.frame_id;
}


void Lut_tool::process()
{
    // std::lock_guard<std::mutex> l(lock_);
    // std::cout << cycle_++ << std::endl;
    if (cv_img_ptr_subs_ == nullptr)
        return;

    input = cv_img_ptr_subs_->image;

    cv::resize(input, input, cv::Size(frame_width, frame_height));
    cv::cvtColor(input, input_hsv, cv::COLOR_BGR2HSV);

    // cv::Mat field_color(segmentField(input_hsv));
    // cv::Mat ball_color( segmentBall(input_hsv) );
    // cv::Mat white_color(segmentWhite(input_hsv));
    cv::Mat output(segmentOutput(input_hsv));

    cv::Mat temp;
    cv::Mat img_range;
    cv::inRange(input_hsv, cv::Scalar(white_param.min_hue, white_param.min_sat, 0),
                cv::Scalar(white_param.max_hue, white_param.max_sat, 255), img_range);

    cv::cvtColor(img_range, img_range, cv::COLOR_GRAY2BGR);

    all_in_one = makeAll(input, input_hsv, output, img_range);
    drawText();

    time_t now = time(0);
    char* dt = ctime(&now);
    std::cout << "New Frame, date: " << dt << std::endl;
    cv::imshow(winname, all_in_one);

    while (ros::ok())
    {
        cv::setMouseCallback(winname, mouse_call, 0);
        // update
        getTrackbar();
        cv::inRange(input_hsv, cv::Scalar(white_param.min_hue, white_param.min_sat, 0),
                    cv::Scalar(white_param.max_hue, white_param.max_sat, 255), img_range);
        cv::cvtColor(img_range, img_range, cv::COLOR_GRAY2BGR);
        all_in_one = makeAll(input, input_hsv, output, img_range);
        drawText();
        cv::imshow(winname, all_in_one);
        // condition
        int key = cv::waitKey(1) & 0xFF;
        if (key == 32)
            break;
    }
}