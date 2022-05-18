
// wujs

#include "ros_lut_tool/lut_tool.hpp"

Lut_tool *lut_tool = NULL;

int main(int argc, char **argv) {
  ros::init(argc, argv, "lut_tool");

  lut_tool = new Lut_tool();

  ros::Rate loop_rate(30);

  while (ros::ok()) {
    lut_tool->process();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void mouse_call(int event, int x, int y, int flags, void *data) {
  if (event == cv::EVENT_LBUTTONDBLCLK & !lut_tool->mouse_stuff.left_down) {
    if (x >= frame_width & y < frame_height) {
      lut_tool->mouse_stuff.left_down = true;

      lut_tool->mouse_stuff.coor1.x =
          std::max(std::min(x, frame_width * 2 - 1), frame_width);
      lut_tool->mouse_stuff.coor1.y =
          std::max(std::min(y, frame_height - 1), 0);

      std::cout << "[lut_tool] Corner 1: " << lut_tool->mouse_stuff.coor1
                << std::endl;
    } else {
      std::cout << "[lut_tool] Crop in HSV area only!" << std::endl;
    }
  }

  if (event == cv::EVENT_LBUTTONDBLCLK & lut_tool->mouse_stuff.left_down) {
    if (std::abs(x - lut_tool->mouse_stuff.coor1.x) > 1 &
        std::abs(y - lut_tool->mouse_stuff.coor1.y) > 1) {
      lut_tool->mouse_stuff.left_up = true;

      lut_tool->mouse_stuff.coor2.x =
          std::max(std::min(x, frame_width * 2 - 1), frame_width);
      lut_tool->mouse_stuff.coor2.y =
          std::max(std::min(y, frame_height - 1), 0);

      std::cout << "[lut_tool] Corner 2: " << lut_tool->mouse_stuff.coor2
                << std::endl;
    } else {
      std::cout << "[lut_tool] Create a region more than 1x1!" << std::endl;
    }
  }

  if (lut_tool->mouse_stuff.left_down & !lut_tool->mouse_stuff.left_up) {
    cv::Point pt;
    pt.x = std::max(std::min(x, frame_width * 2 - 1), frame_width);
    pt.y = std::max(std::min(y, frame_height - 1), 0);

    cv::Mat temp(lut_tool->all_in_one.clone());
    cv::rectangle(temp, lut_tool->mouse_stuff.coor1, pt, cv::Scalar(0, 0, 255));
    cv::imshow(lut_tool->winname, temp);
  }

  if (lut_tool->mouse_stuff.left_down & lut_tool->mouse_stuff.left_up) {
    cv::Rect box;
    box.width =
        std::abs(lut_tool->mouse_stuff.coor1.x - lut_tool->mouse_stuff.coor2.x);
    box.height =
        std::abs(lut_tool->mouse_stuff.coor1.y - lut_tool->mouse_stuff.coor2.y);
    box.x =
        std::min(lut_tool->mouse_stuff.coor1.x, lut_tool->mouse_stuff.coor2.x);
    box.y =
        std::min(lut_tool->mouse_stuff.coor1.y, lut_tool->mouse_stuff.coor2.y);
    // convert the left-top window
    box.x -= frame_width;

    cv::Mat sel_roi(lut_tool->all_in_one, box);
    cv::resize(sel_roi, sel_roi, cv::Size(sel_roi.cols * 4, sel_roi.rows * 4), 0, 0,
           1); // INTER_LINEAR
    cv::imshow("[lut_tool] SELECTED", sel_roi);

    lut_tool->mouse_stuff.left_down = false;
    lut_tool->mouse_stuff.left_up = false;

    while (ros::ok()) {
      int key = cv::waitKey(5) & 0xFF;
      if (key == 'f') {
        std::cout << "[lut_tool] Field color tagged!" << std::endl;
        lut_tool->modifyTable(sel_roi, 1);
        break;
      } else if (key == 'b') {
        std::cout << "[lut_tool] Ball color tagged!." << std::endl;
        lut_tool->modifyTable(sel_roi, 2);
        break;
      } else if (key == 'w') {
        std::cout << "[lut_tool] Line color tagged!." << std::endl;
        lut_tool->modifyTable(sel_roi, 3);
        break;
      } else if (key == 'd') {
        std::cout << "[lut_tool] Color tag deleted!" << std::endl;
        lut_tool->modifyTable(sel_roi, 0);
        break;
      } else if (key == 'b') {
        std::cout << "[lut_tool]  color tags are all deleted!." << std::endl;
        break;
      } else if (key == 'h') {
        std::cout << "[lut_tool] Ball color histogram saved!" << std::endl;
        cv::imwrite("../data/ball_reference.jpeg", sel_roi);
        break;
      } else if (key == 's') {
        std::cout << "[lut_tool] img saved!" << std::endl;
        cv::imwrite("../temp.jpg", lut_tool->input);
        break;
      } else if (key == 'F') {
        std::cout << "[lut_tool] Field color tagged IN RANGE!" << std::endl;
        lut_tool->modifyTableRange(1);
        break;
      } else if (key == 'W') {
        std::cout << "[lut_tool] Line color tagged!." << std::endl;
        lut_tool->modifyTableRange(3);
        break;
      } else if (key == 32)
        break;
    }
    // cv::destroyWindow("[lut_tool] Selected ROI");
  }
}
