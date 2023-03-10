//
// Created by xiang on 2021/9/9.
//

#include <opencv2/opencv.hpp>

// 文件路径，如果不对，请调整
std::string input_file = "../fisheye.jpg";

int main(int argc, char **argv) {
  // 本程序实现鱼眼的等距投影去畸变模型
  // 畸变参数（本例设为零）
  double k1 = 0, k2 = 0, k3 = 0, k4 = 0;

  // 内参
  double fx = 689.21, fy = 690.48, cx = 1295.56, cy = 942.17;

  cv::Mat image = cv::imread(input_file);
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC3); // 去畸变以后的图

  // 计算去畸变后图像的内容
  for (int v = 0; v < rows; v++)
    for (int u = 0; u < cols; u++) {

      double u_distorted = 0, v_distorted = 0;
      // TODO 按照公式，计算点(u,v)对应到畸变图像中的坐标(u_distorted,
      // v_distorted) (~6 lines)

      // start your code here
      double a = (u-cx)/fx, b = (v-cy)/fy;
      double r = sqrt(a*a + b*b);
      double theta = atan(r);
      double theta_2 = theta*theta;
      double theta_4 = theta_2*theta_2;
      double theta_6 = theta_4*theta_2;
      double theta_8 = theta_4*theta_4;
      double theta_d = theta*(1+k1*theta_2+k2*theta_4+k3*theta_6+k4*theta_8);

      double x_distorted = (theta_d / r)*a;
      double y_distorted = (theta_d / r)*b;

      u_distorted = fx*(x_distorted + 0.01*y_distorted)+cx;
      v_distorted = fy*y_distorted + cy;

      // end your code here

      // 赋值 (最近邻插值)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols &&
          v_distorted < rows) {
        image_undistort.at<cv::Vec3b>(v, u) =
            image.at<cv::Vec3b>((int)v_distorted, (int)u_distorted);
      } else {
        image_undistort.at<cv::Vec3b>(v, u) = 0;
      }
    }

  // 画图去畸变后图像
  cv::imshow("image undistorted", image_undistort);
  cv::imwrite("fisheye_undist.jpg", image_undistort);
  cv::waitKey();

  return 0;
}
