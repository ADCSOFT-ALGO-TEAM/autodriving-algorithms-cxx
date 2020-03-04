/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.11
 *  \attention 
 *
*/
#include <vector>
// #include <opencv2/opencv.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/calib3d.hpp>
#include <string>

#define USE_OPENCV4


namespace pch
{

class CalibrationExter
{

public:
    CalibrationExter();
    CalibrationExter(const std::string &images_on_ground_path, const std::string &images_path, const cv::Size2i &board_size, const cv::Size2d &grid_size);
    void GetCameraMatrix(cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
    void GetCameraMatrixAndPlaneFactor(cv::Mat &camera_matrix, cv::Mat &dist_coeffs, std::vector<float> &plane);
    static void OnMouseCallback(int event, int x, int y, int flag, void* param);
private:
    void FitGround(const std::vector<cv::Point3f> &ground_pts, cv::Mat &gound, bool &success);
    void FindChessboardCorners();
    void ConverBoardCornerFrom2DTo3D();
    void BackProjection(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff, std::vector<cv::Point3f> &point3D);
    void CalibrationError(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, double &cumulative_error, double &avg_error);
    

private:
    std::string images_path_;
    std::string images_on_ground_path_;
    cv::Size2i board_size_;
    cv::Size2d grid_size_;
    cv::Size image_size_;;
    std::vector< std::vector< cv::Point2f> > all_images_board_corner_on_2D_;
    std::vector <std::vector< cv::Point3f> > all_images_board_corner_on_3D_;
    std::vector <std::vector< cv::Point2f> > all_images_on_ground_board_corner_2D;
    std::vector <std::vector< cv::Point3f> > all_images_on_ground_board_corner_3D;    
};
    
} // namespace pch
