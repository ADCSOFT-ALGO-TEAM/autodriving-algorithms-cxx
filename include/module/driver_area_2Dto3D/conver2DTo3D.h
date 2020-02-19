/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.11
 *  \attention 
 * 
*/
#include <opencv2/opencv.hpp>
#include <vector>
namespace pch
{



class ConverImage2DToCamera3D
{

public:
    explicit ConverImage2DToCamera3D(const cv::Mat &projective_mat);
    explicit ConverImage2DToCamera3D(const cv::Mat &projective_mat, const double camera_pose_x,
                                     const double camera_pose_y, const double camera_z);

    void Convert(const std::vector<int> &image_point, std::vector<double> &point_3d);
    void Convert(const std::vector<cv::Point> &image_points, std::vector<cv::Point3f> &point_3d);
    void Convert(const std::vector<std::vector<int> > &image_points, std::vector<std::vector<double> > &points_3D);
    void Convert(const cv::Mat_<int> &image_points, cv::Mat &points_3D);

    void DrivingAreaDetection(const cv::Mat &segmentation_res, const std::vector<int> &lables, std::vector<std::vector<cv::Point3f>> &driving_area_3d);                                

private:

    cv::Mat projective_matrix_;
    double camera_pose_X_ ;
    double camera_pose_Y_ ;
    double camera_pose_Z_ ;

private:
    void CheckImagePoints(const std::vector<int> &image_point, std::vector<int> &point, bool &change);

};

}