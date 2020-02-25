/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.24
 *  \attention 
 *
*/

#include "calibration.h"
#include <fstream>

namespace pch
{

void CalibrationExter::FindChessboardCorners()
{
    std::ifstream read_f(images_path_);
    if(!read_f.is_open())
    {
        std::cout<<"imgaes path file is open fail!"<<std::endl;
        return;
    }

    std::cout<<"Start to extract corners from images, Please wait... "<<std::endl;
    std::string image_path;
    cv::Mat img;
    std::vector<cv::Point2f> corners_point;
    bool frist_image = true;
    while (getline(read_f , image_path))
    {
        img = cv::imread(image_path);
        cv::cvtColor(img, img, CV_BGR2GRAY);
        if(frist_image)
        {
            image_size_.height = img.rows;
            image_size_.width = img.cols;
        }
        corners_point.clear();
        if(!cv::findChessboardCorners(img, board_size_, corners_point))
        {
            std::cout<<"The image("<<image_path<<") "<<"can not find chess board cooner"<<std::endl;
            continue;
        }
        else
        {
            /**
             * 
             * cv::cornerSubPix(Srcimg , cornersPointBuf , cv::Size(5,5),cv::Size(-1,-1),
					cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
            */
            cv::cornerSubPix(img, corners_point, cv::Size(5,5), cv::Size(-1,-1),
                            cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
            all_images_board_corner_on_2D_.emplace_back(corners_point);
        }

    }

    read_f.close();
    std::ifstream f_read2(images_on_ground_path_);
    if(! f_read2.is_open())
    {
        std::cout<<"images on ground path("<<images_on_ground_path_<<"), can not open!"<<std::endl;
        return ;
    }

    while (getline(f_read2, image_path))
    {
        img = cv::imread(images_path_);
        cv::cvtColor(img, img, CV_BGR2GRAY);
        corners_point.clear();
        if(!cv::findChessboardCorners(img, board_size_, corners_point))
        {
            std::cout<<"The image("<<image_path<<") "<<"can not find chess board cooner"<<std::endl;
            continue;
        }
        else
        {
            cv::cornerSubPix(img, corners_point, cv::Size(5,5), cv::Size(-1, -1),
                            cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            all_images_on_ground_board_corner_2D.emplace_back(corners_point);
        }
        
    }
    f_read2.close();
    
    
}

void CalibrationExter::GetCameraMatrixAndPlaneFactor(cv::Mat &camera_matrix, cv::Mat &dist_coeffs, std::vector<float> &plane)
{
    plane.clear();
    std::vector<cv::Point3f> pt_3D;
    bool success = false;
    cv::Mat plane_mat;
    GetCameraMatrix(camera_matrix, dist_coeffs);
    BackProjection(camera_matrix, dist_coeffs, pt_3D);
    FitGround(pt_3D, plane_mat, success);
    plane.emplace_back(plane_mat.at<float>(0, 0));
    plane.emplace_back(plane_mat.at<float>(1, 0));
    plane.emplace_back(plane_mat.at<float>(2, 0));
}

void CalibrationExter::FitGround(const std::vector<cv::Point3f> &ground_pts, cv::Mat &ground, bool &success)
{
    size_t count_pts = ground_pts.size();

    float x_square = 0.;
    float x_y      = 0.;
    float x_sum    = 0.;
    float y_square = 0.;
    float y_sum    = 0.;
    float n        = 0.;


    float x_z      = 0.;
    float y_z      = 0.;
    float z_sum    = 0.;

    n += count_pts;

    for(size_t i = 0; i < count_pts; i ++)
    {
        x_square += pow(ground_pts[i].x, 2);
        x_y      += ground_pts[i].x * ground_pts[i].y;
        x_sum    += ground_pts[i].x;
        y_square += pow(ground_pts[i].y, 2.0);
        y_sum    += ground_pts[i].y;

        x_z      += ground_pts[i].x * ground_pts[i].z;
        y_z      += ground_pts[i].y * ground_pts[i].z;
        z_sum    += ground_pts[i].z;
    }

    cv::Mat EquaMat_L = (cv::Mat_<float>(3,3)<< 
                        x_square, x_y,      x_sum,
                        x_y,      y_square, y_sum, 
                        x_sum,    y_sum,    n);
    cv::Mat EquaMat_R = (cv::Mat_<float>(3,1)<<
                        x_z, y_z, z_sum);

    
    if(cv::solve(EquaMat_L, EquaMat_R, ground, cv::DECOMP_SVD))
    {
        success = true;
    }
    else
    {
        success = false;
    }
    
}

void CalibrationExter::BackProjection(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Point3f> point3Ds)
{
    std::vector<cv::Point2f> project_points;
    point3Ds.clear();
    for(size_t index = 0; index < all_images_on_ground_board_corner_2D.size(); index++)
    {
        cv::Mat tvecsMat;
        cv::Mat rvecsMat;

        cv::solvePnP(all_images_on_ground_board_corner_3D[index],all_images_on_ground_board_corner_2D[index],
                     camera_matrix, dist_coeffs, rvecsMat, tvecsMat, true, CV_DLS);
        project_points.clear();
        cv::projectPoints(all_images_on_ground_board_corner_3D[index], rvecsMat, tvecsMat, camera_matrix, dist_coeffs, project_points);

        double error = cv::norm(project_points, all_images_on_ground_board_corner_2D, cv::NORM_L2);
        if(error < 12.0)
        {
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecsMat, rotation_matrix);
            cv::Mat pt;
            for(int corner_count = 0; corner_count<(board_size_.height * board_size_.width); corner_count++)
            {
                pt = (cv::Mat_<float>(3, 1)<<all_images_on_ground_board_corner_3D[index][corner_count].x, all_images_on_ground_board_corner_3D[index][corner_count].x, all_images_on_ground_board_corner_3D[index][corner_count].z );

                cv::Mat pt_3D = rotation_matrix * pt + tvecsMat;
                cv::Point3f point3D;
                float* pt_ptr =  pt_3D.ptr<float>();
                point3D.x  = *pt_ptr;
                pt_ptr++;
                point3D.y = *pt_ptr;
                pt_ptr++;
                point3D.z = *pt_ptr;
                point3Ds.emplace_back(point3D);
            }
        }

    }
}

void CalibrationExter::GetCameraMatrix(cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
{
    FindChessboardCorners();
    ConverBoardCornerFrom2DTo3D();
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    cv::calibrateCamera(all_images_board_corner_on_3D_, all_images_board_corner_on_2D_, image_size_, camera_matrix, dist_coeffs, rvecs, tvecs, 0);
    double error_avg = 0.;
    double error_cumulative = 0.;
    CalibrationError(camera_matrix, dist_coeffs, rvecs, tvecs, error_cumulative, error_avg);
    std::cout<<"Calibration Error: cumulative error: "<<error_cumulative<<" avg error: "<<error_avg<<std::endl;
}

void CalibrationExter::CalibrationError(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs, double &cumulative_error, double &avg_error)
{
    avg_error = 0.0;
    cumulative_error = 0.0;
    for(size_t i = 0; i < all_images_board_corner_on_2D_.size(); i++)
    {
        std::vector<cv::Point2f> dist_error;
        cv::projectPoints(all_images_board_corner_on_3D_[i],rvecs[i],tvecs[i],camera_matrix,dist_coeffs,dist_error);
        double error = cv::norm(dist_error, all_images_board_corner_on_2D_[i], cv::NORM_L2);
        error /= dist_error.size();
        cumulative_error += error;
    }
    avg_error = cumulative_error / all_images_board_corner_on_2D_.size();
}

void CalibrationExter::ConverBoardCornerFrom2DTo3D()
{
    //for every image, the chess board corners coordinate is all the same
    std::vector<cv::Point3f> chess_board_corner_3d;
    for(int h = 0; h < board_size_.height; h++)
    {
        for(int w = 0; w < board_size_.width; w++)
        {
            cv::Point3f pt;
            pt.x = h * grid_size_.height;
            pt.y = w * grid_size_.width;
            pt.z = 0.;
            chess_board_corner_3d.emplace_back(pt);
        }
    }

    all_images_board_corner_on_3D_.clear();
    for(size_t index = 0; index < all_images_board_corner_on_2D_.size(); index++)
    {
        all_images_board_corner_on_3D_.emplace_back(chess_board_corner_3d);
    }

    all_images_on_ground_board_corner_3D.clear();
    for(size_t index =0; index < all_images_on_ground_board_corner_2D.size(); index++)
    {
        all_images_on_ground_board_corner_3D.emplace_back(chess_board_corner_3d);
    }
}

CalibrationExter::CalibrationExter()
{}

CalibrationExter::CalibrationExter(const std::string &images_on_ground_path, const std::string &images_path, const cv::Size2i &board_szie, const cv::Size2d &grid_size):
                images_path_(images_path), images_on_ground_path_(images_on_ground_path), board_size_(board_szie), grid_size_(grid_size)
{  
}

}