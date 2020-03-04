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
void CalibrationExter::OnMouseCallback(int event, int x, int y, int flag, void* param)
{
    (void) flag;
    std::vector<cv::Point2f>* corners_point_ptr = (std::vector<cv::Point2f>*) param;
    switch (event)
    {
    case cv::EVENT_LBUTTONDBLCLK :
        cv::Point2f pt;
        pt.x = x;
        pt.y = y;
        corners_point_ptr->emplace_back(pt);
    }
}

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
        #ifndef USE_OPENCV4
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        #endif // !1
        if(frist_image)
        {
            image_size_.height = img.rows;
            image_size_.width = img.cols;
        }
        corners_point.clear();
        #ifdef USE_OPENCV4
        if(!cv::findChessboardCornersSB(img, board_size_, corners_point, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))
        #else
        if(!cv::findChessboardCorners(img, board_size_, corners_point))
        #endif // USE_OPENCV4
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
            #ifdef USE_OPENCV4
            all_images_board_corner_on_2D_.emplace_back(corners_point);
            cv::drawChessboardCorners(img, board_size_, cv::Mat(corners_point), true);
            cv::imshow("chessboard", img);
            cv::waitKey(0);
            #else
            cv::cornerSubPix(img, corners_point, cv::Size(5,5), cv::Size(-1,-1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER,30,0.1));
            all_images_board_corner_on_2D_.emplace_back(corners_point);
            
            #endif // USE_OPENCV4
            
            std::cout<<"The image("<<image_path<<") "<<" find chess board cooner"<<std::endl;
        }

    }


    read_f.close();
    std::ifstream f_read2(images_on_ground_path_);
    if(! f_read2.is_open())
    {
        std::cout<<"images on ground path("<<images_on_ground_path_<<"), can not open!"<<std::endl;
        return ;
    }

    all_images_on_ground_board_corner_3D.clear();
    std::vector<cv::Point3f> corner_pt3D;
    while (getline(f_read2, image_path))
    {
        img = cv::imread(image_path);
       // cv::cvtColor(img, img, CV_BGR2GRAY);
        corners_point.clear();
        #ifdef USE_OPENCV4
        
        if(!cv::findChessboardCornersSB(img, board_size_, corners_point,  cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY))
        {
            std::cout<<"The image("<<image_path<<") "<<"can not find chess board cooner"<<std::endl;
            continue;
        }
        else
        {
            // cv::cornerSubPix(img, corners_point, cv::Size(5,5), cv::Size(-1, -1),
            //                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(img, board_size_, cv::Mat(corners_point),true);
            cv::imshow("chessboard", img);
            cv::waitKey(0);
            all_images_on_ground_board_corner_2D.emplace_back(corners_point);
            std::cout<<"The image("<<image_path<<") "<<"find chess board cooner"<<std::endl;
        }
        
        #else
        corner_pt3D.clear();
        ////////////////////
        ///Select points manually
        std::cout<<"select points manually, board row:"<<board_size_.height<<"  board col: "<<board_size_.width<<std::endl;
        cv::namedWindow("selectting");
        cv::imshow("selectting", img);
        cv::waitKey(10);
        cv::setMouseCallback("selectting", CalibrationExter::OnMouseCallback, &corners_point);
        cv::Mat temp_img;
        while (true)
        {
            temp_img = img.clone();
            for(cv::Point2f &pt: corners_point)
            {
                cv::circle(temp_img, cv::Point(pt.x, pt.y), 3, cv::Scalar(0, 0, 255), 0.5);
            }
            cv::imshow("selectting", temp_img);
            int key = cv::waitKey(10);
            // std::cout<<key<<std::endl;
            if(99 == key) corners_point.clear();
            if(98 == key) 
            {
                if(corners_point.size() > 0)  corners_point.pop_back();
            }
            if (27 == key) break;
        }
        for(size_t i = 0; (i < corners_point.size()); i++)
        {
            if(corners_point.size() < (size_t)(board_size_.height * board_size_.width))
            {
                size_t row = (i+1) / board_size_.height;
                size_t col = (i+1) % board_size_.width;
                float x = row * grid_size_.height;
                float y = col * grid_size_.width;
                cv::Point3f pt_3d(x, y, 0.0);
                corner_pt3D.emplace_back(pt_3d);
            }
        }
        if(corners_point.size() >0)
        {
            all_images_on_ground_board_corner_2D.emplace_back(corners_point);
            all_images_on_ground_board_corner_3D.emplace_back(corner_pt3D);
        }
        #endif // DEBUG
        ///////////////////
        
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
    std::cout<<"fit success: "<<success<<std::endl;
    std::cout<<"plane : "<<plane_mat<<std::endl;
    plane.emplace_back(plane_mat.at<float>(0, 0));
    plane.emplace_back(plane_mat.at<float>(1, 0));
    plane.emplace_back(plane_mat.at<float>(2, 0));
}

void CalibrationExter::FitGround(const std::vector<cv::Point3f> &ground_pts, cv::Mat &ground, bool &success)
{
    size_t count_pts = ground_pts.size();
    std::cout<<"ground_pts size: "<<count_pts<<std::endl;

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
    std::cout<<"EquaMat_L: "<<EquaMat_L<<"  EquaMat_R: "<<EquaMat_R<<std::endl;
    if(cv::solve(EquaMat_L, EquaMat_R, ground, cv::DECOMP_SVD))
    {
        success = true;
    }
    else
    {
        success = false;
    }
    
}

void CalibrationExter::BackProjection(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, std::vector<cv::Point3f> &point3Ds)
{
    std::vector<cv::Point2f> project_points;
    point3Ds.clear();
    for(size_t index = 0; index < all_images_on_ground_board_corner_2D.size(); index++)
    {
        cv::Mat tvecsMat;
        cv::Mat rvecsMat;
        std::cout<<"start solve PNP"<<std::endl;
        std::cout<<"camera_matrix: "<<camera_matrix<<std::endl;
        cv::solvePnP(all_images_on_ground_board_corner_3D[index],all_images_on_ground_board_corner_2D[index],
                     camera_matrix, dist_coeffs, rvecsMat, tvecsMat, true, cv::SOLVEPNP_DLS);
        project_points.clear();
        std::cout<<"start project "<<std::endl;
        cv::projectPoints(all_images_on_ground_board_corner_3D[index], rvecsMat, tvecsMat, camera_matrix, dist_coeffs, project_points);

        double error = cv::norm(project_points, all_images_on_ground_board_corner_2D[index], cv::NORM_L2);
        // std::cout<<"3d size : "<<all_images_on_ground_board_corner_3D[index].size()<<" 2d size: "<<all_images_on_ground_board_corner_2D[index].size()<<
        //         "project_points : "<<project_points.size()<<std::endl;
        std::cout<<"fit ground projection error: "<<error<<std::endl;

        std::cout<<std::endl<<std::endl<<std::endl;
        if(error < 12.0)
        {
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvecsMat, rotation_matrix);
            cv::Mat pt;
            for(int corner_count = 0; corner_count<(board_size_.height * board_size_.width); corner_count++)
            {
                pt = (cv::Mat_<double>(3, 1)<<all_images_on_ground_board_corner_3D[index][corner_count].x, all_images_on_ground_board_corner_3D[index][corner_count].x, all_images_on_ground_board_corner_3D[index][corner_count].z );
                cv::Mat pt_3D = rotation_matrix * pt + tvecsMat;
                cv::Point3f point3D;
                double* pt_ptr =  pt_3D.ptr<double>();
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
    #ifdef USE_OPENCV4
    all_images_on_ground_board_corner_3D.clear();
    for(size_t index =0; index < all_images_on_ground_board_corner_2D.size(); index++)
    {
        all_images_on_ground_board_corner_3D.emplace_back(chess_board_corner_3d);
    }
    #endif // DEBUG
    
}

CalibrationExter::CalibrationExter()
{}

CalibrationExter::CalibrationExter(const std::string &images_on_ground_path, const std::string &images_path, const cv::Size2i &board_szie, const cv::Size2d &grid_size):
                images_path_(images_path), images_on_ground_path_(images_on_ground_path), board_size_(board_szie), grid_size_(grid_size)
{  
}

}