/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.11
 *  \attention 
 *
*/
#include "conver2DTo3D.h"


namespace pch
{
    void ConverImage2DToCamera3D::ExtracFact()
    {
        //std::cout<<"matrix: "<<camera_matrix_<<std::endl;
        if(6 == camera_matrix_.type())
        {
            fu_ = camera_matrix_.at<double>(0,0);
            u0_ = camera_matrix_.at<double>(0,2);
            fv_ = camera_matrix_.at<double>(1,1);
            v0_ = camera_matrix_.at<double>(1,2);

            a0_ = ground_plane_.at<double>(0,0);
            a1_ = ground_plane_.at<double>(1,0);
            a2_ = ground_plane_.at<double>(2,0);
        }
        else
        {
            fu_ = camera_matrix_.at<float>(0,0);
            u0_ = camera_matrix_.at<float>(0,2);
            fv_ = camera_matrix_.at<float>(1,1);
            v0_ = camera_matrix_.at<float>(1,2);

            a0_ = ground_plane_.at<float>(0,0);
            a1_ = ground_plane_.at<float>(1,0);
            a2_ = ground_plane_.at<float>(2,0);
        }
        
    }
    ConverImage2DToCamera3D::ConverImage2DToCamera3D(const std::string &calbration_file)
    {
        camera_pose_X_ = 0; 
        camera_pose_Y_ = 0;
        camera_pose_Z_ = 0;
        cv::FileStorage f(calbration_file, cv::FileStorage::READ);
        f["camera_matrix"]>>camera_matrix_;
        f["dist_coeffs"]>>dist_coeffs_;
        f["ground_fac"]>>ground_plane_;
        f.release();
        ExtracFact();
        use_projective_ = false;
    }

    ConverImage2DToCamera3D::ConverImage2DToCamera3D(const std::string &calbration_file, const double camera_pose_x,
                                                     const double camera_pose_y, const double camera_pose_z):
                                                     camera_pose_X_(camera_pose_x), camera_pose_Y_(camera_pose_y), camera_pose_Z_(camera_pose_z)
    {
        cv::FileStorage f(calbration_file, cv::FileStorage::READ);
        f["camera_matrix"]>>camera_matrix_;
        f["dist_coeffs"]>>dist_coeffs_;
        f["ground_fac"]>>ground_plane_;
        f.release();
        ExtracFact();
        use_projective_ = false;
    }

    ConverImage2DToCamera3D::ConverImage2DToCamera3D(const cv::Mat &projective_mat, const std::string &calbration_file):projective_matrix_(projective_mat)
    {
        camera_pose_X_ = 0;
        camera_pose_Y_ = 0;
        camera_pose_Z_ = 0;
        cv::FileStorage f(calbration_file, cv::FileStorage::READ);
        f["camera_matrix"]>>camera_matrix_;
        f["dist_coeffs"]>>dist_coeffs_;
        f["ground_fac"]>>ground_plane_;
        f.release();
        ExtracFact();
        use_projective_ = true;
    }

    ConverImage2DToCamera3D::ConverImage2DToCamera3D(const cv::Mat &projective_mat, const std::string &calbration_file,const double camera_pose_x,
                                     const double camera_pose_y, const double camera_pose_z):
                                     projective_matrix_(projective_mat), camera_pose_X_(camera_pose_x),
                                     camera_pose_Y_(camera_pose_y), camera_pose_Z_(camera_pose_z)
                                     {
                                        cv::FileStorage f(calbration_file, cv::FileStorage::READ);
                                        f["camera_matrix"]>>camera_matrix_;
                                        f["dist_coeffs"]>>dist_coeffs_;
                                        f["ground_fac"]>>ground_plane_;
                                        f.release();
                                        ExtracFact();
                                        use_projective_ = true;
                                     }

    void ConverImage2DToCamera3D::Convert(const std::vector<int> &image_point, std::vector<double> &point_3D)
    {
        // (void) image_point;
        // (void) point_3D;
	point_3D.clear();
        if(use_projective_)
        {
            bool change = false;
	    cv::Mat point_3d;
	    cv::Mat point;
            std::vector<int> point_t;
            CheckImagePoints(image_point, point_t, change);
            if(change)
            {
                point = (cv::Mat_<double>(3,1)<<point_t[0],point_t[1], point_t[2]);
            }
            else
            {
                point = (cv::Mat_<double>(3,1)<<image_point[0], image_point[1], image_point[2]);
            }


            point_3d = projective_matrix_ * point;//3*3 x 3*1 = 3*1

            assert(point_3d.at<double>(2,0) != 0);

            point_3D.clear();
            point_3D.push_back( point_3d.at<double>(0,0) / point_3d.at<double>(2,0) + camera_pose_X_);
            point_3D.push_back( point_3d.at<double>(1,0) / point_3d.at<double>(2,0) + camera_pose_Y_);
            point_3D.push_back(1 + camera_pose_Z_);
        }
        else
        {
            float x_3d = (image_point[0] - u0_) / fu_;
            float y_3d = (image_point[1] - v0_) / fv_;
            double temp = 1 - a0_ * x_3d - a1_ * y_3d;
            double t  = a2_ / temp -1;
	    double x = (1+t) * x_3d;
	    double y = (1+t) * y_3d;
	    double z = (1+t);
            point_3D.emplace_back(x);
            point_3D.emplace_back(y);
            point_3D.emplace_back(z);
        //    std::cout<<"2D x: "<<image_point[0]<<" y: "<<image_point[1]<<std::endl<<" 3d: x: "<<(1 + t) * x_3d<<
        //    " Y: "<<(1 + t) * y_3d<<" Z: "<<(1 + t)<<std::endl<<" x_3d: "<<x_3d<<" y_3d: "<<y_3d<<std::endl;
        }

	//std::cout<<"3D Point: x "<<point_3D[0]<<" y "<<point_3D[1]<<" z "<<point_3D[2]<<std::endl;
        

    }
    void ConverImage2DToCamera3D::Convert(const std::vector<cv::Point> &image_points, std::vector<cv::Point3f> &points_3d)
    {
        points_3d.clear();
        std::vector<double> point_3d_t;
        for(cv::Point pt : image_points)
        {

        std::vector<int> point_2d;
        cv::Point3f point_3d;
        point_2d.emplace_back(pt.x);
        point_2d.emplace_back(pt.y);
        point_2d.emplace_back(1);
       
        Convert(point_2d, point_3d_t);
        point_3d.x = point_3d_t[0];
        
        point_3d.y = point_3d_t[1];
        point_3d.z = point_3d_t[2];
        points_3d.emplace_back(point_3d);
        }

    }
    void ConverImage2DToCamera3D::Convert(const std::vector<std::vector<int> > &image_points, 
                                         std::vector<std::vector<double> > &points_3D)
    {

        points_3D.clear();
        std::vector<double> point_3d_t;
        for( std::vector<int> image_point : image_points)
        {
            Convert(image_point, point_3d_t);
            points_3D.emplace_back(point_3d_t);
        }
    }

    void ConverImage2DToCamera3D::Convert(const cv::Mat_<int> &image_points, cv::Mat &points_3D)
    {
        // (void) points_3D;
        // (void) image_points;
        cv::Mat points_2d;
        assert(image_points.rows <= 3);
        assert(image_points.rows > 1);
        if(image_points.rows ==2)
        {
            
            points_2d = cv::Mat_<double>(3, image_points.cols);
            for(int i = 0; i < image_points.cols; i++)
            {
                points_2d.at<double>(0, i) = image_points.at<int>(0,i);
                points_2d.at<double>(1, i) = image_points.at<int>(1,i);
                points_2d.at<double>(3, i) = 1.0;

            }

        }
        else
        {
            assert(image_points.at<int>(3,0) == 1);
            image_points.convertTo(points_2d, CV_64FC1);
        }
        cv::Mat x = cv::Mat_<double>::ones(1, image_points.cols) * camera_pose_X_;
        cv::Mat y = cv::Mat_<double>::ones(1, image_points.cols) * camera_pose_Y_;
        cv::Mat z = cv::Mat_<double>::ones(1, image_points.cols) * camera_pose_Z_;
        
        cv::Mat T = cv::Mat_<double>::zeros(3, image_points.cols);
        x.copyTo(T.row(0));
        y.copyTo(T.row(1));
        z.copyTo(T.row(2));
       points_3D = (projective_matrix_ * image_points) + T; // 3*3 x 3*n = 3*n

    }

    void ConverImage2DToCamera3D::CheckImagePoints(const std::vector<int> &image_point, std::vector<int> &point, bool &change)
    {
        if(2 == image_point.size())
        {
            change = true;
            point.emplace_back(image_point[0]);
            point.emplace_back(image_point[1]);
            point.emplace_back(1);
        }
        else
        {
            change = false;
        }
        
    }

    void ConverImage2DToCamera3D::DrivingAreaDetection(const cv::Mat &segmentation_res, const std::vector<int> &lables, std::vector<std::vector<cv::Point3f>> &driving_area_3d)
    {
    #ifdef DEBUG
        cv::Mat debug_img = cv::Mat::ones(segmentation_res.size(), CV_8UC3);
    #endif // DEBUG
       cv::Mat undistort_image;// = segmentation_res;
       cv::undistort(segmentation_res, undistort_image, camera_matrix_, dist_coeffs_);
        for(size_t i = 0; i < lables.size(); i++)
        {
            int label = lables[i];
            cv::Mat binary_img;
            std::vector<std::vector<cv::Point>> contous;
            cv::compare(undistort_image, label, binary_img, cv::CMP_EQ);
            cv::findContours(binary_img, contous, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
            
            #ifdef DEBUG
            cv::drawContours(debug_img,contous , -1, cv::Scalar::all(255));
            cv::imshow("DEBUG_CONTOUS", debug_img);
            cv::waitKey(0);
            #endif // DEBUG
            std::vector<cv::Point3f> area_contous_3d;
            if(0 == contous.size()) continue;
            Convert(contous[0], area_contous_3d);
            driving_area_3d.emplace_back(area_contous_3d);
        }
    }
}
