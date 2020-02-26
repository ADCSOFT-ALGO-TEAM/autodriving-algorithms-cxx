#include "conver2DTo3D.h"
#include "SimulationLaneSegmentation.h"
#include <opencv2/opencv.hpp>


int main(int argc, const char** argv) {
    (void) argc;
    (void) argv;

    pch::Simulater lane_seg(640, 320);
    lane_seg.GenerateLaneSegmentation();
    std::vector<int> labels;

    cv::Mat img = lane_seg.GetLaneSegmentation(labels);
    cv::Mat vision_lane = lane_seg.GetLaneVison();
    std::vector<std::vector<cv::Point3f>> result;
    cv::FileStorage f("/home/pengccheng/Data/work/location_t/interalMatrix_.yaml", cv::FileStorage::READ);
    cv::imshow("vision", vision_lane);
    cv::waitKey(0);
    std::string calbration_file = "/home/pengccheng/Data/work/location_t/interalMatrix_.yaml";

    cv::Mat projective_mat = (cv::Mat_<double>(3,3)<<-2.02430528e-01, +1.18949238e-02,  +2.06928941e+02,
                                                    +2.03825224e-02, -4.31747500e-01,  +5.63260609e+01,
                                                    -5.38963184e-05, -1.49079365e-03,  +1.00000000e+00);

    pch::ConverImage2DToCamera3D convert( calbration_file);

    convert.DrivingAreaDetection(img, labels, result);
    std::cout<<"RESULT:"<<std::endl;
    // for(std::vector<cv::Point3f> &pts : result)
    // {
    //     for(cv::Point3f pt : pts)
    //     {
    //         std::cout<<"("<<pt.x<<" , "<<pt.y<<" , "<<pt.z<<") ";
    //     }
    //     std::cout<<std::endl<<std::endl;
    // }
    
    return 0;
}