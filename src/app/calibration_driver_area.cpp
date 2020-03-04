/**
 * @brief This program is used to calibrate the camera 
 *        internal parameters and ground equations in the free space
 * @author pengcheng(yslrpch@126.com)
 * @date 2020/02/25
*/

#include <calibration.h>
#include <string.h>
#include <cstdlib>

void help()
{
     std::cout<<"use page: "<<std::endl<<
        "   please use this program: ./calibration_driver_area  --calibration_camera calibration_images_file --calibration_ground calibration_borad_on_ground_file  --board_size 5 5 --grid_size 10 10"<<std::endl;
}

int main(int argc, const char** argv) {

    if(3 > argc)
    {
        help();
        return 0;
    }
    std::string calibration_camera_images_path_file;
    std::string calibration_ground_images_path_file;
    int board_width;
    int board_height;
    float grid_width;
    float grid_height;
    if(0 == strcmp("--calibration_camera", argv[1]))
    {
        calibration_camera_images_path_file  = std::string(argv[2]);
    }
    else
    {
        help();
        std::cout<<"calibration_camera fail"<<std::endl;
        return 0;
    }
    
    if(0 == strcmp("--calibration_ground", argv[3]))
    {
        calibration_ground_images_path_file = std::string(argv[4]);
    }
    else
    {
        help();
        std::cout<<"calibration_ground fail"<<std::endl;
        return 0;
    }

    if(0 == strcmp("--board_size", argv[5]))
    {
        board_height = std::atoi(argv[6]);
        board_width = std::atoi(argv[7]);
    }
    else
    {
        help();
        return 0;
    }
    
    if(0 == strcmp("--grid_size", argv[8]))
    {
        grid_height = std::atof(argv[9]);
        grid_width = std::atof(argv[10]);
    }
    else
    {
        help();
        return 0;
    }
    
    //create calibration

    //pch::CalibrationExter cal(calibration_camera_images_path_file, calibration_ground_images_path_file, cv::Size(board_height, board_width),cv::Size2d(grid_height, grid_width));
    pch::CalibrationExter cal(calibration_ground_images_path_file, calibration_camera_images_path_file, cv::Size(board_width, board_height),cv::Size2d(grid_height, grid_width));

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<float> palne;

    cal.GetCameraMatrixAndPlaneFactor(camera_matrix, dist_coeffs, palne);
    cv::Mat plane_mat = (cv::Mat_<float>(3,1)<<palne[0], palne[1], palne[2]);
    //save result 
    cv::FileStorage fs("calibration_driver.yaml", cv::FileStorage::WRITE);
    fs<<"camera_matrix"<<camera_matrix;
    fs<<"dist_coeffs"<<dist_coeffs;
    fs<<"ground_fac"<<plane_mat;
    fs.release();

    return 0;
}

