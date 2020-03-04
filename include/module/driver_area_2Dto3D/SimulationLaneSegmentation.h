/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.14
 *  \attention 
 * 
*/

#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#ifndef SIMULATION_LANR_SEGMENTATION_H__
#define SIMULATION_LANR_SEGMENTATION_H__


namespace pch
{


class Simulater
{
public:
     explicit Simulater(const size_t width, const size_t height);
     void GenerateLaneSegmentation();
     const cv::Mat& GetLaneSegmentation(std::vector<int>  &lables);
     const cv::Mat& GetLaneVison();
private:
    cv::Mat lane_segmentation_;
    cv::Mat lane_vison_;
    size_t width_;
    size_t height_;
    std::vector<std::vector<std::vector<cv::Point> > > lanes_segmentation_contous;
    std::vector<int> lables_;
    std::vector<cv::Scalar> colors_;

private:
    void ComputeLaneContous();

};
}
#endif // DEBUG