/**
 *  \brerf 
 *  \author pengcheng(pengcheng@adcsoft.cn)
 *  \date 2020.02.14
 *  \attention 
 * 
*/
#include "SimulationLaneSegmentation.h"
namespace pch
{
    Simulater::Simulater(const size_t width, const size_t height): width_(width), height_(height)
    {
        colors_.emplace_back(cv::Scalar(10, 255, 10));
        colors_.emplace_back(cv::Scalar(255, 15, 20));
        colors_.emplace_back(cv::Scalar(23, 13, 255));

    }

    void Simulater::ComputeLaneContous()
    {
        size_t start_x1 = width_ / 4;
        size_t start_x2 = width_ / 8;

        size_t start_y = height_ / 12;
        size_t end_y = height_ - height_ / 10;

        size_t x1_gap = width_ / 6;
        size_t x2_gap = width_ /4;

        std::vector<cv::Point> lane1;
        std::vector<cv::Point> lane2;
        std::vector<cv::Point> lane3;

        cv::Point pt1(start_x1, start_y);
        cv::Point pt2(start_x1 + x1_gap, start_y);
        cv::Point pt3(start_x1 + 2 * x1_gap, start_y);
        cv::Point pt4(start_x1 + 3 * x1_gap, start_y);

        cv::Point pt5(start_x2 , end_y);
        cv::Point pt6(start_x2 + x2_gap, end_y);
        cv::Point pt7(start_x2 + 2 * x2_gap, end_y);
        cv::Point pt8(start_x2 + 3 * x2_gap, end_y);

        lane1.emplace_back(pt1);
        lane1.emplace_back(pt2);
        lane1.emplace_back(pt6);
        lane1.emplace_back(pt5);
        std::vector<std::vector<cv::Point> > lane1_t;
        lane1_t.emplace_back(lane1);
        lanes_segmentation_contous.emplace_back(lane1_t);

        lane2.emplace_back(pt2);
        lane2.emplace_back(pt3);
        lane2.emplace_back(pt7);
        lane2.emplace_back(pt6);
        std::vector<std::vector<cv::Point> > lane2_t;
        lane2_t.emplace_back(lane2);
        lanes_segmentation_contous.emplace_back(lane2_t);

        lane3.emplace_back(pt3);
        lane3.emplace_back(pt4);
        lane3.emplace_back(pt8);
        lane3.emplace_back(pt7);
        std::vector<std::vector<cv::Point> > lane3_t;
        lane3_t.emplace_back(lane3);
        lanes_segmentation_contous.emplace_back(lane3_t);
        
    }

    void Simulater::GenerateLaneSegmentation()
    {
        lane_segmentation_ = cv::Mat::zeros(height_, width_, CV_8UC1);
        lane_vison_ = cv::Mat::zeros(height_, width_, CV_8UC3);
        lables_.clear();
        ComputeLaneContous();
        //std::cout<<lanes_segmentation_contous.size()<<std::endl;
        for(size_t i = 1; i < (lanes_segmentation_contous.size() +1); i ++)
        {
            cv::fillPoly(lane_segmentation_, lanes_segmentation_contous[i-1], cv::Scalar(i));
            cv::fillPoly(lane_vison_, lanes_segmentation_contous[i-1], colors_[i-1]);
            // cv::imshow("t", lane_segmentation_);
            // // std::cout<<lane_segmentation_<<std::endl;
            // cv::waitKey(0);
            lables_.emplace_back(i);
        }
    }
   
   const cv::Mat& Simulater::GetLaneSegmentation(std::vector<int>  &lables)
   {
       lables.clear(); 
       for(int label : lables_)
       {
           lables.emplace_back(label);
       }

       return lane_segmentation_;
   }

   const cv::Mat& Simulater::GetLaneVison()
   {
       return lane_vison_;
   }

}