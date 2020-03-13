#include "conver2DTo3D.h"
#include "SimulationLaneSegmentation.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <mutex>
#include <fstream>

#include "rawpic.pb.h"
#include "modulecomm.h"

template <typename Scalar>
class BufferImage
{
public:
BufferImage(const size_t buffer_size)
{
    buffer_size_ = buffer_size;
}

bool GetItem(Scalar& item)
{
    if(buffer_.size()>0)
    {
        std::lock_guard<std::mutex> mutex_lock(lock_);
        item = buffer_.front();
        buffer_.pop();
        return true;
    }
    else
    {
        return false;
    }
    return  false;
}

void CacheItem(const Scalar& item)
{
    std::lock_guard<std::mutex> mutex_lock(lock_);
    if(buffer_.size() > buffer_size_)
    {
        buffer_.pop();
    }
    buffer_.push(item);

}
private:
    std::queue<Scalar> buffer_;
    size_t buffer_size_;
    std::mutex lock_;
};


BufferImage<cv::Mat> buffer(160);
BufferImage<cv::Mat> buffer2(160);

void Listenpic(const char * strdata,const unsigned int nSize,const unsigned int index,const QDateTime * dt,const char * strmemname)
{
    (void) dt;
    (void) strmemname;
    (void) index;
    if(nSize<1000)return;
    iv::vision::rawpic pic;

    if(false == pic.ParseFromArray(strdata, nSize))
    {
        std::cout<<"picview Listenpic fail."<<std::endl;
        return;
    }

    cv::Mat mat(pic.height(),pic.width(),pic.mattype());
    if(pic.type() == 1)
        memcpy(mat.data,pic.picdata().data(),mat.rows*mat.cols*mat.elemSize());
    else
    {
       mat.release();
       std::vector<unsigned char> buff(pic.picdata().data(),pic.picdata().data() + pic.picdata().size());
       mat = cv::imdecode(buff,cv::IMREAD_COLOR);
    }
    buffer.CacheItem(mat);
}

void Listenpic_show(const char * strdata,const unsigned int nSize,const unsigned int index,const QDateTime * dt,const char * strmemname)
{
    (void) dt;
    (void) strmemname;
    (void) index;
    if(nSize<1000)return;
    iv::vision::rawpic pic;

    if(false == pic.ParseFromArray(strdata, nSize))
    {
        std::cout<<"picview Listenpic fail."<<std::endl;
        return;
    }

    cv::Mat mat(pic.height(),pic.width(),pic.mattype());
    if(pic.type() == 1)
        memcpy(mat.data,pic.picdata().data(),mat.rows*mat.cols*mat.elemSize());
    else
    {
       mat.release();
       std::vector<unsigned char> buff(pic.picdata().data(),pic.picdata().data() + pic.picdata().size());
       mat = cv::imdecode(buff,cv::IMREAD_COLOR);
    }
    buffer2.CacheItem(mat);
}

int main(int argc, const char** argv) {
    (void) argc;
    (void) argv;
    void *subscrib_handler_ptr;
    subscrib_handler_ptr = iv::modulecomm::RegisterRecv("free_space", Listenpic);
    void *subscrib_handler_ptr1;
    subscrib_handler_ptr1 = iv::modulecomm::RegisterRecv("free_space_show", Listenpic_show);

    std::vector<int> labels;
    labels.push_back(1);
    labels.push_back(2);
    std::vector<std::vector<cv::Point3f>> result;
    std::string calbration_file = "/home/pengccheng/Data/work/ADC/autodriving-algorithms-cxx/params/calibration_driver.yaml";
    std::string save_dir = "/home/pengccheng/Data/work/ADC/autodriving-algorithms-cxx/result/";
    pch::ConverImage2DToCamera3D convert( calbration_file);
   
    int index = 1;

    std::ofstream out;

    while (1)
    {
       
        cv::Mat seg_label;
        if(buffer.GetItem(seg_label))
        {
            std::string free_spce_path = save_dir + std::to_string(index) + ".txt";
            std::string color_path = save_dir + std::to_string(index)+".png";
            out.open(free_spce_path, std::ios::trunc);
            result.clear();
            convert.DrivingAreaDetection(seg_label, labels, result);
            for(std::vector<cv::Point3f> pts : result)
            {
                out<<"space:"<<std::endl;
                for(cv::Point3f pt: pts)
                {
                    out<<pt.x<<" , "<<pt.y<<" , "<<pt.z<<std::endl;
                   // std::cout<<pt.x<<" , "<<pt.y<<" , "<<pt.z<<std::endl;
                }
            }
            out.close();
            cv::Mat color ;
            while(!buffer2.GetItem(color))
            {}
            cv::imwrite(color_path, color);
            index++;
        }
        else
        {
            //std::cout<<"could not get data"<<std::endl;
        }
        
    }
   
    return 0;
}
