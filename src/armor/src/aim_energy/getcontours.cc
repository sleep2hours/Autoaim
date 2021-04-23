#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    bool windMill::GetContours(cv::Mat &img) //获得装甲板,R,叶片
    {
        std::vector<cv::Mat> channels;
        cv::split(img, channels);
        cv::Mat binary;
        if (_isblue)
        {
            binary = channels[0] - channels[2];
        }
        else
        {
            binary = channels[2] - channels[0];
        }
        cv::dilate(binary, binary, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
        cv::threshold(binary, binary, 90, 255, CV_THRESH_BINARY);

        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, fin_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        cv::findContours(binary, R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
#ifdef debug_show
        // cv::imshow("binary", binary);
#endif
        if (FindValidFinArmor(fin_contours, hierarchy))
        {
            // cv::circle(src, cv::Point(0, 0), 23, cv::Scalar(255, 0, 0), -1);
            if (FindValidR(R_contours))
            {
                // cv::circle(src,center,175,cv::Scalar(123,34,234),1);
                return true;
            }
            else
            {
                // std::cout<<"Find R Failed"<<std::endl;
                return false;
            }
        }
        else
        {
            // std::cout<<"Find Armor Failed"<<std::endl;
            return false;
        }
    }
}