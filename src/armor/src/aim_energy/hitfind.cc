#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    bool windMill::FindHitPoint(cv::Mat &img)
    {
        if (GetContours(img))
        {
            hit_point = armors[0].center;
            // cv::circle(src,hit_point,5,cv::Scalar(0,255,0),-1);
            KF.predict();
            measurement.at<float>(0) = hit_point.x;
            measurement.at<float>(1) = hit_point.y;
            cv::Mat correction = KF.correct(measurement);
            hit_point = cv::Point(correction.at<float>(0), correction.at<float>(1));
            // cv::circle(src, hit_point, 3, cv::Scalar(255, 0, 0), -1);
            now_angle = CalAngle(hit_point, center);
            // std::cout<<angle<<std::endl;
            dangle = now_angle - last_angle;
            last_angle = now_angle;
        }
        else
            return false;
    }

}
