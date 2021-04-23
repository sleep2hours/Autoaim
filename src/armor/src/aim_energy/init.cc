#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    windMill::windMill(bool blue)
    {
        _isblue = blue;
        cnt = clockwize = anticlockwize = 0;
        KF.init(4, 2, 0);
        KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                               0, 1, 0, 1,
                               0, 0, 1, 0,
                               0, 0, 0, 1); //H
        measurement = cv::Mat::zeros(2, 1, CV_32F);
        setIdentity(KF.measurementMatrix);
        setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-2));      //Q
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-5));  //R
        setIdentity(KF.errorCovPost, cv::Scalar::all(1));            //P
        randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(1)); //init position
    }

    windMill::~windMill() { ; }
}
