/**====================Decision Module===================//
 * @author:GQR
 * @date: 5/2/2020
 * @Editing date:6/2/2020
 * @last modification: made by HQY to fit for his codes
 * @latest modification date: 7/2/2020
*/
#ifndef ARMOR_DECISION_HPP
#define ARMOR_DECISION_HPP

#include <cstdlib>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "../aim_deps/AimDeps.hpp"
#ifdef _DEBUG_DECISION
#include <fstream>
#include <sstream>
#endif
    static std::vector<aim_deps::Armor> _input_armors;
class PnPdecision{
private:
     //std::vector<aim_deps::Evaluated_armor> _All_evaluated_armors;     //Store all the evaluated armor
     aim_deps::PnP_depended_param *_params=&aim_deps::pnp_depended_param;  

     std::vector<float> _All_armors_point;  
public:

     PnPdecision(){
     }

     /// @param spec 指定装甲板数字进行决策,如果发生指定，一般是确定了车号，需要确定同一辆车的最优
     /// @param exc 指定装甲板不会被列入到决策对象中
     /// @param mat 在调整参数时输入的一个图像
     /// @return the index of the best Armor input
     int Armor_decision(const std::vector<aim_deps::Armor> &Input_armors,
          int spec = 0, int exc = 0, cv::Mat Input_mat = cv::Mat()
     );
     #ifdef _DEBUG_DECISION
     void Draw_and_output( std::vector<aim_deps::Armor> Input_armors,cv::Mat &src);
     static void on_mouse(int event,int x,int y,int flags,void *ustc);
     #endif
};

#endif //ARMOR_DECISION_HPP