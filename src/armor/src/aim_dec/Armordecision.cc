#include "aim_dec/Armordecision.hpp"

int PnPdecision::Armor_decision(const std::vector<aim_deps::Armor> &Input_armors, int spec, int exc,cv::Mat Input_mat)
{
     _input_armors=Input_armors;
     #ifdef _DEBUG_DECISION
     Draw_and_output(Input_armors,Input_mat);
     #endif
     _All_armors_point.clear();
     for(auto &armor : Input_armors)
     {
          aim_deps::Evaluated_armor _eva_armor;
          //classify the armor
          switch (armor.armor_number)
               {
               case 1:
                    {
                    _eva_armor._type=aim_deps::Armor_type::Hero;
                    _eva_armor.Type_score=_params->Hero_score;
                    break;
                    }
               case 2:
                    {
                    _eva_armor._type=aim_deps::Armor_type::Dad;
                    _eva_armor.Type_score=_params->Dad_score;
                    break;
                    }
               case 3 :
                    {_eva_armor._type=aim_deps::Armor_type::Infantry;
                    _eva_armor.Type_score=_params->Infantry_score;
                    break;
                    }
               case 4 :
                    {_eva_armor._type=aim_deps::Armor_type::Infantry;
                    _eva_armor.Type_score=_params->Infantry_score;
                    break;
                    }
               case 5 :
                    {
                    _eva_armor._type=aim_deps::Armor_type::Infantry;
                    _eva_armor.Type_score=_params->Infantry_score;
                    break;
                    }
               case 8 :
                    {
                    _eva_armor._type=aim_deps::Armor_type::Base;
                    _eva_armor.Type_score=_params->Base_score;
                    break;
                    }
               default:
                    _eva_armor._type=aim_deps::Armor_type::None;
                    _eva_armor.Type_score=0;
                    break;
               }
          //calculate the scores 
          if((spec != 0 && armor.armor_number == spec) || spec == 0){
               if(armor.armor_number != exc){             //装甲板号等于exc将会被剔除
                    if(armor.valid){
                         #ifdef _DEBUG_DECISION
                         
                         #endif
                         _eva_armor.Distance_score = armor.t_vec.z * _params->Distant_base;
                         _eva_armor.Size_score = (armor.t_vec.y+armor.t_vec.x) * _params->Size_base;
                         _eva_armor.Rotation_score = (armor.r_vec.at<float>(0,0) + armor.r_vec.at<float>(1,0)) * _params->Rotation_base;
                         _eva_armor.Total_score = (std::abs(_eva_armor.Size_score) + _eva_armor.Distance_score) *
                         _params->Distance_multi + _eva_armor.Type_score;
                         _eva_armor._armor = armor;
                         _All_armors_point.emplace_back(_eva_armor.Total_score);
                         //std::cout <<_params->Distance_multi << std::endl;
                    }
                    else _All_armors_point.emplace_back(aim_deps::FAILED_SCORE);     //invalid装甲板分值为1023
               }
          }
     }
     #ifdef _DEBUG_DECISION
     return 0;            //return 0 here cuz you need to adjust decision manully
     #endif
     
     //get the index of the armor which has the highest totalscore
     auto minPosition = min_element(_All_armors_point.begin(), _All_armors_point.end());
     return minPosition - _All_armors_point.begin();
}
#ifdef _DEBUG_DECISION
void PnPdecision::Draw_and_output( std::vector<aim_deps::Armor> Input_armors,cv::Mat & src)
{
     char str[2];
     cvNamedWindow("Debug_decision");
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< Input_armors.size(); ++i) {
        if(Input_armors[i].armor_number != -1 && Input_armors[i].valid){   //有意义的数字
      //非最佳装甲板使用黄色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, Input_armors[i].vertex[j], 
                    Input_armors[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 255), 2);   
            }
            ///snprintf(str, 2, "%d", j);      //最佳装甲板位置x

	        ///cv::putText(src, str, tar_list[i].vertex[j]+cv::Point2f(2, 2),
	        ///    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 100, 255));
            //snprintf(str, 2, "%d", Input_armors[i].t_vec.z);
           // cv::putText(src, str, cv::Point2f(25.0, 10.0) + Input_armors[i].vertex[2],
          //cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
        }
    }
    //std::cout<<"debug"<<std::endl;
    if(!src.empty()){
     cv::imshow("Debug_decision",src);

     //_input_armors.assign(Input_armors.begin(),Input_armors.back());
     cv::setMouseCallback("Debug_decision",on_mouse,(void*)&Input_armors);
          cv::waitKey(0);
    }
}
void  PnPdecision::on_mouse(int event,int x,int y,int flags,void *ustc)
{
     std::vector<aim_deps::Armor>& Input_armors=*(std::vector<aim_deps::Armor>*)ustc;
     switch (event)
     {
     case cv::EVENT_LBUTTONDOWN:
          for(auto &armor:_input_armors )
          {
                    std::ofstream outFile;
	outFile.open("/home/bili/Autoaim/src/armor/data.csv",std::ios::app); // 打开模式可省略
               std::vector<cv::Point2f> vertec_in_vector(armor.vertex,armor.vertex+4);
               int optimal=0;
               if(cv::pointPolygonTest(vertec_in_vector,cv::Point(x,y),false)>0)
               {
                    optimal=1;
                    outFile<<armor.t_vec.x<<","<<armor.t_vec.y<<","<<armor.t_vec.z<<","<<armor.r_vec.at<float>(0,0)<<","<<armor.r_vec.at<float>(1,0)<<","<<optimal<<std::endl;
               }
          //printf("Out data is %f",&armor.t_vec.x);
          outFile.close();
          }
          break;
     case cv::EVENT_RBUTTONDOWN:
          for(auto &armor:_input_armors )
          {
                    std::ofstream outFile;
	outFile.open("/home/bili/Autoaim/src/armor/data.csv",std::ios::app); // 打开模式可省略
               std::vector<cv::Point2f> vertec_in_vector(armor.vertex,armor.vertex+4);
               int optimal=0;
                if(cv::pointPolygonTest(vertec_in_vector,cv::Point(x,y),false)>0)
               {
                    optimal=0;
                    outFile<<armor.t_vec.x<<","<<armor.t_vec.y<<","<<armor.t_vec.z<<","<<armor.r_vec.at<float>(0,0)<<","<<armor.r_vec.at<float>(1,0)<<","<<optimal<<std::endl;
               }
              outFile.close();
          }
          break;
     default:
          break;
     }
}
#endif