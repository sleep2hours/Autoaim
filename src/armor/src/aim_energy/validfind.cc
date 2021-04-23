#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    bool windMill::FindValidFinArmor(std::vector<std::vector<cv::Point>> fin_contours, std::vector<cv::Vec4i> hierarchy) //筛选有效装甲板及叶片
    {
        for (int i = 0; i < fin_contours.size(); i++)
        {
            if (hierarchy[i][3] != -1 || hierarchy[i][2] == -1)
                continue;

            if (IsValidFin(fin_contours[i]))
            {
                if (IsValidArmor(fin_contours[hierarchy[i][2]]))
                {
                    fins.emplace_back(cv::minAreaRect(fin_contours[i]));
                    armors.emplace_back(cv::minAreaRect(fin_contours[hierarchy[i][2]]));
                }
            }
        }
        if (fins.size() != 1)
        {
            // std::cout << "fins_size:" << fins.size() << std::endl;
            return false;
        }

        fin_vec = armors[0].center - fins[0].center;

        return true;
    }

    bool windMill::FindValidR(std::vector<std::vector<cv::Point>> R_contours) //判断是否为有效R
    {
        for (int i = 0; i < R_contours.size(); i++)
        {
            if (IsValidR(R_contours[i]))
            {
                center_pt.emplace_back(cv::minAreaRect(R_contours[i]));
            }
            else
            {
                continue;
            }
        }
        if (center_pt.empty())
            return false;
        float lastangle = 0; //上一向量夹角
        for (int i = 0; i < center_pt.size(); i++)
        {
            cv::Point2f cen = center_pt[i].center - fins[0].center;
            double angle = CalVecAngle(cen, fin_vec);
            if (angle > lastangle)
            {
                lastangle = angle;
                center = center_pt[i].center;
            }
        }
        // cv::circle(src,center,180,cv::Scalar(12,233,135),1);
        return true;
    }

    bool windMill::IsValidFin(std::vector<cv::Point> contour) //判断是否为有效叶片
    {
        double s_area = cv::contourArea(contour);
        if (s_area < params.s_leaf_min || s_area > params.s_leaf_max)
            return false;

        cv::RotatedRect fin_rect = minAreaRect(contour);
        cv::Size2f fin_size = fin_rect.size;
        float length = fin_size.height > fin_size.width ? fin_size.height : fin_size.width; //将矩形的长边设置为长
        float width = fin_size.height < fin_size.width ? fin_size.height : fin_size.width;  //将矩形的短边设置为宽
        float lw_ratio = length / width;
        float s_ratio = s_area / fin_size.area();

        if (lw_ratio < params.fin_ratio_min || lw_ratio > params.fin_ratio_max)
            return false;
        if (s_ratio < params.s_fin_ratio_min || s_ratio > params.fin_ratio_max)
            return false;
#ifdef debug_show_fins
        char str[20];
        cv::RotatedRect preRect = cv::minAreaRect(contour);
        cv::Point2f *vertices = new cv::Point2f[4];
        preRect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::circle(src, vertices[i], 2 * i, cv::Scalar(0, 0, 255), -1);
            cv::line(src, vertices[i], vertices[(i + 1) % 4], cv::Scalar(10, 155, 255));
        }
        snprintf(str, 20, "leaf_s:%f", s_area);
        cv::putText(src, str, cv::Point2f(80, 90),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "lw_ratio:%f", lw_ratio);
        cv::putText(src, str, cv::Point2f(80, 110),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "s_ratio:%f", s_ratio);
        cv::putText(src, str, cv::Point2f(80, 130),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        cv::circle(src, preRect.center, 3, cv::Scalar(255, 235, 12), -1);
#endif
        return true;
    }

    bool windMill::IsValidArmor(std::vector<cv::Point> contour) //判断是否为有效装甲板
    {
        double s_area = cv::contourArea(contour);
        if (s_area < params.s_armor_min || s_area > params.s_armor_max)
            return false;
        cv::RotatedRect armor_rect = minAreaRect(contour);
        cv::Size2f armor_size = armor_rect.size;
        float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; //将矩形的长边设置为长
        float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  //将矩形的短边设置为宽
        float lw_ratio = length / width;
        float s_ratio = s_area / armor_size.area();
        if (lw_ratio < params.armor_ratio_min || lw_ratio > params.armor_ratio_max)
            return false;

        if (s_ratio < params.s_armor_ratio_min || s_ratio > params.s_armor_ratio_max)
            return false;
#ifdef debug_show_armors
        char str[20];
        cv::RotatedRect preRect = cv::minAreaRect(contour);
        cv::Point2f *vertices = new cv::Point2f[4];
        preRect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::circle(src, vertices[i], 2 * i, cv::Scalar(0, 0, 255), -1);
            cv::line(src, vertices[i], vertices[(i + 1) % 4], cv::Scalar(155, 255, 255));
        }
        snprintf(str, 20, "armor_s:%f", s_area);
        cv::putText(src, str, cv::Point2f(80, 210),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "armor_lw_ratio:%f", lw_ratio);
        cv::putText(src, str, cv::Point2f(80, 230),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        snprintf(str, 20, "armor_s_ratio:%f", s_ratio);
        cv::putText(src, str, cv::Point2f(80, 250),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        cv::circle(src, preRect.center, 3, cv::Scalar(255, 235, 12), -1);
#endif
        return true;
    }

    bool windMill::IsValidR(std::vector<cv::Point> contour) //判断是否为有效R
    {
        double s_area = cv::contourArea(contour);
        if (s_area < params.s_R_min || s_area > params.s_R_max)
            return false;
        cv::RotatedRect R_rect = minAreaRect(contour);
        cv::Size2f R_size = R_rect.size;
        float length = R_size.height > R_size.width ? R_size.height : R_size.width; //将矩形的长边设置为长
        float width = R_size.height < R_size.width ? R_size.height : R_size.width;  //将矩形的短边设置为宽
        float lw_ratio = length / width;
        float s_ratio = s_area / R_size.area();

        if (lw_ratio < params.R_ratio_min || lw_ratio > params.R_ratio_max)
            return false;
        if (s_ratio < params.s_R_ratio_min || s_ratio > params.s_R_ratio_max)
            return false;
#ifdef debug_show_R
        char str[20];
        cv::RotatedRect preRect = cv::minAreaRect(contour);
        cv::Point2f *vertices = new cv::Point2f[4];
        preRect.points(vertices);
        for (int i = 0; i < 4; i++)
        {
            cv::circle(src, vertices[i], 2 * i, cv::Scalar(0, 0, 255), -1);
            cv::line(src, vertices[i], vertices[(i + 1) % 4], cv::Scalar(10, 155, 255));
        }
        snprintf(str, 20, "R_s:%f", s_area);
        cv::putText(src, str, cv::Point2f(80, 150),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
                    std::cout<<s_area<<std::endl;
        snprintf(str, 20, "R_lw_ratio:%f", lw_ratio);
        cv::putText(src, str, cv::Point2f(80, 170),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
                    
        snprintf(str, 20, "R_s_ratio:%f", s_ratio);
        cv::putText(src, str, cv::Point2f(80, 190),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0));
        cv::circle(src, preRect.center, 3, cv::Scalar(255, 235, 12), -1);
#endif
        return true;
    }
}