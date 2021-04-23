/**==========pnp测距测试模块，基于opencv solvePNP的iterative算法==============
 * @author: sentinel
 * last date of modification: 2020.5.23
 * 最后修改的内容：删除了几个没有用的函数和变量
*/
#include "aim_pos/GetPos.hpp"

GetPos::GetPos() {
	using namespace aim_deps;
	objPoints_small = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH_SMALL, _HALF_HEIGHT, 0),		//2,3,4,1象限顺序
		cv::Point3f(-_HALF_LENGTH_SMALL, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_SMALL, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_SMALL, _HALF_HEIGHT, 0),
	};
	objPoints_big = std::vector<cv::Point3f>{
		cv::Point3f(-_HALF_LENGTH_BIG, _HALF_HEIGHT, 0),		//2,3,4,1象限顺序
		cv::Point3f(-_HALF_LENGTH_BIG, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_BIG, -_HALF_HEIGHT, 0),
		cv::Point3f(_HALF_LENGTH_BIG, _HALF_HEIGHT, 0),
	};
	insM = aim_deps::HERO_INTRINSIC;
	distCoeffs = aim_deps::HERO_DIST;
	//insM.convertTo(insM, CV_64F);
	tVec.create(3, 1, CV_64F);
	rVec.create(3, 1, CV_64F);
	// g_ctrl.Init(0, 7.95, -4.56, 0.0, 0.0, 16.0, 0.017772);//0.000814);
}

GetPos::~GetPos(){;}

void GetPos::calcBallistic(cv::Point3f &pos, serial_com::comm &msg){
	if(doSlideWindow(pos, cv::Point2f(msg.x, msg.y))){	// 是否能使用滑动窗口
		pos.x = mean_pos.x;
		pos.y = mean_pos.y;
		pos.z = mean_pos.z;
	}
}

void GetPos::batchProcess(std::vector<aim_deps::Armor> &tar_list) const{
	for(size_t i = 0; i<tar_list.size(); ++i){
		positionScore(tar_list[i]);
	}
}

void GetPos::positionScore(aim_deps::Armor &tar) const{
	std::vector<cv::Point2f> tmp = {tar.vertex[0], tar.vertex[1], tar.vertex[2], tar.vertex[3]};
	if(tar.Isbigarmor){	
		cv::solvePnP(cv::InputArray(objPoints_big), cv::InputArray(tmp), 
			cv::InputArray(insM), cv::InputArray(distCoeffs), 
			cv::OutputArray(rVec), cv::OutputArray(tVec), false);
		tar.t_vec = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
		tar.r_vec = rVec.clone();
	}
	else{
		cv::solvePnP(cv::InputArray(objPoints_small), cv::InputArray(tmp), 
			cv::InputArray(insM), cv::InputArray(distCoeffs), 
			cv::OutputArray(rVec), cv::OutputArray(tVec), false);
		tar.t_vec = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
		tar.r_vec = rVec.clone();
	}
}

void GetPos::packUp(std::vector<cv::Mat> &rmats, std::vector<cv::Mat> &tmats, 
	const std::vector<aim_deps::Armor> &tar_list) const
{
	rmats.clear();
	tmats.clear();
	for(size_t i = 0; i< tar_list.size(); ++i){
		if(tar_list[i].armor_number != -1 && tar_list[i].valid){
			cv::Mat vect(3, 1, CV_64F);		
			/// cv::Point3f到cv::Mat
			vect.at<double>(0) = (double)tar_list[i].t_vec.x;
			vect.at<double>(1) = (double)tar_list[i].t_vec.y;
			vect.at<double>(2) = (double)tar_list[i].t_vec.z;
			cv::Mat rtmp;
			cv::Rodrigues(tar_list[i].r_vec, rtmp);
			rmats.emplace_back(rtmp);
			tmats.emplace_back(vect);
		}
	}
}

bool GetPos::doSlideWindow(cv::Point3f pos, cv::Point2f gimbal, int order){
    if(isnanf(gimbal.x) || isnanf(gimbal.y)){       // 出现了异常值
        return false;
    }
    float gim_size = (float)gimpos.size(), pos_size = (float)filter.size();
    if(gim_size < order - 2){
        gimpos.emplace_back(gimbal);
        filter.emplace_back(pos);
        mean_pos = (pos_size * mean_pos + pos) / (pos_size + 1);
        mean_gim = (gim_size * mean_gim + gimbal) / (gim_size + 1);
        return false;
    }
    // 计算云台位置均值
    if(gimpos.size() >= (size_t)order){         // 达到最大容量, 计算均值时，用上次的均值减去第一个入队点的均值分量　＋　新点分量
        cv::Point2f first = gimpos.front();
        mean_gim += gimbal / (float)order - first / (float)order;
    }
    else{                           // 未达到最大容量 无需减去第一个入队点的均值分量
        mean_gim = (gim_size * mean_gim + gimbal) / (gim_size + 1);
    }
    push_back(gimpos, gimbal, (size_t)order);
    
    // 计算云台位置方差， 假如方差很小，则认为相对静止
    cv::Point2f var_gim(0.0, 0.0);
    std::for_each(gimpos.begin(), gimpos.end(), [&](cv::Point2f &p){
        cv::Point2f tmp = p - mean_gim;
        var_gim.x += tmp.x * tmp.x / gim_size;
        var_gim.y += tmp.y * tmp.y / gim_size;
    });

    if(pos_size >= 9){         // 达到最大容量, 计算均值时，用上次的均值减去第一个入队点的均值分量　＋　新点分量
        cv::Point3f first = filter.front();
        mean_pos += pos / (float)order - first / (float)order;
    }
    else{                           // 未达到最大容量 无需减去第一个入队点的均值分量
        mean_pos = (pos_size * mean_pos + pos) / (pos_size + 1);
    }
    push_back<cv::Point3f>(filter, pos, (size_t)order);

    // 方差阈值可变
    // 与距离有关的方差阈值设置
    // 横轴偏移容忍度更小
    float thresh = 0.00017 * mean_pos.z - 0.01;
    gimbal_debug("Mean gim: %f, %f\n", mean_gim.x, mean_gim.y);
    gimbal_debug("Var(x, y): %f, %f with threshold (x: %f, y: %f)\n", var_gim.x, var_gim.y, thresh * 0.64, thresh);
    if(var_gim.x < thresh * 0.64  && var_gim.y < thresh){
        // 连续8帧比较稳定
        if(idle_cnt >= 8){
            return true;
        }
        else{
            idle_cnt++;
            return false;
        }
    }
    else{
        idle_cnt = 0;
        return false;
    }
}
