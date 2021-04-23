#include "vicinity/Detective.hpp"

namespace AimVicinity
{

//对旋转矩形进行校正
cv::RotatedRect &adjustRec(cv::RotatedRect &rec, const int mode)
{
	using std::swap;

	float &width = rec.size.width;
	float &height = rec.size.height;
	float &angle = rec.angle;

	while (angle >= 90.0)
		angle -= 180.0;
	while (angle < -90.0)
		angle += 180.0;

	if (angle >= 45.0)
	{
		swap(width, height);
		angle -= 90.0;
	}
	else if (angle < -45.0)
	{
		swap(width, height);
		angle += 90.0;
	}

	return rec;
}

ArmorDetails::ArmorDetails()
{
	vertex.resize(4);
	for (int i = 0; i < 4; ++i)
	{
		vertex[i] = cv::Point2f(0, 0);
	}
	type = 0;
}
//根据灯条创建装甲版
ArmorDetails::ArmorDetails(const Lightbar &lLight, const Lightbar &rLight,const int Armortype)
{
	lightPairs[0] = lLight.rec();
	lightPairs[1] = rLight.rec();

	cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
	cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
	cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
	cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);
    //左侧等条的四个点
	cv::Point2f pts_l[4];
	exLLight.points(pts_l);
	cv::Point2f upper_l = pts_l[2];
	cv::Point2f lower_l = pts_l[3];
    //右侧灯条的四个点
	cv::Point2f pts_r[4];
	exRLight.points(pts_r);
	cv::Point2f upper_r = pts_r[1];
	cv::Point2f lower_r = pts_r[0];
	//左右灯条一起构成装甲轮廓
	vertex.resize(4);
	vertex[0] = upper_l;
	vertex[1] = upper_r;
	vertex[2] = lower_r;
	vertex[3] = lower_l;

	type=Armortype;
}

Detective::Detective()
{
	_roi = cv::Rect(cv::Point(0, 0), _srcImg.size());
	_debugWindowName = "debug info";
}

Detective::Detective(const aim_deps::Vicinity_param &vicinity_param)
{
	_param = vicinity_param;
	_roi = cv::Rect(cv::Point(0, 0), _srcImg.size());
	_debugWindowName = "debug info";
}

void Detective::init(const aim_deps::Vicinity_param &vicinity_param)
{
	_param = vicinity_param;
}
//读取图片
void Detective::loadImg(const cv::Mat &srcImg)
{
	_srcImg = srcImg;
#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
	_debugImg = srcImg.clone();
#endif // DEBUG_DETECTION || SHOW_RESULT
	cv::Rect imgBound = cv::Rect(cv::Point(0, 0), _srcImg.size());
	_roi = imgBound;
	_roiImg = _srcImg.clone();
}

int Detective::detect()
{
	/*
	*	Detect lights and build light bars' desciptors
	*/
	_armors.clear();
	std::vector<Lightbar> lightInfos;
	{
		//预处理
		cv::Mat binBrightImg,_hsvImg,_maskImg;
		cvtColor(_roiImg, _hsvImg, cv::COLOR_BGR2YUV,3);
		if(_enemy_color==BLUE)
		{
			cv::inRange(_hsvImg,_param.blue_lower_bound,_param.blue_upper_bound,_maskImg);
		}
		else{
			cv::inRange(_hsvImg,_param.red_lower_bound,_param.red_upper_bound,_maskImg);
		}
		cvtColor(_roiImg, _grayImg, cv::COLOR_BGR2GRAY, 1);
		cv::threshold(_grayImg, binBrightImg, _param.brightness_threshold, 255, cv::THRESH_BINARY);
		//cv::threshold(_maskImg, binBrightImg, _param.brightness_threshold, 255, cv::THRESH_BINARY);
		cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
		cv::morphologyEx(binBrightImg,binBrightImg,cv::MORPH_CLOSE,element);
		dilate(binBrightImg, binBrightImg, element);
		cv::imshow("Binary",binBrightImg);
		/*
		*	开始找灯条
		*/
		std::vector<std::vector<cv::Point>> lightContours;
		cv::findContours(binBrightImg.clone(), lightContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (const auto &contour : lightContours)
		{
			float lightContourArea = contourArea(contour);
			if (contour.size() <= 5 ||
				lightContourArea < _param.light_min_area)
				continue;//灯条面积过小的排除
			cv::RotatedRect lightRec = fitEllipse(contour);
			adjustRec(lightRec, 1);
			if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
				lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity)
				continue;//灯条长宽比不符合的排除
			lightRec.size.width *= _param.light_color_detect_extend_ratio;
			lightRec.size.height *= _param.light_color_detect_extend_ratio;
			//乘以个倍数，更好配对
			cv::Rect lightRect = lightRec.boundingRect();
			//创建roi
			const cv::Rect srcBound(cv::Point(0, 0), _roiImg.size());
			lightRect &= srcBound;
			cv::Mat lightImg = _roiImg(lightRect);
			cv::Mat lightMask = cv::Mat::zeros(lightRect.size(), CV_8UC1);
			cv::Point2f lightVertexArray[4];
			lightRec.points(lightVertexArray);
			std::vector<cv::Point> lightVertex;
			for (int i = 0; i < 4; ++i)
			{
				lightVertex.emplace_back(cv::Point(lightVertexArray[i].x - lightRect.tl().x,
											   lightVertexArray[i].y - lightRect.tl().y));
			}
			//输入灯条轮廓
			fillConvexPoly(lightMask, lightVertex, 255);

			if (lightImg.size().area() <= 0 || lightMask.size().area() <= 0)
				continue;//如果roi是空的话排除
			cv::dilate(lightMask, lightMask, element);
			const cv::Scalar meanVal = mean(lightImg, lightMask);
			if (((_enemy_color == BLUE) && (meanVal[BLUE] - meanVal[RED] > 20.0)) || (_enemy_color == RED && meanVal[RED] - meanVal[BLUE] > 20.0))
			{
				lightInfos.emplace_back(Lightbar(lightRec));//选取颜色较为恰当的
			}
		}
		std::vector<cv::RotatedRect> lightsRecs;
		for (auto &light : lightInfos)
		{
			lightsRecs.emplace_back(light.rec());
		}
		//cvex::showRectangles(_debugWindowName, _debugImg, _debugImg, lightsRecs, cvex::MAGENTA, -1, _roi.tl());
		//找到灯条后画出来
	}

	/*
	*	find and filter light bar pairs
	*/
	{
		sort(lightInfos.begin(), lightInfos.end(), [](const Lightbar &ld1, const Lightbar &ld2) {
			return ld1.center.x < ld2.center.x;//水平排序
		});
		std::vector<int> minRightIndices(lightInfos.size(), -1);
		for (size_t i = 0; i < lightInfos.size(); ++i)
		{
			for (size_t j = i + 1; (j < lightInfos.size()); ++j)
			{
				const Lightbar &leftLight = lightInfos[i];
				const Lightbar &rightLight = lightInfos[j];
				cv::Mat pairImg = _debugImg.clone();
				std::vector<cv::RotatedRect> curLightPair{leftLight.rec(), rightLight.rec()};
				float angleDiff_ = abs(leftLight.angle - rightLight.angle);
				float LenDiff_ratio = abs(leftLight.length - rightLight.length) / std::max(leftLight.length, rightLight.length);
				if (angleDiff_ > _param.light_max_angle_diff_ ||
					LenDiff_ratio > _param.light_max_height_diff_ratio_)
				{
					continue;//角度差或者长度查太大的话排除
				}
				float dis = std::sqrt(std::pow((leftLight.center.x - rightLight.center.x), 2) + std::pow((leftLight.center.y - rightLight.center.y), 2));
				float meanLen = (leftLight.length + rightLight.length) / 2;
				float yDiff = abs(leftLight.center.y - rightLight.center.y);
				float yDiff_ratio = yDiff / meanLen;
				float xDiff = abs(leftLight.center.x - rightLight.center.x);
				float xDiff_ratio = xDiff / meanLen;
				float ratio = dis / meanLen;
				if (yDiff_ratio > _param.light_max_y_diff_ratio_ ||
					xDiff_ratio < _param.light_min_x_diff_ratio_ ||
					ratio > _param.armor_max_aspect_ratio_ ||
					ratio < _param.armor_min_aspect_ratio_)
				{
					continue;//xy轴差远的话排除
				}
				int armorType = ratio > _param.armor_big_armor_ratio ? 2 : 1;
				// calculate the rotation score
				//根据配对结果创建装甲
				ArmorDetails armor(leftLight, rightLight,armorType);
				_armors.emplace_back(armor); //已经找到了
				break;
			}
		}
	}
	if (_armors.empty())
	{
		return 0;
	}
	return 1;			/// HQY adds this, P.S:你没用上你的返回值
}
//以下函数用来自喵和与云台板通信
//返回目标的轮廓

const std::vector<aim_deps::Armor> Detective::get_all_armors() const
{ 
	//std::cout<<_armors.size()<<std::endl;
	std::vector<aim_deps::Armor> outputArmor;
	for (auto &armor: _armors)
	{
		aim_deps::Armor _cache;
		int index=0;
		for(auto &point:armor.vertex)
		{
			_cache.vertex[index]=point;
			++index;
		}
		_cache.Isbigarmor = armor.type ==2 ? true : false;
		_cache.left_light = aim_deps::Light(armor.lightPairs[0]);
		_cache.right_light = aim_deps::Light(armor.lightPairs[1]);
		outputArmor.emplace_back(_cache);
	}
	return outputArmor;
}
void Detective::drawArmorPlates(cv::Mat &src, const std::vector<aim_deps::Armor> tar_list, const int optimal)
{
char str[2];
    cv::line(src, cv::Point(720, 0), cv::Point(720, 1080), cv::Scalar(255, 0, 0));
	cv::line(src, cv::Point(0, 540), cv::Point(1440, 540), cv::Scalar(255, 0, 0));
    for (size_t i = 0; i< tar_list.size(); ++i) {
        if(tar_list[i].armor_number != -1 && tar_list[i].valid){   //有意义的数字
            if((int)i != optimal){       //非最佳装甲板使用黄色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j], 
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 255), 2);   
                }
            }
            else{                   //最佳装甲板使用绿色绘制
                for (int j = 0; j < 4; ++j){
                    cv::line(src, tar_list[i].vertex[j],
                    tar_list[i].vertex[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
            }
            ///snprintf(str, 2, "%d", j);      //最佳装甲板位置x
	        ///cv::putText(src, str, tar_list[i].vertex[j]+cv::Point2f(2, 2),
	        ///    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0, 100, 255));
            snprintf(str, 2, "%d", tar_list[i].armor_number);
            cv::putText(src, str, cv::Point2f(25.0, 10.0) + tar_list[i].vertex[2],
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255));
        }
    }
}
void Detective::showDebugImg() const
{
	imshow(_debugWindowName, _debugImg);
}
cv::Mat Detective::get_gray_img()
{
	return this->_grayImg;
}

} // namespace AutoAim

namespace cv
{

template <typename T1, typename T2>
inline Point_<T1> operator/(const cv::Point_<T1> &pt, const T2 den)
{
	return Point_<T1>(pt.x / den, pt.y / den);
}

typedef cv::Rect_<float> Rect2f;

template <typename T>
cv::RotatedRect operator+(const cv::RotatedRect &rec, const cv::Point_<T> &pt)
{
	return cv::RotatedRect(cv::Point2f(rec.center.x + pt.x, rec.center.y + pt.y), rec.size, rec.angle);
}

template <typename T>
float distance(const cv::Point_<T> &pt1, const cv::Point_<T> &pt2)
{
	return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}

} // namespace cv

//以下为部分补充代码（包括计算距离等，还有一些颜色信息）
namespace cvex
{
const cv::Scalar BLUE(255, 0, 0);
const cv::Scalar GREEN(0, 255, 0);
const cv::Scalar RED(0, 0, 255);
const cv::Scalar MAGENTA(255, 0, 255);
const cv::Scalar YELLOW(0, 255, 255);
const cv::Scalar CYAN(255, 255, 0);
const cv::Scalar WHITE(255, 255, 255);
//两条线段找焦点
template <typename ValType>
const cv::Point2f crossPointOf(const std::array<cv::Point_<ValType>, 2> &line1, const std::array<cv::Point_<ValType>, 2> &line2)
{
	ValType a1 = line1[0].y - line1[1].y;
	ValType b1 = line1[1].x - line1[0].x;
	ValType c1 = line1[0].x * line1[1].y - line1[1].x * line1[0].y;

	ValType a2 = line2[0].y - line2[1].y;
	ValType b2 = line2[1].x - line2[0].x;
	ValType c2 = line2[0].x * line2[1].y - line2[1].x * line2[0].y;

	ValType d = a1 * b2 - a2 * b1;

	if (d == 0.0)
	{
		return cv::Point2f(FLT_MAX, FLT_MAX);
	}
	else
	{
		return cv::Point2f(float(b1 * c2 - b2 * c1) / d, float(c1 * a2 - c2 * a1) / d);
	}
}
inline const cv::Point2f crossPointOf(const cv::Vec4f &line1, const cv::Vec4f &line2)
{
	const std::array<cv::Point2f, 2> line1_{cv::Point2f(line1[2], line1[3]), cv::Point2f(line1[2] + line1[0], line1[3] + line1[1])};
	const std::array<cv::Point2f, 2> line2_{cv::Point2f(line2[2], line2[3]), cv::Point2f(line2[2] + line2[0], line2[3] + line2[1])};
	return crossPointOf(line1_, line2_);
}

void rotatedRectangle(cv::Mat &img, const cv::RotatedRect &rec, const cv::Scalar &color)
{
	if (&rec == nullptr)
		return;
	cv::Point2f rect_points[4];
	rec.points(rect_points);
	for (int j = 0; j < 4; ++j)
	{
		line(img, rect_points[j], rect_points[(j + 1) % 4], color);
	}
}
//画轮廓
template <typename ContourType>
void showContour(const std::string &windowName,
				 const cv::Mat &srcImg,
				 cv::Mat &dstImg,
				 ContourType contour,
				 const cv::Scalar &color,
				 const int waitTime = -1,
				 const cv::Point offset = cv::Point(0, 0))
{
	if (srcImg.channels() == 1)
		cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if (srcImg.data != dstImg.data)
		srcImg.copyTo(dstImg);

	cv::drawContours(dstImg, std::vector<ContourType>{contour}, -1, color, 1, 8, cv::noArray(), INT_MAX, offset);

	cv::imshow(windowName, dstImg);
	if (waitTime >= 0)
	{
		cv::waitKey(waitTime);
	}
}
//画轮廓
template <typename ContoursType>
void showContours(const std::string &windowName,
				  const cv::Mat &srcImg,
				  cv::Mat &dstImg,
				  const ContoursType &contours,
				  const cv::Scalar &color,
				  const int waitTime = -1,
				  const cv::Point offset = cv::Point(0, 0))
{
	if (srcImg.channels() == 1)
		cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
	else if (srcImg.data != dstImg.data)
		srcImg.copyTo(dstImg);

	cv::drawContours(dstImg, contours, -1, color, 1, 8, cv::noArray(), INT_MAX, offset);

	cv::imshow(windowName, dstImg);
	if (waitTime >= 0)
	{
		cv::waitKey(waitTime);
	}
}

template <typename RecType, bool = std::is_pointer<RecType>::value>
struct showRectangle_wrapper
{
};

template <typename RecType>
struct showRectangle_wrapper<RecType, true>
{
	static void showRectangle(const std::string &windowName,
							  const cv::Mat &srcImg,
							  const cv::Mat &dstImg,
							  RecType &rec,
							  const cv::Scalar &color,
							  const int waitTime = -1,
							  const cv::Point offset = cv::Point(0, 0))
	{
		if (srcImg.channels() == 1)
			cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if (srcImg.data != dstImg.data)
			srcImg.copyTo(dstImg);

		rotatedRectangle(dstImg, (*rec) + offset, color);

		cv::imshow(windowName, dstImg);
		if (waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}
	}
};

template <typename RecType>
struct showRectangle_wrapper<RecType, false>
{
	static void showRectangle(const std::string &windowName,
							  const cv::Mat &srcImg,
							  cv::Mat &dstImg,
							  RecType &rec,
							  const cv::Scalar &color,
							  const int waitTime = -1,
							  const cv::Point offset = cv::Point(0, 0))
	{
		if (srcImg.channels() == 1)
			cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if (srcImg.data != dstImg.data)
			srcImg.copyTo(dstImg);

		rotatedRectangle(dstImg, rec + offset, color);

		cv::imshow(windowName, dstImg);
		if (waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}
	}
};

template <typename RecType>
void showRectangle(const std::string &windowName,
				   const cv::Mat &srcImg,
				   cv::Mat &dstImg,
				   RecType &rec,
				   const cv::Scalar &color,
				   const int waitTime = -1,
				   const cv::Point offset = cv::Point(0, 0))
{
	using RecTypeWithoutRef = typename std::remove_reference<RecType>::type;
	showRectangle_wrapper<RecTypeWithoutRef>::showRectangle(windowName, srcImg, dstImg, rec, color, waitTime, offset);
}

template <typename RecsContainer, bool = std::is_pointer<RecsContainer>::value>
struct showRecs_wrapper
{
};

template <typename Recs>
struct showRecs_wrapper<Recs, true>
{
	static void showRectangles(const std::string &windowName,
							   const cv::Mat &srcImg,
							   cv::Mat &dstImg,
							   Recs &recs_or_recPtrs,
							   const cv::Scalar &color,
							   const int waitTime = -1,
							   const cv::Point offset = cv::Point(0, 0))
	{
		if (srcImg.channels() == 1)
			cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if (srcImg.data != dstImg.data)
			srcImg.copyTo(dstImg);

		for (const auto &rec : recs_or_recPtrs)
		{

			rotatedRectangle(dstImg, (*rec) + offset, color);
		}
		cv::imshow(windowName, dstImg);
		if (waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}
	}
};

template <typename Recs>
struct showRecs_wrapper<Recs, false>
{
	static void showRectangles(const std::string &windowName,
							   const cv::Mat &srcImg,
							   cv::Mat &dstImg,
							   Recs &recs_or_recPtrs,
							   const cv::Scalar &color,
							   const int waitTime = -1,
							   const cv::Point offset = cv::Point(0, 0))
	{
		if (srcImg.channels() == 1)
			cvtColor(srcImg, dstImg, cv::COLOR_GRAY2BGR);
		else if (srcImg.data != dstImg.data)
			srcImg.copyTo(dstImg);

		for (const auto &rec : recs_or_recPtrs)
		{
			rotatedRectangle(dstImg, rec + offset, color);
		}
		cv::imshow(windowName, dstImg);
		if (waitTime >= 0)
		{
			cv::waitKey(waitTime);
		}
	}
};

template <typename RecsContainer>
void showRectangles(const std::string &windowName,
					const cv::Mat &srcImg,
					cv::Mat &dstImg,
					RecsContainer &recs_or_recPtrs,
					const cv::Scalar &color,
					const int waitTime = -1,
					const cv::Point offset = cv::Point(0, 0))
{
	using RecsWithoutRef = typename std::remove_reference<RecsContainer>::type;
	showRecs_wrapper<RecsWithoutRef>::showRectangles(windowName, srcImg, dstImg, recs_or_recPtrs, color, waitTime, offset);
}

} // namespace cvex