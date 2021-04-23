/// 最终形式应该已经确定, HPG不使用mutex实现
/// 2020.5.1 by HQY
#include "aim_cam/HPG.hpp"

namespace hpg
{

    HPGrabbing::HPGrabbing()
    {
        for (int i = 0; i < 2; ++i)
        {
            buf[i].r_busy = true; //初始时默认不可读取，需等待第一次写入成功后方可读入
            buf[i].w_busy = false;
            buf[i].frame.pBufAddr = NULL;
        }
        grab = true;
        exp_long = 3000;
        exp_short = 4000;
        balance_b = 1957;
        balance_r = 3600;
        next_pos = 1;
        last_pos = NULL_POS;
    }

    HPGrabbing::~HPGrabbing() { ; }

    void HPGrabbing::init(int bb, int rb, int mode, const std::string path)
    {
        balance_b = bb;
        balance_r = rb;
        if (mode == FROM_VIDEO)
        {
            cap.open(path);
        }
        grab_mode = mode;
    }

    void HPGrabbing::start()
    {
        switch (grab_mode)
        {
        case FROM_CAM:
        {
            ctl.startGrabbing();
            //auto func = std::bind(&HPGrabbing::writeWorkThread, this, ctl.handle);
            std::thread w_thread(&HPGrabbing::writeWorkThread, this, ctl.handle);
            w_thread.detach(); //主线程创建后，子线程一直在工作
            hpg_debug("W_thread detached.\n");
            break;
        }
        case FROM_VIDEO:
            hpg_debug("Video loaded from the specified location.\n");
            break;
        case FROM_CAM_LOW_PERFORM:
        {
            ctl.startGrabbing();
            hpg_debug("Low performance grabbing.\n");
            break;
        }
        default:
            hpg_debug("Error occurs. Un-existing grab_mode:%d\n", grab_mode);
        }
    }

    int HPGrabbing::getMat(cv::Mat &src)
    {
        if (grab_mode == FROM_CAM)
        {
            if (buf[next_pos].r_busy == false)
            {                        //当前next_pos位置可以读取
                last_pos = next_pos; //设置本次读取位置
                readBuffer(last_pos, src);
                return GRAB_OK;
            }
            else
            {
                if (buf[1 - next_pos].r_busy == false)
                { //1-next_pos位置可以读取
                    last_pos = 1 - next_pos;
                    readBuffer(last_pos, src);
                    return GRAB_OK;
                }
            }
            return GRAB_FAILED;
        }
        else if (grab_mode == FROM_VIDEO)
        {
            return cap.read(src) ? GRAB_OK : GRAB_FAILED;
        }
        else
        {
            src = ctl.getOpencvMat();
            return src.empty() ? GRAB_FAILED : GRAB_OK;
        }
    }

    void HPGrabbing::setCameraParams(int mode)
    {
        switch (mode)
        {
        case DISTANCE_AIM:
            ctl.setExposureTime(exp_short);
            ctl.setWhiteBalance(balance_b, 1024, balance_r);
            break;
        case AUTO_AIM:
            ctl.setExposureTime(exp_long);
            ctl.setWhiteBalance(balance_b, 1024, balance_r);
            break;
        case 4:
            ctl.setExposureTime(exp_short);
            ctl.setWhiteBalance(balance_b, 1024, balance_r);
            break;
        default:
            break;
        }
    }

    ///////////////////////////////////////////////////////////////////////////
    /////////////////////////////////高性能取流部分///////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    void *HPGrabbing::writeWorkThread(void *pUser)
    {
        int next = 0, pos = 0, error = MV_OK;
        while (1)
        {
            pos = next;                      //保留本次写入位置
            next = writeBuffer(next, error); //每次写入之前会free上一次写入的buffer
            if (buf[pos].frame.pBufAddr != NULL)
            { //先释放存在已经写入数据的位置
                error = MV_CC_FreeImageBuffer(pUser, &buf[pos].frame);
#ifdef HPG_DEBUG
                if (error != MV_OK)
                {
                    hpg_debug("Freeing previous frame failed[0x%x].\n", error);
                }
                hpg_debug("Freed one spot.\n");
#endif
            }
            if (!grab)
                break;
        }
        hpg_debug("Exiting thread: writing...\n");
        return 0;
    }

    int HPGrabbing::writeBuffer(int pos, int &e)
    {
        if (isWritable(pos))
        {
            buf[pos].w_busy = true;
            buf[pos].r_busy = true;
            e = MV_CC_GetImageBuffer(ctl.handle, &buf[pos].frame, 1000);
            if (e != MV_OK)
            {
                hpg_debug("Failed to get image buffer.[0x%x]\n", e);
                return NULL_POS;
            }
        }
        buf[pos].w_busy = true;
        buf[pos].r_busy = false; //对应位置写入完毕，不可写入，可以读取
        hpg_debug("Writing succeeded. [%d].r_busy, w_busy=(%d, %d)\n",
                  pos, buf[pos].r_busy, buf[pos].w_busy);
        next_pos = pos; //可以read的位置在pos处（本次读取位置）
        return 1 - pos; //返回下一个写入位置
    }

    void HPGrabbing::readBuffer(const int pos, cv::Mat &src)
    {
        buf[pos].r_busy = true;
        buf[pos].w_busy = true;
        //================解码==================//
        cv::Mat yuyv_img(1080, 1440, CV_8UC2, buf[pos].frame.pBufAddr);
        cv::cvtColor(yuyv_img, src, cv::COLOR_YUV2BGR_YUYV);
        hpg_debug("Decoding is a success.\n");
    }

    bool HPGrabbing::isWritable(const int pos)
    {
        hpg_debug("Writing to [%d], isWritable:%d, isReadable:%d.\n",
                  pos, buf[pos].w_busy, buf[pos].r_busy);
        while (buf[pos].w_busy == true)
        { //自旋锁,无超时保护
            std::this_thread::yield();
        }
        return true;
    }
} //namespace hpg