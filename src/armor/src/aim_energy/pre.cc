#include "aim_energy/WindMill.hpp"

namespace WINDMILL
{
    void windMill::SmallPredict(float v, float pitch_deg)
    {
        if (direction)
        {
            params.constant_speed = -45;
        }
        else
        {
            params.constant_speed = 45;
        }
        double dist = HitDist(now_angle);
        double t_arrive1 = CalTime(dist, v, pitch_deg);
        double t_sum = t_arrive1;
        double t_arrive2 = 0.0;
        double next_angle = now_angle;
        double dt;
        for (int i = 0; i < 20; i++)
        {
            dt = t_arrive1 - t_arrive2;
            t_sum += dt;
            if (dt < 0.0005)
                break;
            next_angle = SPreNextAngle(next_angle, dt);
            dist = HitDist(next_angle);
            t_arrive2 = t_arrive1;
            t_arrive1 = CalTime(dist, v, pitch_deg);
        }
        pre_angle = SPreNextAngle(now_angle, t_sum);
        pre_point = CalPoint(center, params.R, pre_angle);
        // pre_point = hit_point;
        cv::circle(src, pre_point, 3, cv::Scalar(0, 0, 255), -1);
    }

    void windMill::BigPredict(float v, float pitch_deg)
    {
        if (anglevelocity_rad.size() < 30)
        {
            return;
        }
        else
        {
            FaiFit();
            double dist = HitDist(now_angle);
            double t_arrive1 = CalTime(dist, v, pitch_deg);
            double t_sum = t_arrive1;
            double t_arrive2 = 0.0;
            double next_angle = now_angle;
            double dt;
            double t0 = now_t - start_t;
            for (int i = 0; i < 20; i++)
            {
                dt = t_arrive1 - t_arrive2;
                t_sum += dt;
                if (dt < 0.0005)
                    break;
                next_angle = BPreNextAngle(next_angle, t0, dt);
                t0 += dt;
                dist = HitDist(next_angle);
                t_arrive2 = t_arrive1;
                t_arrive1 = CalTime(dist, v, pitch_deg);
            }
            pre_angle = BPreNextAngle(now_angle, t0, t_sum);
            pre_point = CalPoint(center, params.R, pre_angle);
            // pre_point = hit_point;
            cv::circle(src, pre_point, 3, cv::Scalar(0, 0, 255), -1);
        }
    };

    void windMill::FaiFit()
    {
        double lastfai;
        double fai_iterator = spd.fai;
        double loss_sum = 0.0;
        for (int i = 0; i < 50; i++)
        {
            iterator++;
            lastfai = fai_iterator;
            double diff1_sum = FaiDiff(fai_iterator, t_list, anglevelocity_rad);
            loss_sum = LossFun(fai_iterator, t_list, anglevelocity_rad);
            fai_iterator -= exp(-0.0003 * iterator) * (diff1_sum);
            if (loss_sum < last_loss)
            {
                break;
            }
        }
        double weight = last_loss / (loss_sum + last_loss);
        spd.fai = weight * spd.fai + (1 - weight) * fai_iterator;
        while (spd.fai < 0)
        {
            spd.fai += 2 * Pi;
        }
        while (spd.fai > 2 * Pi)
        {
            spd.fai -= 2 * Pi;
        }
    }
}