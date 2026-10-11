//
// Created by shiro2233 on 2026/10/11.
//

#ifndef LESSON3_RTOS_PID_HPP
#define LESSON3_RTOS_PID_HPP


class PID {
public:
    PID(float kp, float ki, float kd,
        float i_max, float out_max, float d_filter_k);

    void reset();
    void setGains(float kp, float ki, float kd);   // 运行中在线改增益
    void setLimits(float i_max, float out_max, float d_filter_k);   // 在线改限幅/滤波
    float calc(float ref, float fdb);

private:
    float kp_, ki_, kd_;
    float i_max_, out_max_, d_filter_k_;
    float err_, last_err_, err_sum_, d_out_, output_;
};


#endif //LESSON3_RTOS_PID_HPP
