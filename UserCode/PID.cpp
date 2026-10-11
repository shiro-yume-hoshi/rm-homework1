//
// Created by shiro2233 on 2026/10/11.
//
#include "PID.hpp"

PID::PID(float kp, float ki, float kd,
         float i_max, float out_max, float d_filter_k)
    : kp_(kp), ki_(ki), kd_(kd),
      i_max_(i_max), out_max_(out_max), d_filter_k_(d_filter_k),
      err_(0), last_err_(0), err_sum_(0), d_out_(0), output_(0) {
}

void PID::reset() {
    err_ = last_err_ = err_sum_ = d_out_ = output_ = 0;
}

float PID::calc(float ref, float fdb) {
    err_ = ref - fdb;

    err_sum_ += err_;
    if (err_sum_ > i_max_)  err_sum_ = i_max_;
    if (err_sum_ < -i_max_) err_sum_ = -i_max_;

    float d_raw = err_ - last_err_;
    d_out_ = d_out_ * (1.0f - d_filter_k_) + d_raw * d_filter_k_;

    output_ = kp_ * err_ + ki_ * err_sum_ + kd_ * d_out_;

    if (output_ > out_max_)  output_ = out_max_;
    if (output_ < -out_max_) output_ = -out_max_;

    last_err_ = err_;
    return output_;
}
