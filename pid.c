#include <stdio.h>
#include <math.h>

// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float target;       // 目标值
    float prev_error;   // 上一次误差
    float integral;     // 积分项
} PIDController;

// 初始化PID控制器
void pid_init(PIDController *pid, float kp, float ki, float kd, float target) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->prev_error = 0;
    pid->integral = 0;
}

// PID控制计算
float pid_compute(PIDController *pid, float current) {
    float error = pid->target - current;
    
    // 计算积分项
    pid->integral += error;
    
    // 计算微分项
    float derivative = error - pid->prev_error;
    
    // PID输出计算
    float output = pid->kp * error + 
                  pid->ki * pid->integral + 
                  pid->kd * derivative;
    
    // 保存当前误差
    pid->prev_error = error;
    
    return output;
}

// 声明load函数
extern void load(int left_speed, int right_speed);

// 直线运动PID控制函数
void straight_line_control(float target_angle) {
    const int BASE_SPEED = 50;  // 基础速度
    PIDController angle_pid;
    
    // 初始化PID控制器，这些参数需要根据实际情况调整
    pid_init(&angle_pid, 2.0f, 0.01f, 1.0f, target_angle);
    
    // 假设这是当前角度的获取函数（需要根据实际硬件实现）
    float current_angle = 0;  // 这里需要替换为实际的角度获取函数
    
    // PID控制输出
    float pid_output = pid_compute(&angle_pid, current_angle);
    
    // 根据PID输出调整左右轮速度
    int left_speed = BASE_SPEED - (int)pid_output;
    int right_speed = BASE_SPEED + (int)pid_output;
    
    // 限制速度范围
    if (left_speed > 100) left_speed = 100;
    if (left_speed < 0) left_speed = 0;
    if (right_speed > 100) right_speed = 100;
    if (right_speed < 0) right_speed = 0;
    
    // 控制电机
    load(left_speed, right_speed);
}
