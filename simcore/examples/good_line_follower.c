// ✅ 好的循迹示例：PD 控制 + 限幅 + 稳定延时
// 预期结果：跑完 track01，评分 85+

#include "stm32f1xx_hal.h"

// @sim bind motor_left=TIM2_CH1
// @sim bind motor_right=TIM2_CH2

#define BASE_SPEED   500
#define MAX_TURN     350

float Kp = 300.0;      // 比例：决定转多急
float Kd = 90.0;       // 微分：抑制震荡
float last_pos = 0;

int main(void) {
    HAL_Init();

    while (1) {
        // 读加权巡线位置：左负右正，范围约 -2000 ~ 2000
        float pos = Read_Line();

        // PD 控制
        float turn = Kp * (pos / 2000.0) + Kd * ((pos - last_pos) / 2000.0);
        last_pos = pos;

        // 限幅，避免转向过猛冲出赛道
        if (turn > MAX_TURN) turn = MAX_TURN;
        if (turn < -MAX_TURN) turn = -MAX_TURN;

        int left  = BASE_SPEED + (int)turn;
        int right = BASE_SPEED - (int)turn;

        Motor_Set(left, right);

        HAL_Delay(10);      // 10ms 控制周期
    }
}
