// ⚠️ 积分饱和示例：PID 里的 I 项没有限幅
// 预期结果：能跑但会剧烈震荡，触发 CARSIM006 警告，评分中等

float Kp = 200.0;
float Ki = 3.0;        // 积分太强，而且下面没有限幅
float Kd = 40.0;

float integral = 0;
float last_err = 0;

int main(void) {
    while (1) {
        float pos = Read_Line();
        float err = 0 - pos;

        // 积分项一路累加，偏差持续存在时会饱和
        integral = integral + err;

        float out = Kp * (err / 2000.0) + Ki * (integral / 2000.0) + Kd * ((err - last_err) / 2000.0);
        last_err = err;

        Motor_Set(450 + (int)out, 450 - (int)out);
        HAL_Delay(10);
    }
}
