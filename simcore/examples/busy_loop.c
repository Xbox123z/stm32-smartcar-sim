// ❌ 忙等示例：主循环里没有 HAL_Delay
// 预期结果：静态检查报 CARSIM003 错误，仿真判定为忙等崩溃

int main(void) {
    while (1) {
        int pos = Read_Line();
        Motor_Set(500 + pos, 500 - pos);
        // 忘了 HAL_Delay：循环空转，仿真时间无法推进
    }
}
