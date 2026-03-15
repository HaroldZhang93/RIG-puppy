#include "xgo.h"
#include <math.h>
#include <stdlib.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_flash.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "xgo_action.h"


Motor motor[MOTOR_NUM];
uint16_t zero_buffer[MOTOR_NUM] = {2400,600,2400,600,1500};
uint16_t zero_buffer_default[MOTOR_NUM] = {2400,600,2400,600,1500};
uint16_t motor_speed = 0;
uint8_t Action_ID = 0;
uint8_t actionLoop_FLAG = 0;
uint8_t serial_lock = 0;
float vx = 0.0;
float vyaw = 0.0;
int calibrate_mode = 0;
int init_flag = 0;
// 5 组相位偏移表：前进、后退、右转、左转，对应 4 条腿和腰舵机的相位关系。
float l_p[][5] = {{3*PI/4.0, 3*PI/4.0, 3*PI/4.0, 3*PI/4.0, PI/4.0},
                  {-PI/4.0, -PI/4.0, -PI/4.0, -PI/4.0, PI/4.0},
                  {3*PI/4.0, -PI/4.0,  -PI/4.0, 3*PI/4.0, PI/4.0},
                  {-PI/4.0, 3*PI/4.0,  3*PI/4.0, -PI/4.0, PI/4.0}};

namespace {
// 行走时四条腿相对零位的基础展开量，决定默认“撑开”姿态。
constexpr float kWalkLegBaseOffset = 700.0f;
// 静止站立时的基础展开量，略小于行走姿态，避免完全僵硬。
constexpr float kIdleLegBaseOffset = 600.0f;
// 速度低于这个阈值时不进入步态摆动，保持站立姿态。
constexpr float kMotionStartThreshold = 15.0f;
// 最小动态步幅，确保低速命令下四肢也能迈开，不只是轻微抖动。
constexpr float kMinStepAmplitude = 180.0f;
// 最大动态步幅，限制高速时的腿部摆动，避免过大导致姿态失稳。
constexpr float kMaxStepAmplitude = 420.0f;
// 速度模长映射上限：命令强度达到这里后，步幅和步频按满量程处理。
constexpr float kCommandMagnitudeLimit = 300.0f;
// 最小步频增量，控制低速时相位推进速度。
constexpr float kMinPhaseIncrement = 0.12f;
// 最大步频增量，控制高速时相位推进速度。
constexpr float kMaxPhaseIncrement = 0.32f;
// 腰部摆动相对于腿部摆幅的比例，用于辅助重心转移。
constexpr float kWaistSwingScale = 0.85f;
// 步态调试日志采样间隔，数值越大日志越稀疏。
constexpr uint32_t kGaitDebugLogInterval = 25;

// 记录最近一次步态计算结果，供串口命令随时查看当前参数和目标位。
struct GaitDebugSnapshot {
    bool active = false;
    float ratio = 0.0f;
    float step = 0.0f;
    float pace_t = 0.0f;
    float phase_step = 0.0f;
    short des_pos[MOTOR_NUM] = {0};
};

bool gait_debug_enabled = false;
uint32_t gait_debug_counter = 0;
GaitDebugSnapshot gait_debug_snapshot;

// 简单限幅，防止步幅/步频映射超出预期范围。
float ClampFloat(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 保存本次步态解算结果，便于 `gaitstate` 和 `gaitdbg` 输出观察。
void UpdateGaitDebugSnapshot(bool active, float ratio, float step, float pace_t, float phase_step) {
    gait_debug_snapshot.active = active;
    gait_debug_snapshot.ratio = ratio;
    gait_debug_snapshot.step = step;
    gait_debug_snapshot.pace_t = pace_t;
    gait_debug_snapshot.phase_step = phase_step;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        gait_debug_snapshot.des_pos[i] = motor[i].DesPos;
    }
}
}  // namespace


void set_action_loop_flag(uint8_t flag){
    if(flag==1){
        Action_ID = 1;
        actionLoop_FLAG = 1;
    }else{
        Action_ID = 255;
        actionLoop_FLAG = 0;
    }
}

void WriteZeroPos(){
    uint32_t data[MOTOR_NUM];
    for(int i=0;i<MOTOR_NUM;i++){
        data[i] = motor[i].FbPos;
        motor[i].ZeroPos = motor[i].FbPos;
        printf("write zeropos [%d]: %d\r\n", i, motor[i].FbPos);
    }
    esp_err_t err = esp_flash_erase_region(NULL, FLASH_ZERO_POS_ADDR, 4096);
    if (err != ESP_OK) {
        return;
    }
    err = esp_flash_write(NULL, data, FLASH_ZERO_POS_ADDR, sizeof(data));
    if (err != ESP_OK) {
        return;
    }
}

bool ReadZeroPos(){
    uint32_t data[MOTOR_NUM] = {0};    
    esp_err_t err = esp_flash_read(NULL, data, FLASH_ZERO_POS_ADDR, sizeof(data));
    for(int i=0;i<MOTOR_NUM;i++){
        printf("zeropos [%d]: %ld\r\n", i, data[i]);
    }
    if (err != ESP_OK) {
        for(int i=0;i<MOTOR_NUM;i++){
            motor[i].ZeroPos = zero_buffer_default[i];
        }
        return false;
    }
    for(int i=0;i<MOTOR_NUM;i++){  
        if(data[i]<200||data[i]>2800){
            return false;
        }else{
            motor[i].ZeroPos = data[i];
        }
    }
    return true;
}

void InitZeroPos(){
    bool res;
    res = ReadZeroPos();
    for(int i=0;i<MOTOR_NUM;i++){
        motor[i].ID = i+1;
    }
    if(res){
        for(int i=0;i<MOTOR_NUM;i++){
            motor[i].Load = 1;
        }
    }else{
        calibrate_mode = 1;
    }
    init_flag = 1;
}

void SendMotorCommand(uint8_t *pData,uint16_t size)
{
    if(serial_lock){
		return;
	}else{
		serial_lock = 1;
	}
	uart_write_bytes(UART_NUM_2,pData,size);
    uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(50));
	serial_lock = 0;
}

void SetMotorPos(uint8_t ID,uint8_t addr,short pos,short vel){
	uint8_t bBuf[11];
	uint8_t checkSum = 0x00;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[3] = 0x07;
	bBuf[4] = 0x03;
	bBuf[5] = addr;
	bBuf[6] = pos & 0xff;
	bBuf[7] = pos>>8;
	bBuf[8] = vel & 0xff;
	bBuf[9] = vel>>8;
	checkSum = ID + 0x07 +0x03 + addr + bBuf[6] + bBuf[7] + bBuf[8] + bBuf[9];
	bBuf[10] = 0xff - checkSum;
	SendMotorCommand(bBuf, 11);
}

void ReadMotorState(uint8_t ID){
	uint8_t bBuf[8];
	uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[3] = 0x04;
	bBuf[4] = 0x02;
	bBuf[5] = 0x48;
	bBuf[6] = 0x06;
	CheckSum = ID + 0x04 + 0x02 + 0x48 + 0x06;
	bBuf[7] = ~CheckSum;
	SendMotorCommand(bBuf, 8);
}

void EnableMotor(uint8_t ID, uint8_t mode){
	uint8_t bBuf[8];
    uint8_t CheckSum = 0;
	bBuf[0] = 0xff;
	bBuf[1] = 0xff;
	bBuf[2] = ID;
	bBuf[3] = 0x04;
	bBuf[4] = 0x03;
	bBuf[5] = 0x30;
	bBuf[6] = mode;
	CheckSum = ID + 0x04 + 0x03 + 0x30 + mode;
	bBuf[7] = ~CheckSum;
	SendMotorCommand(bBuf, 8);
}

void EnableAllMotor(int mode){ 
    for(int i=0;i<MOTOR_NUM;i++){
        motor[i].Load = mode;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    for(int j=0;j<10;j++){
        for(int i=0;i<MOTOR_NUM;i++){
            EnableMotor(i+1, mode);
        }
    }
}

void move(){
    float ratio = 0.0f;
    float step = 0.0f;
    float phase_step = 0.0f;
    // 取命令绝对值，用于判断是否起步，以及计算前进/转向的混合比例。
    float abs_vx = fabsf(vx);
    float abs_vyaw = fabsf(vyaw);
    // 命令模长代表整体运动强度，后面会映射到步幅和步频。
    float command_mag = sqrtf(vx * vx + vyaw * vyaw);
    // 步态主相位，类似“当前走到一条步态曲线的哪个位置”。
    static float pace_t = 0.0f;
    int x_index = 0;
    int yaw_index = 2;

    // 根据前进/后退方向，选择不同的前后运动相位模板。
    if(vx > 0){
        x_index = 0;
    }else{
        x_index = 1;
    }

    // 根据左转/右转方向，选择不同的转向相位模板。
    if(vyaw > 0){
        yaw_index = 3;
    }else{
        yaw_index = 2;
    }

    if(Action_ID == 0){
        // 只有速度超过阈值才进入步态摆动，否则维持站立姿态。
        bool walk_active = (abs_vx > kMotionStartThreshold || abs_vyaw > kMotionStartThreshold);
        if(walk_active){
            float mix_sum = abs_vx + abs_vyaw;
            // 把输入速度归一化到 0~1，作为步幅和步频的统一驱动量。
            float normalized_mag = ClampFloat(command_mag / kCommandMagnitudeLimit, 0.0f, 1.0f);
            // ratio 越接近 1 越偏向前后步态，越接近 0 越偏向原地转向步态。
            ratio = (mix_sum > 0.0f) ? (abs_vx / mix_sum) : 1.0f;
            // 将命令强度映射到有效步幅区间，保证低速也有足够迈腿幅度。
            step = kMinStepAmplitude + normalized_mag * (kMaxStepAmplitude - kMinStepAmplitude);
            // 同时按命令强度映射步频，速度越大，相位推进越快。
            phase_step = kMinPhaseIncrement + normalized_mag * (kMaxPhaseIncrement - kMinPhaseIncrement);
            pace_t += phase_step;
            // 相位始终保持在 0~2PI，避免无限增大。
            if(pace_t > 2.0f * PI){
                pace_t -= 2.0f * PI;
            }

            // 4 条腿都围绕各自 ZeroPos 做“基础站姿 + 余弦摆动”。
            // 前两条腿使用 +cos，后两条腿使用 -cos，形成对角交替的推进节奏。
            // 相位由前后模板和转向模板按 ratio 混合，既能直行也能边走边转。
            motor[0].DesPos = motor[0].ZeroPos - (short)kWalkLegBaseOffset + (short)(step * cosf(pace_t + ratio * l_p[x_index][0] + (1.0f - ratio) * l_p[yaw_index][0]));
            motor[1].DesPos = motor[1].ZeroPos + (short)kWalkLegBaseOffset + (short)(step * cosf(pace_t + ratio * l_p[x_index][1] + (1.0f - ratio) * l_p[yaw_index][1]));
            motor[2].DesPos = motor[2].ZeroPos - (short)kWalkLegBaseOffset - (short)(step * cosf(pace_t + ratio * l_p[x_index][2] + (1.0f - ratio) * l_p[yaw_index][2]));
            motor[3].DesPos = motor[3].ZeroPos + (short)kWalkLegBaseOffset - (short)(step * cosf(pace_t + ratio * l_p[x_index][3] + (1.0f - ratio) * l_p[yaw_index][3]));
            // 腰舵机只做辅助摆动，帮助重心转移，因此幅度单独按比例缩放。
            motor[4].DesPos = motor[4].ZeroPos + (short)(step * kWaistSwingScale * cosf(pace_t + ratio * l_p[x_index][4] + (1.0f - ratio) * l_p[yaw_index][4]));
        }else{
            // 静止时回到稳定站姿，并清零相位，避免下一次起步从中途相位开始。
            pace_t = 0.0f;
            motor[0].DesPos = motor[0].ZeroPos - (short)kIdleLegBaseOffset;
            motor[1].DesPos = motor[1].ZeroPos + (short)kIdleLegBaseOffset;
            motor[2].DesPos = motor[2].ZeroPos - (short)kIdleLegBaseOffset;
            motor[3].DesPos = motor[3].ZeroPos + (short)kIdleLegBaseOffset;
            motor[4].DesPos = motor[4].ZeroPos;
        }

        // 保存本轮步态结果，供串口调试命令和周期日志读取。
        UpdateGaitDebugSnapshot(walk_active, ratio, step, pace_t, phase_step);
        // 打开调试后按固定采样间隔输出，避免每个控制周期都刷屏。
        if(gait_debug_enabled && walk_active && (++gait_debug_counter % kGaitDebugLogInterval) == 0){
            ESP_LOGI("XGO_GAIT",
                "active=%d vx=%.1f vyaw=%.1f ratio=%.2f step=%.1f phase_step=%.3f pace_t=%.2f des=[%d,%d,%d,%d,%d]",
                gait_debug_snapshot.active,
                vx,
                vyaw,
                gait_debug_snapshot.ratio,
                gait_debug_snapshot.step,
                gait_debug_snapshot.phase_step,
                gait_debug_snapshot.pace_t,
                gait_debug_snapshot.des_pos[0],
                gait_debug_snapshot.des_pos[1],
                gait_debug_snapshot.des_pos[2],
                gait_debug_snapshot.des_pos[3],
                gait_debug_snapshot.des_pos[4]);
        }
    }else{
        // 如果当前在执行预设动作，就交给动作状态机接管舵机目标位。
        xgo_action();
    }
} 

void SetGaitDebug(bool enable) {
    gait_debug_enabled = enable;
    gait_debug_counter = 0;
}

void PrintGaitDebugSnapshot() {
    // 打印最近一次步态解算快照，方便边发 move 命令边观察参数变化。
    printf("[GAIT] debug=%d active=%d vx=%.1f vyaw=%.1f ratio=%.2f step=%.1f phase_step=%.3f pace_t=%.2f des=[%d,%d,%d,%d,%d]\r\n",
        gait_debug_enabled ? 1 : 0,
        gait_debug_snapshot.active ? 1 : 0,
        vx,
        vyaw,
        gait_debug_snapshot.ratio,
        gait_debug_snapshot.step,
        gait_debug_snapshot.phase_step,
        gait_debug_snapshot.pace_t,
        gait_debug_snapshot.des_pos[0],
        gait_debug_snapshot.des_pos[1],
        gait_debug_snapshot.des_pos[2],
        gait_debug_snapshot.des_pos[3],
        gait_debug_snapshot.des_pos[4]);
}

uint8_t rxFlag = 0;
uint8_t rxLen = 0;
uint8_t rxDataLen = 0;
uint8_t id = 0;
uint8_t rxBuffer[30] = {0};
void xgo_rx(){
    uint8_t tempBuf[10];
    uint8_t res = 0; 
    uint8_t checkSum = 0;
    uint16_t POS_LOW_Byte = 0;
    uint16_t POS_HIGH_Byte = 0;
    uint16_t VEL_LOW_Byte = 0;
    uint16_t VEL_HIGH_Byte = 0;
    uint16_t TOR_LOW_Byte = 0;
    uint16_t TOR_HIGH_Byte = 0;
    while(uart_read_bytes(UART_NUM_2, tempBuf, 1, 5) > 0){
        res = tempBuf[0];
        switch(rxFlag)
        {
            case 0:
                if(res == 0xFF)
                    {rxFlag = 1;rxBuffer[0] = 0xFF;}
                    break;
            case 1:
                if(res == 0xFF)
                    {rxFlag = 2;rxBuffer[1] = 0xFF;}
                else{
                    rxFlag = 0;
                }
                break;
            case 2:
                    rxBuffer[2] = res;
                    id = res;
                    rxFlag = 3;
                    break;
            case 3:
                if(res == 0x08||res == 0x0B)
                    {					
                        rxFlag = 4; 
                        rxBuffer[3] = res;
                        rxLen = 0;
                        rxDataLen = res;
                        checkSum = 0;
                    }
                else{
                    rxFlag = 0;
                }                    
                break;
            case 4:
                rxBuffer[4+rxLen] = res;
                rxLen++;
                if(rxLen==rxDataLen){
                    for(int i=0; i<1+rxDataLen; i++){
                        checkSum += rxBuffer[2+i];
                    }
                    checkSum = ~checkSum;
                    if(checkSum == rxBuffer[3+rxDataLen]){          
                        POS_LOW_Byte =  rxBuffer[rxDataLen - 1];
                        POS_HIGH_Byte =  rxBuffer[rxDataLen];
                        VEL_LOW_Byte =  rxBuffer[rxDataLen - 3];
                        VEL_HIGH_Byte =  rxBuffer[rxDataLen - 2];
                        TOR_LOW_Byte =  rxBuffer[rxDataLen + 1];
                        TOR_HIGH_Byte =  rxBuffer[rxDataLen + 2];
                        if(id>0&&id<=MOTOR_NUM){
                            id = id - 1;
                            motor[id].FbPos = POS_LOW_Byte | (POS_HIGH_Byte << 8);
                            motor[id].FbSpd = VEL_LOW_Byte | (VEL_HIGH_Byte << 8);
                            motor[id].FbTor = TOR_LOW_Byte | (TOR_HIGH_Byte << 8);
                            id = id + 1;
                        }	 
                    }
                    checkSum = 0;
                    rxFlag = 0;                  			
                }
                break;
            default:
                rxFlag = 0;
                break;
        }		
    }
}

void detect_triple_click() {
    static int click_count = 0;           
    static uint32_t first_click_time = 0; 
    static bool button_pressed = false; 
    if(calibrate_mode == 0){
        return;
    }
    int level = gpio_get_level(GPIO_NUM_0);
    uint32_t current_time = esp_timer_get_time() / 1000;     

    if (level == 0 && !button_pressed) {
        button_pressed = true;
        
        if (click_count == 0) {
            first_click_time = current_time;
            click_count = 1;
        } else {
            if (current_time - first_click_time <= 1000) {
                click_count++;
                
                if (click_count >= 3) {
                    WriteZeroPos();
                    calibrate_mode = 0;                   
                    click_count = 0;
                    first_click_time = 0;
                    EnableAllMotor(1);
                }
            } else {
                click_count = 1;
                first_click_time = current_time;
            }
        }
    }
    
    if (level == 1 && button_pressed) {
        button_pressed = false;
    }
    
    if (click_count > 0 && (current_time - first_click_time) > 1000) {
        click_count = 0;
        first_click_time = 0;
    }
}

//Custom Servo Control Function - You can add your own servo commands here
void xgo_control() { 
    if(init_flag == 0){
        return;
    }
    static uint32_t counter = 0;
    static uint32_t counter2 = 0;
    static uint8_t read_id = 1;
    counter++; 
    counter2++;
    move();

    for(int i=0;i<5;i++){
        if(motor[i].Load){    
            SetMotorPos(i+1, 0x35, motor[i].DesPos, motor_speed);            
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if(counter2%10 == 0){
        detect_triple_click();
    }

    if(counter%10 == 0){
        ReadMotorState(read_id);
        counter = 0;
        read_id++;
        if(read_id > 6){
            read_id = 1;
        }
    }
}



