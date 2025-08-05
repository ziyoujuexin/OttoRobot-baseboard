#include "esp32c3_szp.h"
#include "app_process.h"
#include <inttypes.h>  // 添加此行

/***********************************************************/
/*********************    串口 ↓   *************************/

// UART配置参数
#define UART_NUM           UART_NUM_1
// 修改C3端的引脚定义以匹配S3端
#define UART_TX_PIN        GPIO_NUM_11  // 修改：对应S3的RX
#define UART_RX_PIN        GPIO_NUM_10  // 修改：对应S3的TX
#define UART_BAUD_RATE     115200
#define UART_BUFFER_SIZE   256            // 缓冲区大小

// 通信协议定义
#define FRAME_HEADER       0x55AA         // 修改：改为0x55AA以匹配S3端
#define FRAME_TAIL         0xFF           // 帧尾：0xFF
#define SENDER_ID          0x01           // 发送方ID
#define DATA_TYPE_MOTION   0x10           // 数据类型：运动指令

// 删除重复的运动状态定义，使用头文件中的定义
// 删除重复的motion_command_t结构体定义，使用头文件中的定义

// 全局变量
static const char *UART_TAG = "UART";
// static QueueHandle_t motion_queue = NULL;

/**
 * @brief 串口初始化函数
 */
void uart_init(void) {
    ESP_LOGI(UART_TAG, "开始UART初始化...");
    
    // 1. 配置 UART 参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART参数配置失败: %s", esp_err_to_name(ret));
        return;
    }

    // 2. 设置 UART 引脚
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART引脚设置失败: %s", esp_err_to_name(ret));
        return;
    }

    // 3. 安装 UART 驱动
    ret = uart_driver_install(UART_NUM, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(UART_TAG, "UART驱动安装失败: %s", esp_err_to_name(ret));
        return;
    }
    
    // 增强队列验证
    if (motion_queue == NULL) {
        ESP_LOGE(UART_TAG, "Motion queue not initialized in main");
        // 修复格式字符串错误
        ESP_LOGE(UART_TAG, "Available heap: %" PRIu32 " bytes", esp_get_free_heap_size());
        return;
    }
    
    // 验证队列是否可用
    UBaseType_t queue_spaces = uxQueueSpacesAvailable(motion_queue);
    ESP_LOGI(UART_TAG, "Motion queue验证成功，可用空间: %" PRIu32, (uint32_t)queue_spaces);
    
    ESP_LOGI(UART_TAG, "UART initialized on IO%d (TX) and IO%d (RX)", UART_TX_PIN, UART_RX_PIN);
}

/**
 * @brief 验证帧数据
 * @param frame 帧数据
 * @param len 帧长度
 * @return true 验证成功，false 验证失败
 */
static bool validate_frame(uint8_t *frame, int len) {
    if (len != 7) {
        return false;
    }

    // 检查帧头
    uint16_t header = (frame[0] << 8) | frame[1];
    if (header != FRAME_HEADER) {
        return false;
    }

    // 检查发送方ID和数据类型
    if (frame[2] != SENDER_ID || frame[3] != DATA_TYPE_MOTION) {
        return false;
    }
    
    // 检查帧尾
    if (frame[6] != FRAME_TAIL) {
        return false;
    }

    // 验证校验和
    uint8_t checksum = frame[0] + frame[1] + frame[2] + frame[3] + frame[4];
    if ((checksum & 0xFF) != frame[5]) {
        return false;
    }

    return true;
}

/**
 * @brief 串口接收任务
 * @param pvParameters 任务参数
 */
void uart_receive_task(void *pvParameters) {
    uint8_t data[UART_BUFFER_SIZE];
    uint8_t frame_buffer[7];  // 固定帧长度7字节
    int frame_index = 0;
    bool frame_started = false;

    ESP_LOGI(UART_TAG, "UART receive task started");

    while (1) {
        // 读取串口数据
        int len = uart_read_bytes(UART_NUM, data, UART_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (!frame_started) {
                    // 寻找帧头
                    if (frame_index == 0 && data[i] == 0xAA) {
                        frame_buffer[frame_index++] = data[i];
                    } else if (frame_index == 1 && data[i] == 0x55) {
                        frame_buffer[frame_index++] = data[i];
                        frame_started = true;
                    } else {
                        frame_index = 0;
                    }
                } else {
                    // 接收帧数据
                    frame_buffer[frame_index++] = data[i];
                    
                    if (frame_index >= 7) {
                        // 帧接收完成，验证并处理
                        if (validate_frame(frame_buffer, 7)) {
                            motion_command_t cmd;
                            cmd.motion_type = frame_buffer[4];
                            
                            // 发送到运动控制队列
                            if (xQueueSend(motion_queue, &cmd, 0) != pdTRUE) {
                                ESP_LOGW(UART_TAG, "Motion queue full, dropping command");
                            } else {
                                ESP_LOGI(UART_TAG, "Received motion command: %d", cmd.motion_type);
                            }
                        } else {
                            ESP_LOGW(UART_TAG, "Invalid frame received");
                        }
                        
                        // 重置帧接收状态
                        frame_index = 0;
                        frame_started = false;
                    }
                }
            }
        }
    }
}

/*********************    串口 ↑   *************************/
/***********************************************************/

// 注释掉所有I2C扩展和PCA9685相关代码
/*
// PCA9548A相关代码
static i2c_master_dev_handle_t PCA9548A_dev_handle = NULL;

void PCA9548A_init()
{
    // PCA9548A初始化代码已注释
}

void PCA9548A_select_channel(uint8_t channel)
{
    // PCA9548A通道选择代码已注释
}

void PCA9548A_reset(void)
{
    // PCA9548A复位代码已注释
}

// PCA9685相关代码
static i2c_master_dev_handle_t pca9685_dev_handle = NULL;

esp_err_t pca9685_write_reg(uint8_t reg, uint8_t data)
{
    // PCA9685寄存器写入代码已注释
}

void pca9685_setPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    // PCA9685 PWM设置代码已注释
}

void pca9685_setFreq(float freq)
{
    // PCA9685频率设置代码已注释
}

void pca9685_init(float freq, uint8_t angle)
{
    // PCA9685初始化代码已注释
}

void set_angle(uint8_t channel, uint8_t angle)
{
    // PCA9685角度设置代码已注释
}
*/

/***********************************************************/
/*********************    直接舵机控制 ↓   *******************/

// 舵机GPIO引脚定义
#define SERVO_GPIO_0    GPIO_NUM_0
#define SERVO_GPIO_1    GPIO_NUM_1
#define SERVO_GPIO_2    GPIO_NUM_2
#define SERVO_GPIO_3    GPIO_NUM_3

// LEDC配置参数
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // 设置占空比分辨率为13位
#define LEDC_FREQUENCY          (50) // 舵机PWM频率50Hz

// 舵机角度到占空比的转换参数
#define SERVO_MIN_PULSEWIDTH_US (500)  // 最小脉宽500us (0度)
#define SERVO_MAX_PULSEWIDTH_US (2500) // 最大脉宽2500us (180度)
#define SERVO_MAX_DEGREE        (180)  // 最大角度

// 舵机通道配置
static const gpio_num_t servo_gpios[SERVO_COUNT] = {
    SERVO_GPIO_0, SERVO_GPIO_1, SERVO_GPIO_2, SERVO_GPIO_3
};

static const char *SERVO_TAG = "SERVO";

/**
 * @brief 角度转换为占空比
 * @param angle 角度 (0-180)
 * @return 占空比值
 */
static uint32_t servo_angle_to_duty(uint8_t angle) {
    if (angle > SERVO_MAX_DEGREE) {
        angle = SERVO_MAX_DEGREE;
    }
    
    // 计算脉宽 (微秒)
    uint32_t pulsewidth_us = SERVO_MIN_PULSEWIDTH_US + 
        (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)) / SERVO_MAX_DEGREE;
    
    // 转换为占空比 (13位分辨率)
    uint32_t duty = (pulsewidth_us * ((1 << LEDC_DUTY_RES) - 1)) / (1000000 / LEDC_FREQUENCY);
    
    return duty;
}

/**
 * @brief 舵机PWM初始化
 */
void servo_pwm_init(void) {
    // 配置LEDC定时器
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // 配置LEDC通道
    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {
            .channel    = (ledc_channel_t)i,
            .duty       = 0,
            .gpio_num   = servo_gpios[i],
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        ESP_LOGI("esp32c3_szp", "舵机通道%d初始化完成，GPIO: %d", i, servo_gpios[i]);
    }
    
    ESP_LOGI("esp32c3_szp", "所有舵机PWM通道初始化完成");
}

/**
 * @brief 设置舵机角度
 * @param channel 舵机通道 (0-3)
 * @param angle 角度 (0-180)
 */
void set_servo_angle(uint8_t channel, uint8_t angle) {
    if (channel >= SERVO_COUNT) {
        ESP_LOGE(SERVO_TAG, "无效的舵机通道: %d", channel);
        return;
    }
    
    if (angle > SERVO_MAX_DEGREE) {
        ESP_LOGW(SERVO_TAG, "角度超出范围，限制为180度");
        angle = SERVO_MAX_DEGREE;
    }
    
    uint32_t duty = servo_angle_to_duty(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, (ledc_channel_t)channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, (ledc_channel_t)channel));
    
    ESP_LOGI(SERVO_TAG, "舵机%d设置角度: %d度, 占空比: %ld", channel, angle, duty);
    
    // 给舵机一些时间移动
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief 设置所有舵机到中心位置
 */
void servo_home_position(void) {
    ESP_LOGI("esp32c3_szp", "设置所有舵机到中心位置");
    for (int i = 0; i < SERVO_COUNT; i++) {
        set_servo_angle(i, 90);  // 设置到90度中心位置
    }
}

/*********************    直接舵机控制 ↑   *******************/
/***********************************************************/

// 舵机引脚定义（8个舵机）
#define SERVO_PIN_0    GPIO_NUM_2   // 左腿
#define SERVO_PIN_1    GPIO_NUM_3   // 右腿  
#define SERVO_PIN_2    GPIO_NUM_4   // 左脚
#define SERVO_PIN_3    GPIO_NUM_5   // 右脚
#define SERVO_PIN_4    GPIO_NUM_6   // 左手臂舵机1
#define SERVO_PIN_5    GPIO_NUM_7   // 左手臂舵机2
#define SERVO_PIN_6    GPIO_NUM_8   // 左耳朵舵机1
#define SERVO_PIN_7    GPIO_NUM_9   // 左耳朵舵机2

static const gpio_num_t servo_pins[SERVO_COUNT] = {
    SERVO_PIN_0, SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3,
    SERVO_PIN_4, SERVO_PIN_5, SERVO_PIN_6, SERVO_PIN_7
};
