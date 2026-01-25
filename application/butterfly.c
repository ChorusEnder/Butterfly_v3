#include "tim.h"
#include "arm_math.h"

#include "motor.h"
// #include "as5600.h"
#include "remote_fs.h"
#include "butterfly.h"
// #include "butterfly_task.h"
#include "daemon.h"

static butterfly_mode_e butterfly_mode;
static Motor_Instance_s* motor_l;
static Motor_Instance_s* motor_r;
float feedforward_l;
float feedforward_r;
static RC_Fs_Ctrl_s *rc_fs;


/*-------------------以下是Asin(wt)+B有关的参数---------*/
static float angle_l;
static float angle_r;
static float time;
static float w = 8;//角速度,单位:rad/s

static float Al = 100;
static float bl = 20;
static float Ar = 100;
static float br = 20;
/*----------------------------------------------------*/

// static float dt;
// static uint32_t last_t;

void Butterfly_Init()
{
    // OSTask_Init();
    // DWT_Init(64);

    Motor_Init_Config_s motorConfig = {
        .controller = {
            // .loop_type = ANGLE_LOOP | SPEED_LOOP,
            .loop_type = OPEN_LOOP,
            .pid_ref = 0.0f,
            .angle_pid = {
                .kp = 5.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .deadband = 1.0f,
                .maxout = 700,
                .Improve = PID_T_Intergral | PID_I_limit | PID_D_Filter | PID_OutputFilter | PID_Changing_I,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 20.0f,
            },
            .speed_pid = {
                .kp = 0.0f,
                .ki = 0.1f,
                .kd = 0.0f,
                .deadband = 1,
                .maxout = VALUE_COMPARE,
                // .Improve = PID_T_Intergral | PID_I_limit | PID_D_On_Measurement | PID_D_Filter | PID_OutputFilter,
                .Improve = 0b00000000,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 500.0f,  
            }
        },
        .setting = {
            .pwm_config = {
                .htim = &htim3,
                .channel1 = TIM_CHANNEL_1,
                .channel2 = TIM_CHANNEL_2,
            },
            .flag_motor_reverse = MOTOR_DIR_NORMAL,
            .flag_feedback_reverse = FEEDBACK_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
            .motor_offset = 28.0f,

            .ptr_angle = NULL,
            .ptr_speed = NULL,
        }
    };
    motor_l = Motor_Init(&motorConfig);

    motorConfig.setting.pwm_config.htim = &htim3;
    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_3;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_4;
    motorConfig.setting.flag_motor_reverse = MOTOR_DIR_REVERSE;
    motorConfig.setting.flag_feedback_reverse = FEEDBACK_DIR_REVERSE;
    motorConfig.setting.motor_offset = 216.0f;
    motorConfig.setting.ptr_angle = NULL;
    motorConfig.setting.ptr_speed = NULL;
    motor_r = Motor_Init(&motorConfig);//正面

    rc_fs = RC_Fs_Init_Ibus(&huart1);

}


static void RemoteControl()
{
    if (sw_is_down(rc_fs->swd)){
        butterfly_mode = BUTTERFLY_MODE_STOP;
        return;
    }

    //电机版控制
    if (sw_is_up(rc_fs->swa) && sw_is_up(rc_fs->swb)){
        //由遥控器控制翼面位置
        butterfly_mode = BUTTERFLY_MODE_POSITION;

        angle_l = (rc_fs->rocker_l1) / 500.0f * 90.0f;
        angle_r = (rc_fs->rocker_l1) / 500.0f * 90.0f;

    }
    else if(sw_is_up(rc_fs->swa) && sw_is_down(rc_fs->swb)){
        //随正弦函数自动扑翼
        butterfly_mode = BUTTERFLY_MODE_FLYING;

    }

    //机构版控制
    if (sw_is_down(rc_fs->swa) && sw_is_up(rc_fs->swb))
    {
        butterfly_mode = BUTTERFLY_MODE_MECHANISM;

        angle_l = 300 * (rc_fs->rocker_l1 + 500) / 1000.0f;
        angle_r = 300 * (rc_fs->rocker_l1 + 500) / 1000.0f;

    }
    else if(sw_is_down(rc_fs->swa) && sw_is_down(rc_fs->swb))
    {

    }
}


static void MotorControl()
{
    //前馈计算,似乎可以直接移至motor.c中
    feedforward_l = 50 * cos(MotorGetAngle(motor_l) * 3.14f / 180.0f);
    feedforward_r = -100 * cos(MotorGetAngle(motor_r) * 3.14f / 180.0f);
    // feedforward_l = 50 * arm_cos_f32(MotorGetAngle(motor_l) * 3.14f / 180.0f);
    // feedforward_r = -100 * arm_cos_f32(MotorGetAngle(motor_r) * 3.14f / 180.0f);
    MotorSetFeedforward(motor_l, feedforward_l);
    MotorSetFeedforward(motor_r, feedforward_r);

    //默认使能
    MotorEnable(motor_l);
    MotorEnable(motor_r);

    //默认角度闭环控制
    MotorChangeLoop(motor_l, ANGLE_LOOP);
    MotorChangeLoop(motor_r, ANGLE_LOOP);

    switch (butterfly_mode)
    {
        case BUTTERFLY_MODE_STOP:
            //急停模式
            MotorStop(motor_l);
            MotorStop(motor_r);
            break;

        case BUTTERFLY_MODE_POSITION:
            //定点模式,通过遥控器控制翅膀位置
            
            //限幅
            if (angle_l < -80) angle_l = -80;
            if (angle_r < -80) angle_r = -80;
            if (angle_l > 100) angle_l = 90;
            if (angle_r > 100) angle_r = 90;

            break;
        case BUTTERFLY_MODE_FLYING:
            //飞行模式,通过遥控器控制翅膀速度,转向,升降...

            // angle_l = Al * arm_sin_f32(w * time) + bl;
            // angle_r = Ar * arm_sin_f32(w * time) + br;
            angle_l = Al * sin(w * time) + bl;
            angle_r = Ar * sin(w * time) + br;
            
            break;

        case BUTTERFLY_MODE_MECHANISM:
            //机构版模式,通过遥控器控制机构位置-开环控制
            MotorChangeLoop(motor_l, OPEN_LOOP);
            MotorChangeLoop(motor_r, OPEN_LOOP);

            //次模式不需要前馈
            MotorSetFeedforward(motor_l, 0);
            MotorSetFeedforward(motor_r, 0);

            break;     
    }

    MotorSetRef(motor_l, angle_l);
    MotorSetRef(motor_r, angle_r);

}


void Butterfly_Task()
{

    //application
    RemoteControl();
    MotorControl();

    //moudules
    MotorTask();
    DaemonTask();



}