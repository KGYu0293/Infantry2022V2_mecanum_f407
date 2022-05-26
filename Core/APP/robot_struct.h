//该头文件主要储存机器人的结构定义等APP层需要用到的参数

//底盘结构定义
// the radius of wheel(mm)，轮子半径
#define CHASSIS_RADIUS 71  // 71.25
// the perimeter of wheel(mm)，轮子周长
#define CHASSIS_PERIMETER 448  // 71.25*2pi
// wheel track distance(mm)，轮距
#define CHASSIS_WHEELTRACK 340
// wheelbase distance(mm)，轴距
#define CHASSIS_WHEELBASE 340
// gimbal is relative to chassis center x axis offset(mm)，云台相对于底盘中心的偏移，往右为正
#define CHASSIS_ROTATE_X_OFFSET 0
// gimbal is relative to chassis center y axis offset(mm)，云台相对于底盘中心的偏移，往前为正
#define CHASSIS_ROTATE_Y_OFFSET 0
// the deceleration ratio of chassis motor，底盘电机减速比
#define CHASSIS_MOTOR_DECELE_RATIO 14.0f
#define RADIAN_COEF 57.3f  // 180°/pi

//云台结构定义
// 云台水平并朝向底盘正前方时云台和底盘的编码器值
#define PITCH_MOTOR_ENCORDER_BIAS 1080
#define YAW_MOTOR_ENCORDER_BIAS 0
// 云台抬头/低头限位
#define PITCH_ENCORDER_HIGHEST 1870
#define PITCH_ENCORDER_LOWEST 560

//发弹结构定义
/* 摩擦轮半径(mm) */
#define SHOOT_RADIUS 30
/* 摩擦轮周长(mm) */
#define SHOOT_PERIMETER 94.25f  // 30 * 2 * pi
/* 拨弹电机减速比 */
#define SHOOT_MOTOR_DECELE_RATIO 36.0f
/* 拨弹轮每转一圈发弹数(个) */
#define SHOOT_NUM_PER_CIRCLE 8
/* 每发射一颗小弹增加的热量 */
#define SHOOT_UNIT_HEAT_17MM 10
/* 每发射一颗小弹增加的热量 */
#define SHOOT_UNIT_HEAT_42MM 100
//定义轴与下标的对应关系
#define ROLL_AXIS 0
#define PITCH_AXIS 1
#define YAW_AXIS 2
