#define MIN_ACCEL_X 60
#define MAX_ACCEL_X 300    // Maximun motor acceleration in (steps/seg2)/1000
#define MIN_ACCEL_Y 60
#define MAX_ACCEL_Y 145 
#define MAX_SPEED_X 25000 
#define MAX_SPEED_Y 25000

// This is for the Accel ramp implementation (to smooth the intial acceleration), simplified S-profile
// S型加减速曲线的加速度最小值和最大值
#define ACCEL_RAMP_MIN 2500  // The S profile is generated up to this speed
#define ACCEL_RAMP_MAX 10000

// 几何校准
// This depends on the pulley teeth. For 42 teeth GT2 => 19, for 40 teeth GT2 => 20, for 16 teeth T5 => 20
#define X_AXIS_STEPS_PER_UNIT 19    // With 42 teeth GT2 pulley and 1/8 microstepping on drivers
#define Y_AXIS_STEPS_PER_UNIT 19    // 200*8 = 1600 steps/rev = 1600/42teeth*2mm = 19.047, using 19 is an error of 1mm every 40cm not too much!

// 机器人能够移动的最大距离（单位mm）
#define ROBOT_MIN_X 0//-20000
#define ROBOT_MIN_Y 0//-20000
#define ROBOT_MAX_X 220//20000
#define ROBOT_MAX_Y 400//20000

// 桌面的中心（单位是mm），与摄像头位置和电机参数有关
// X方向与图像方向相同
// Y方向与图像方向相反
#define ROBOT_INITIAL_X 190
#define ROBOT_INITIAL_Y 210

// Initial robot position in mm
// The robot must be at this position at start time
// Default: Centered in X and minimun position in Y
#define ROBOT_INITIAL_POSITION_X 300
#define ROBOT_INITIAL_POSITION_Y 45   // Measured from center of the robot pusher to the table border

// Robot defense and attack lines(need change)
#define ROBOT_DEFENSE_POSITION 200
#define ROBOT_DEFENSE_ATTACK_POSITION 220

#define POSITION_TOLERANCE 5 // 5 steps

// Camera pixels
// NOTE: We are using the camera at 320x240 but we are outputing a 640x480 pixel equivalent position
#define CAM_PIX_WIDTH 640
#define CAM_PIX_HEIGHT 480
#define CAM_PIX_CENTER_X 320
#define CAM_PIX_CENTER_Y 240

// Camera geometric calibration
// 通过测量两个已知点来获得mm/pix： 60 / 69 = 0.86956
// 260与10是球能够在y轴达到的最大值和最小值，350mm是实际距离
#define CAM_PIX_TO_MM 0.86956

// 实际的mm与最终控制的移动参数之间的比例关系
#define MM_TO_CONTROL_X 0.655
#define MM_TO_CONTROL_Y 0.575

// Utils (don´t modify)
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ZERO_SPEED 65535

 //-----------------------------------------------------
#define DEFINE_LINE 350
#define TIMER_PARAMETER_X 5000000
#define TIMER_PARAMETER_Y 2000000

int sampleCount = 0; // 失步数检测间隔
