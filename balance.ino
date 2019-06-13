#include <PID_v1.h>
#include <Wire.h>
// fuck
int d[2];
//PID
//Define Variables we’ll be connecting to // Xác định các biến chúng tôi sẽ kết nối với
double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

double KpX=4, KiX=1.5, KdX=0.0001;
double KpY=4, KiY=1.5, KdY=0.0001;

PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Arduino Code/////////////////camservo.ino.txt//////////////////////
//#include <PWMServo.h>
#include <Servo.h>

// Title:   Auto Pan-Tilt Servo/Cam Control
// Subject: This Sketch receives X,Y coordinates from serial then    Chủ đề: Phác thảo này nhận tọa độ X, Y từ serial rồi
//          moves the camera to center of those coordinates.        // di chuyển camera đến trung tâm của các tọa độ đó.

#define  servomaxx   180   // max degree servo horizontal (x) can turn   tối đa servo ngang (x) có thể biến
#define  servomaxy   180   // max degree servo vertical (y) can turn     chiều dọc servo tối đa (y) có thể biến
#define  screenmaxx   600   // max screen horizontal (x)resolution       độ phân giải màn hình ngang (x) tối đa
#define  screenmaxy   480    // max screen vertical (y) resolution       độ phân giải màn hình dọc (y) tối đa
#define  servocenterx   90  // center po#define  of x servo              trung tâm po # định nghĩa của x servo
#define  servocentery   90  // center po#define  of y servo              trung tâm po # định nghĩa của y servo
#define  servopinx    40//11   // digital pin for servo x                     pin kỹ thuật số cho servo x
#define  servopiny   41 //12  // digital servo for pin y                      servo kỹ thuật số cho pin y
#define  baudrate 115200  // com port speed. Must match your C++ setting com tốc độ cổng. Phải phù hợp với cài đặt C ++ của bạn
#define distancex 3  //khoảng cách x // x servo rotation steps                           x bước xoay servo
#define distancey 3  //khoảng cách y // y servo rotation steps                           y bước xoay servo

int valx = 0;       // store x data from serial port                     lưu trữ dữ liệu x từ cổng nối tiếp
int valy = 0;       // store y data from serial port                     lưu trữ dữ liệu y từ cổng nối tiếp
int posx = 0;
int posy = 0;
int incx = 5;  // significant increments of horizontal (x) camera movement  sự gia tăng đáng kể của chuyển động camera ngang (x)
int incy = 5;  // significant increments of vertical (y) camera movement    sự gia tăng đáng kể của chuyển động camera dọc (y)

Servo servox;
Servo servoy;
 
//short MSB = 0;  // to build  2 byte integer from serial in byte                        MSB ngắn = 0; // để xây dựng số nguyên 2 byte từ nối tiếp theo byte
//short LSB = 0;  // to build  2 byte integer from serial in byte                        LSB ngắn = 0; // để xây dựng số nguyên 2 byte từ nối tiếp theo byte
//int   MSBLSB = 0;  //to build  2 byte integer from serial in byte                      int MSBLSB = 0; // để xây dựng số nguyên 2 byte từ nối tiếp theo byte
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// Uncomment this lines to connect to an external Wifi router (join an existing Wifi network)
// Bỏ ghi chú các đường này để kết nối với bộ định tuyến Wifi bên ngoài (tham gia mạng Wifi hiện có)
//#define EXTERNAL_WIFI
//#define WIFI_SSID "YOUR_WIFI"
//#define WIFI_PASSWORD "YOUR_PASSWORD"
//#define WIFI_IP "192.168.1.101"  // Force ROBOT IP
//#define TELEMETRY "192.168.1.38" // Tlemetry server port 2223

#define TELEMETRY "192.168.4.2" // Default telemetry server (first client) port 2223  Máy chủ từ xa mặc định (máy khách đầu tiên) cổng 2223

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)      THÔNG SỐ CHẾ ĐỘ BÌNH THƯỜNG (CÀI ĐẶT TỐI ĐA)
#define MAX_THROTTLE 550
#define MAX_STEERING 140
#define MAX_TARGET_ANGLE 14

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)    PRO MODE = THÊM NHIỀU HƠN (CÀI ĐẶT TỐI ĐA)
#define MAX_THROTTLE_PRO 780   // Max recommended value: 860    Giá trị đề xuất tối đa: 860
#define MAX_STEERING_PRO 260   // Max recommended value: 280    Giá trị đề xuất tối đa: 280
#define MAX_TARGET_ANGLE_PRO 26   // Max recommended value: 32  Giá trị đề xuất tối đa: 32

// Default control terms for EVO 2         Điều khoản kiểm soát mặc định cho EVO 2
#define KP 0.32       //0.32
#define KD 0.134    // 0.1 - 0.16 0.17 - 0.2 // {0.134}
#define KP_THROTTLE 0.080 
#define KI_THROTTLE 0.1 
#define KP_POSITION 0.06  
#define KD_POSITION 0.45  
#define KI_POSITION 0.02 // uncomment

// Control gains for raiseup (the raiseup movement requiere special control parameters)  Kiểm soát mức tăng để tăng (chuyển động tăng lên yêu cầu các tham số điều khiển đặc biệt)
#define KP_RAISEUP 0.1   
#define KD_RAISEUP 0.16   //0.16
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup    Không kiểm soát tốc độ khi nâng lên
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm wind up constants for PI control    Lặp lại các hằng số để kiểm soát PI
#define ITERM_MAX 10000

#define ANGLE_OFFSET 0.0  // Offset angle for balance (to compensate robot own weight distribution)   Góc bù cho cân bằng (để bù phân bố trọng lượng của robot)

// Servo definitions      Định nghĩa séc-vô
#define SERVO_AUX_NEUTRO 1500  // Servo neutral position    Vị trí trung tính Servo
#define SERVO_MIN_PULSEWIDTH 700     // CHIỀU RỘNG TỐI THIỂU SERVO
#define SERVO_MAX_PULSEWIDTH 2500    // CHIỀU RỘNG TỐI DA SERVO

#define SERVO2_NEUTRO 1500
#define SERVO2_RANGE 1400

// Telemetry
#define TELEMETRY_BATTERY 1
#define TELEMETRY_ANGLE 1
//#define TELEMETRY_DEBUG 1  // Dont use TELEMETRY_ANGLE and TELEMETRY_DEBUG at the same time!

#define ZERO_SPEED 65535
#define MAX_ACCEL 14      // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)   Tăng tốc động cơ tối đa (GIÁ TRỊ TỐI ĐA: 20) (mặc định: 14)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define DEBUG 0   // 0 = No debug info (default) DEBUG 1 for console output           0 = Không có thông tin gỡ lỗi (mặc định) DEBUG 1 cho đầu ra giao diện điều khiển

// AUX definitions    AUX định nghĩa
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

String MAC;  // MAC address of Wifi module     Địa chỉ MAC của mô-đun Wifi

uint8_t cascade_control_loop_counter = 0;
uint8_t loop_counter;       // To generate a medium loop 40Hz    Để tạo ra một vòng lặp trung bình 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz                     vòng lặp chậm 2Hz
uint8_t sendBattery_counter; // To send battery status           Để gửi trạng thái pin
int16_t BatteryValue;

long timer_old;
long timer_value;
float debugVariable;
float dt;

// Angle of the robot (used for stability control)            Góc của robot (được sử dụng để kiểm soát ổn định)
float angle_adjusted;        //điều chỉnh góc
float angle_adjusted_Old;
float angle_adjusted_filtered=0.0;      //điều chỉnh góc lọc

// Default control values from constant definitions          Giá trị điều khiển mặc định từ các định nghĩa không đổi
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float Kp_position = KP_POSITION;
float Kd_position = KD_POSITION;
bool newControlParameters = false;
bool modifing_control_parameters = false;    //sửa đổi control_parameter
int16_t position_error_sum_M1;
int16_t position_error_sum_M2;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
int16_t throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
float angle_offset = ANGLE_OFFSET;

boolean positionControlMode = false;
uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive) tích cực hơn

int16_t motor1;
int16_t motor2;

// position control
volatile int32_t steps1;
volatile int32_t steps2;
int32_t target_steps1;
int32_t target_steps2;
int16_t motor1_control;
int16_t motor2_control;

int16_t speed_M1, speed_M2;        // Actual speed of motors                                Tốc độ thực tế của động cơ
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors                   Hướng thực tế của động cơ bước
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)    tốc độ robot tổng thể (được đo từ tốc độ bước)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed                                 Ước tính tốc độ robot

// OSC output variables            //Biến đầu ra OSC
uint8_t OSCpage;
uint8_t OSCnewMessage;
float OSCfader[4];
float OSCxy1_x;
float OSCxy1_y;
float OSCxy2_x;
float OSCxy2_y;
uint8_t OSCpush[4];
uint8_t OSCtoggle[4];
uint8_t OSCmove_mode;
int16_t OSCmove_speed;
int16_t OSCmove_steps1;
int16_t OSCmove_steps2;


//================================================= read bluetooth=====================================
int state;
void readControl() 
{
  while(Serial2.available()>0)
  {
    state = Serial2.read();
    Serial2.print(state);
     if (state == 'F') // tien
    {
      throttle = throttle - 3;
      if (throttle <= -100) throttle = -100;
      steering = 0;      
    }
    else if (state == 'B') //lui
    {
      throttle = throttle + 3;
      if (throttle >= 100) throttle = 100;
      steering = 0;      
    }
    else if (state == 'L') 
    {
      steering = 25;
      if (throttle > 10) throttle = throttle - 1;
      if (throttle < -10) throttle = throttle + 1;
      if ((throttle <= 10) && (throttle >= -10)) throttle = 0; 
    }
    
    else if (state == 'R') 
    {
      steering = -25;
      if (throttle > 10) throttle = throttle - 1;
      if (throttle < -10) throttle = throttle + 1;
      if ((throttle <= 10) && (throttle >= -10)) throttle = 0; 
    }
    else if ((state == 'S')||(state == 'D'))
    {
      steering = 0;    
      if (throttle > 10) throttle = throttle - 4;
      if (throttle < -10) throttle = throttle + 4;
      if ((throttle <= 10) && (throttle >= -10)) throttle = 0; 
    }
    // tien trai 
    else if (state == 'G') 
    {
      throttle = throttle - 3;
      if (throttle <= -100) throttle = -100;
      steering = 25;
    }
    // tien phai
    else if (state == 'I') 
    {
        throttle = throttle - 3;
        if (throttle <= -100) throttle = -100;
        steering = -25;
    }
    // lui trai
    else if (state == 'H') 
    {
       throttle = throttle + 3;
      if (throttle >= 100) throttle = 100;
      steering = 25;
    }

   // lui phai
    else if (state == 'J') 
    {
       throttle = throttle + 3;
      if (throttle >= 100) throttle = 100;
      steering = -25;
    }
  }
}

//=======================================================================================================
// TRACKING CONTROL
void piControl()
{
      if(posx < 80) {
        steering = steering -10;    //phai
        if(steering <= -20) steering = -20;
      }
      if(posx > 100) {
        steering = steering + 10;     //trai
        if(steering >= 20) steering = 20;
      }
      if((posx >= 80) && (posx <= 100)) {
        steering = 0;
      }

      /////////////////////////////////////////////////////////
      if((posy < 100) && (posy > 90)) 
      {
        throttle = throttle - 5;//tien
        if(throttle <= -50) throttle = -50;
      }
      if((posy < 126) && (posy >= 100)) 
      {
        throttle = throttle - 3;//tien
        if( throttle <= -40) throttle = -40;
      }
      
      if(posy > 136) 
      {
        throttle = throttle + 3;//lui
        if( throttle >= 40) throttle = 40;
      }
      
      if((posy >= 126) && (posy <= 136)||(posy <= 90)) {
        if(throttle > 10)throttle = throttle - 4;
        if(throttle < -10)throttle = throttle + 4;
        if((throttle >= -10)&&(throttle <= 10)) throttle=0;
      }
}


//=======================================================================================================
// PID SERVO
void pid_servo()
{
  if (Serial3.available() >= 2)  
  {
    d[0]= (int)Serial3.read() -48;
    d[1]= (int)Serial3.read() -48;
    
    // Calculate PID outputs with Inputs
    InputX = d[0];
    InputY = d[1];
    
     posx = servox.read(); 
      posy = servoy.read();
      
    if((d[0] >= 1)&& (valx <= 7))
    {
      myPIDX.Compute();
      myPIDY.Compute();

      // Update Servo Position
      posx = constrain(posx + OutputX*2/3, 60, 140);
      posy = constrain(posy + OutputY*2/3, 20, 60);
      servox.write(posx);
      servoy.write(posy);
      piControl();
    }
    if (d[0] == 9) //Disconnect
    {
    //  servox.write(90);
     // servoy.write(90);
      steering = 0;
      throttle = 0;
    }
    
    if (d[0] == 0)  //No object
    {
      steering = 0;
      throttle = 0;

    }
  }
}




//======================================================================================================
// MAIN LOOP

// MPU6050
// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// MPU6050 IMU code
// Read the accel and gyro registers and calculate a complementary filter for sensor fusion between gyros and accel
// Đọc các thanh ghi accel và con quay hồi chuyển và tính toán một bộ lọc bổ sung cho phản ứng tổng hợp cảm biến giữa con quay và accel

// Code based on arduino.cc MPU6050 sample
// Open Source / Public Domain
//
// Documentation:"MPU-6000 and MPU-6050 Register Map and Descriptions": RM-MPU-6000A.pdf

// MPU6050 Register map
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R

// Defines for the bits, to be able to change            Xác định cho các bit, để có thể thay đổi
// between bit number and binary definition.           // giữa số bit và định nghĩa nhị phân.
// By using the bit number, programming the sensor     // Bằng cách sử dụng số bit, lập trình cảm biến
// is like programming the AVR microcontroller.        // giống như lập trình vi điều khiển AVR.
// But instead of using "(1<<X)", or "_BV(X)",         // Nhưng thay vì sử dụng "(1 << X)" hoặc "_BV (X)",
// the Arduino "bit(X)" is used.                       // Arduino "bit (X)" được sử dụng.
#define MPU6050_D0 0
#define MPU6050_D1 1
#define MPU6050_D2 2
#define MPU6050_D3 3
#define MPU6050_D4 4
#define MPU6050_D5 5
#define MPU6050_D6 6
#define MPU6050_D7 7

// AUX_VDDIO Register
#define MPU6050_AUX_VDDIO MPU6050_D7  // I2C high: 1=VDD, 0=VLOGIC

// CONFIG Register    ... Đăng ký CONFIG
// DLPF is Digital Low Pass Filter for both gyro and accelerometers.  // DLPF là Bộ lọc thông thấp kỹ thuật số cho cả con quay hồi chuyển và gia tốc kế.
// These are the names for the bits.                                  // Đây là tên của các bit.
// Use these only with the bit() macro.                               // Chỉ sử dụng những cái này với macro bit ().
#define MPU6050_DLPF_CFG0     MPU6050_D0
#define MPU6050_DLPF_CFG1     MPU6050_D1
#define MPU6050_DLPF_CFG2     MPU6050_D2
#define MPU6050_EXT_SYNC_SET0 MPU6050_D3
#define MPU6050_EXT_SYNC_SET1 MPU6050_D4
#define MPU6050_EXT_SYNC_SET2 MPU6050_D5

// Combined definitions for the EXT_SYNC_SET values                  // Các định nghĩa kết hợp cho các giá trị EXT_SYNC_SET
#define MPU6050_EXT_SYNC_SET_0 (0)
#define MPU6050_EXT_SYNC_SET_1 (bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_2 (bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_3 (bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_4 (bit(MPU6050_EXT_SYNC_SET2))
#define MPU6050_EXT_SYNC_SET_5 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET0))
#define MPU6050_EXT_SYNC_SET_6 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1))
#define MPU6050_EXT_SYNC_SET_7 (bit(MPU6050_EXT_SYNC_SET2)|bit(MPU6050_EXT_SYNC_SET1)|bit(MPU6050_EXT_SYNC_SET0))

// Alternative names for the combined definitions. .... Tên thay thế cho các định nghĩa kết hợp.
#define MPU6050_EXT_SYNC_DISABLED     MPU6050_EXT_SYNC_SET_0
#define MPU6050_EXT_SYNC_TEMP_OUT_L   MPU6050_EXT_SYNC_SET_1
#define MPU6050_EXT_SYNC_GYRO_XOUT_L  MPU6050_EXT_SYNC_SET_2
#define MPU6050_EXT_SYNC_GYRO_YOUT_L  MPU6050_EXT_SYNC_SET_3
#define MPU6050_EXT_SYNC_GYRO_ZOUT_L  MPU6050_EXT_SYNC_SET_4
#define MPU6050_EXT_SYNC_ACCEL_XOUT_L MPU6050_EXT_SYNC_SET_5
#define MPU6050_EXT_SYNC_ACCEL_YOUT_L MPU6050_EXT_SYNC_SET_6
#define MPU6050_EXT_SYNC_ACCEL_ZOUT_L MPU6050_EXT_SYNC_SET_7

// Combined definitions for the DLPF_CFG values      Các định nghĩa kết hợp cho các giá trị DLPF_CFG
#define MPU6050_DLPF_CFG_0 (0)
#define MPU6050_DLPF_CFG_1 (bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_2 (bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_3 (bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_4 (bit(MPU6050_DLPF_CFG2))
#define MPU6050_DLPF_CFG_5 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG0))
#define MPU6050_DLPF_CFG_6 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1))
#define MPU6050_DLPF_CFG_7 (bit(MPU6050_DLPF_CFG2)|bit(MPU6050_DLPF_CFG1)|bit(MPU6050_DLPF_CFG0))

// Alternative names for the combined definitions
// This name uses the bandwidth (Hz) for the accelometer,
// for the gyro the bandwidth is almost the same.
#define MPU6050_DLPF_260HZ    MPU6050_DLPF_CFG_0
#define MPU6050_DLPF_184HZ    MPU6050_DLPF_CFG_1
#define MPU6050_DLPF_94HZ     MPU6050_DLPF_CFG_2
#define MPU6050_DLPF_44HZ     MPU6050_DLPF_CFG_3
#define MPU6050_DLPF_21HZ     MPU6050_DLPF_CFG_4
#define MPU6050_DLPF_10HZ     MPU6050_DLPF_CFG_5
#define MPU6050_DLPF_5HZ      MPU6050_DLPF_CFG_6
#define MPU6050_DLPF_RESERVED MPU6050_DLPF_CFG_7

// GYRO_CONFIG Register
// The XG_ST, YG_ST, ZG_ST are bits for selftest.
// The FS_SEL sets the range for the gyro.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_FS_SEL0 MPU6050_D3
#define MPU6050_FS_SEL1 MPU6050_D4
#define MPU6050_ZG_ST   MPU6050_D5
#define MPU6050_YG_ST   MPU6050_D6
#define MPU6050_XG_ST   MPU6050_D7

// Combined definitions for the FS_SEL values
#define MPU6050_FS_SEL_0 (0)
#define MPU6050_FS_SEL_1 (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_2 (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_3 (bit(MPU6050_FS_SEL1)|bit(MPU6050_FS_SEL0))

// Alternative names for the combined definitions
// The name uses the range in degrees per second.
#define MPU6050_FS_SEL_250  MPU6050_FS_SEL_0
#define MPU6050_FS_SEL_500  MPU6050_FS_SEL_1
#define MPU6050_FS_SEL_1000 MPU6050_FS_SEL_2
#define MPU6050_FS_SEL_2000 MPU6050_FS_SEL_3

// ACCEL_CONFIG Register
// The XA_ST, YA_ST, ZA_ST are bits for selftest.
// The AFS_SEL sets the range for the accelerometer.
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_ACCEL_HPF0 MPU6050_D0
#define MPU6050_ACCEL_HPF1 MPU6050_D1
#define MPU6050_ACCEL_HPF2 MPU6050_D2
#define MPU6050_AFS_SEL0   MPU6050_D3
#define MPU6050_AFS_SEL1   MPU6050_D4
#define MPU6050_ZA_ST      MPU6050_D5
#define MPU6050_YA_ST      MPU6050_D6
#define MPU6050_XA_ST      MPU6050_D7

// Combined definitions for the ACCEL_HPF values
#define MPU6050_ACCEL_HPF_0 (0)
#define MPU6050_ACCEL_HPF_1 (bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_2 (bit(MPU6050_ACCEL_HPF1))
#define MPU6050_ACCEL_HPF_3 (bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_4 (bit(MPU6050_ACCEL_HPF2))
#define MPU6050_ACCEL_HPF_7 (bit(MPU6050_ACCEL_HPF2)|bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))

// Alternative names for the combined definitions
// The name uses the Cut-off frequency.
#define MPU6050_ACCEL_HPF_RESET  MPU6050_ACCEL_HPF_0
#define MPU6050_ACCEL_HPF_5HZ    MPU6050_ACCEL_HPF_1
#define MPU6050_ACCEL_HPF_2_5HZ  MPU6050_ACCEL_HPF_2
#define MPU6050_ACCEL_HPF_1_25HZ MPU6050_ACCEL_HPF_3
#define MPU6050_ACCEL_HPF_0_63HZ MPU6050_ACCEL_HPF_4
#define MPU6050_ACCEL_HPF_HOLD   MPU6050_ACCEL_HPF_7

// Combined definitions for the AFS_SEL values
#define MPU6050_AFS_SEL_0 (0)
#define MPU6050_AFS_SEL_1 (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_2 (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_3 (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

// Alternative names for the combined definitions
// The name uses the full scale range for the accelerometer.
#define MPU6050_AFS_SEL_2G  MPU6050_AFS_SEL_0
#define MPU6050_AFS_SEL_4G  MPU6050_AFS_SEL_1
#define MPU6050_AFS_SEL_8G  MPU6050_AFS_SEL_2
#define MPU6050_AFS_SEL_16G MPU6050_AFS_SEL_3

// FIFO_EN Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SLV0_FIFO_EN  MPU6050_D0
#define MPU6050_SLV1_FIFO_EN  MPU6050_D1
#define MPU6050_SLV2_FIFO_EN  MPU6050_D2
#define MPU6050_ACCEL_FIFO_EN MPU6050_D3
#define MPU6050_ZG_FIFO_EN    MPU6050_D4
#define MPU6050_YG_FIFO_EN    MPU6050_D5
#define MPU6050_XG_FIFO_EN    MPU6050_D6
#define MPU6050_TEMP_FIFO_EN  MPU6050_D7

// I2C_PIN_CFG Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKOUT_EN       MPU6050_D0
#define MPU6050_I2C_BYPASS_EN   MPU6050_D1
#define MPU6050_FSYNC_INT_EN    MPU6050_D2
#define MPU6050_FSYNC_INT_LEVEL MPU6050_D3
#define MPU6050_INT_RD_CLEAR    MPU6050_D4
#define MPU6050_LATCH_INT_EN    MPU6050_D5
#define MPU6050_INT_OPEN        MPU6050_D6
#define MPU6050_INT_LEVEL       MPU6050_D7

// INT_ENABLE Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_EN    MPU6050_D0
#define MPU6050_I2C_MST_INT_EN MPU6050_D3
#define MPU6050_FIFO_OFLOW_EN  MPU6050_D4
#define MPU6050_ZMOT_EN        MPU6050_D5
#define MPU6050_MOT_EN         MPU6050_D6
#define MPU6050_FF_EN          MPU6050_D7

// INT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_DATA_RDY_INT   MPU6050_D0
#define MPU6050_I2C_MST_INT    MPU6050_D3
#define MPU6050_FIFO_OFLOW_INT MPU6050_D4
#define MPU6050_ZMOT_INT       MPU6050_D5
#define MPU6050_MOT_INT        MPU6050_D6
#define MPU6050_FF_INT         MPU6050_D7

// MOT_DETECT_STATUS Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_ZRMOT MPU6050_D0
#define MPU6050_MOT_ZPOS  MPU6050_D2
#define MPU6050_MOT_ZNEG  MPU6050_D3
#define MPU6050_MOT_YPOS  MPU6050_D4
#define MPU6050_MOT_YNEG  MPU6050_D5
#define MPU6050_MOT_XPOS  MPU6050_D6
#define MPU6050_MOT_XNEG  MPU6050_D7

// IC2_MST_DELAY_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_I2C_SLV0_DLY_EN MPU6050_D0
#define MPU6050_I2C_SLV1_DLY_EN MPU6050_D1
#define MPU6050_I2C_SLV2_DLY_EN MPU6050_D2
#define MPU6050_I2C_SLV3_DLY_EN MPU6050_D3
#define MPU6050_I2C_SLV4_DLY_EN MPU6050_D4
#define MPU6050_DELAY_ES_SHADOW MPU6050_D7

// SIGNAL_PATH_RESET Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_TEMP_RESET  MPU6050_D0
#define MPU6050_ACCEL_RESET MPU6050_D1
#define MPU6050_GYRO_RESET  MPU6050_D2

// MOT_DETECT_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_MOT_COUNT0      MPU6050_D0
#define MPU6050_MOT_COUNT1      MPU6050_D1
#define MPU6050_FF_COUNT0       MPU6050_D2
#define MPU6050_FF_COUNT1       MPU6050_D3
#define MPU6050_ACCEL_ON_DELAY0 MPU6050_D4
#define MPU6050_ACCEL_ON_DELAY1 MPU6050_D5

// Combined definitions for the MOT_COUNT
#define MPU6050_MOT_COUNT_0 (0)
#define MPU6050_MOT_COUNT_1 (bit(MPU6050_MOT_COUNT0))
#define MPU6050_MOT_COUNT_2 (bit(MPU6050_MOT_COUNT1))
#define MPU6050_MOT_COUNT_3 (bit(MPU6050_MOT_COUNT1)|bit(MPU6050_MOT_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_MOT_COUNT_RESET MPU6050_MOT_COUNT_0

// Combined definitions for the FF_COUNT
#define MPU6050_FF_COUNT_0 (0)
#define MPU6050_FF_COUNT_1 (bit(MPU6050_FF_COUNT0))
#define MPU6050_FF_COUNT_2 (bit(MPU6050_FF_COUNT1))
#define MPU6050_FF_COUNT_3 (bit(MPU6050_FF_COUNT1)|bit(MPU6050_FF_COUNT0))

// Alternative names for the combined definitions
#define MPU6050_FF_COUNT_RESET MPU6050_FF_COUNT_0

// Combined definitions for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0 (0)
#define MPU6050_ACCEL_ON_DELAY_1 (bit(MPU6050_ACCEL_ON_DELAY0))
#define MPU6050_ACCEL_ON_DELAY_2 (bit(MPU6050_ACCEL_ON_DELAY1))
#define MPU6050_ACCEL_ON_DELAY_3 (bit(MPU6050_ACCEL_ON_DELAY1)|bit(MPU6050_ACCEL_ON_DELAY0))

// Alternative names for the ACCEL_ON_DELAY
#define MPU6050_ACCEL_ON_DELAY_0MS MPU6050_ACCEL_ON_DELAY_0
#define MPU6050_ACCEL_ON_DELAY_1MS MPU6050_ACCEL_ON_DELAY_1
#define MPU6050_ACCEL_ON_DELAY_2MS MPU6050_ACCEL_ON_DELAY_2
#define MPU6050_ACCEL_ON_DELAY_3MS MPU6050_ACCEL_ON_DELAY_3

// USER_CTRL Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_SIG_COND_RESET MPU6050_D0
#define MPU6050_I2C_MST_RESET  MPU6050_D1
#define MPU6050_FIFO_RESET     MPU6050_D2
#define MPU6050_I2C_IF_DIS     MPU6050_D4   // must be 0 for MPU-6050
#define MPU6050_I2C_MST_EN     MPU6050_D5
#define MPU6050_FIFO_EN        MPU6050_D6

// PWR_MGMT_1 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_CLKSEL0      MPU6050_D0
#define MPU6050_CLKSEL1      MPU6050_D1
#define MPU6050_CLKSEL2      MPU6050_D2
#define MPU6050_TEMP_DIS     MPU6050_D3    // 1: disable temperature sensor
#define MPU6050_CYCLE        MPU6050_D5    // 1: sample and sleep
#define MPU6050_SLEEP        MPU6050_D6    // 1: sleep mode
#define MPU6050_DEVICE_RESET MPU6050_D7    // 1: reset to default values

// Combined definitions for the CLKSEL
#define MPU6050_CLKSEL_0 (0)
#define MPU6050_CLKSEL_1 (bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_2 (bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_3 (bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_4 (bit(MPU6050_CLKSEL2))
#define MPU6050_CLKSEL_5 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL0))
#define MPU6050_CLKSEL_6 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1))
#define MPU6050_CLKSEL_7 (bit(MPU6050_CLKSEL2)|bit(MPU6050_CLKSEL1)|bit(MPU6050_CLKSEL0))

// Alternative names for the combined definitions
#define MPU6050_CLKSEL_INTERNAL    MPU6050_CLKSEL_0
#define MPU6050_CLKSEL_X           MPU6050_CLKSEL_1
#define MPU6050_CLKSEL_Y           MPU6050_CLKSEL_2
#define MPU6050_CLKSEL_Z           MPU6050_CLKSEL_3
#define MPU6050_CLKSEL_EXT_32KHZ   MPU6050_CLKSEL_4
#define MPU6050_CLKSEL_EXT_19_2MHZ MPU6050_CLKSEL_5
#define MPU6050_CLKSEL_RESERVED    MPU6050_CLKSEL_6
#define MPU6050_CLKSEL_STOP        MPU6050_CLKSEL_7

// PWR_MGMT_2 Register
// These are the names for the bits.
// Use these only with the bit() macro.
#define MPU6050_STBY_ZG       MPU6050_D0
#define MPU6050_STBY_YG       MPU6050_D1
#define MPU6050_STBY_XG       MPU6050_D2
#define MPU6050_STBY_ZA       MPU6050_D3
#define MPU6050_STBY_YA       MPU6050_D4
#define MPU6050_STBY_XA       MPU6050_D5
#define MPU6050_LP_WAKE_CTRL0 MPU6050_D6
#define MPU6050_LP_WAKE_CTRL1 MPU6050_D7

// Combined definitions for the LP_WAKE_CTRL
#define MPU6050_LP_WAKE_CTRL_0 (0)
#define MPU6050_LP_WAKE_CTRL_1 (bit(MPU6050_LP_WAKE_CTRL0))
#define MPU6050_LP_WAKE_CTRL_2 (bit(MPU6050_LP_WAKE_CTRL1))
#define MPU6050_LP_WAKE_CTRL_3 (bit(MPU6050_LP_WAKE_CTRL1)|bit(MPU6050_LP_WAKE_CTRL0))

// Alternative names for the combined definitions
// The names uses the Wake-up Frequency.
#define MPU6050_LP_WAKE_1_25HZ MPU6050_LP_WAKE_CTRL_0
#define MPU6050_LP_WAKE_2_5HZ  MPU6050_LP_WAKE_CTRL_1
#define MPU6050_LP_WAKE_5HZ    MPU6050_LP_WAKE_CTRL_2
#define MPU6050_LP_WAKE_10HZ   MPU6050_LP_WAKE_CTRL_3

// Default I2C address for the MPU-6050 is 0x68.
#define MPU6050_I2C_ADDRESS 0x68

// Util function to swap byte values
uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

// Declaring an union for the registers and the axis values.
// The byte order does not match the byte order of
// the compiler and AVR chip.
// The AVR chip (on the Arduino board) has the Low Byte
// at the lower address.
// But the MPU-6050 has a different order: High Byte at
// lower address, so that has to be corrected.
// The register part "reg" is only used internally,
// and are swapped in code.
typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};


// Global MPU6050 IMU variables
accel_t_gyro_union accel_t_gyro;
float x_gyro_value;   // in deg/seg units
float x_gyro_offset = 0.0;
float accel_angle;  // in degree units
float angle;

// This function implements a complementary filter to fusion gyro and accel info
float MPU6050_getAngle(float dt)
{
  accel_angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
  x_gyro_value = (accel_t_gyro.value.x_gyro - x_gyro_offset) / 65.5;  // Accel scale at 500deg/seg  => 65.5 LSB/deg/s

  // Complementary filter
  // We integrate the gyro rate value to obtain the angle in the short term and we take the accelerometer angle with a low pass filter in the long term...
  angle = 0.99 * (angle + x_gyro_value * dt) + 0.01 * accel_angle;  // Time constant = 0.99*0.01(100hz)/(1-0.99) = 0.99, around 1 sec.

  // Gyro bias correction
  // We supose that the long term mean of the gyro_value should tend to zero (gyro_offset). This means that the robot is not continuosly rotating.
  int16_t correction = constrain(accel_t_gyro.value.x_gyro, x_gyro_offset - 10, x_gyro_offset + 10); // limit corrections...
  x_gyro_offset = x_gyro_offset * 0.9995 + correction * 0.0005; // Time constant of this correction is around 20 sec.

  //Serial.print(angle);
  //Serial.print(" ");
  //Serial.println(x_gyro_offset);

  return angle;
}

// Calibrate function. Take 100 readings (over 2 seconds) to calculate the gyro offset value. IMU should be steady in this process...
void MPU6050_calibrate()
{
  int i;
  long value = 0;
  float dev;
  int16_t values[100];
  bool gyro_cal_ok = false;
  
  delay(500);
  while (!gyro_cal_ok){
    Serial.println("Gyro calibration... DONT MOVE!");
    // we take 100 measurements in 4 seconds
    for (i = 0; i < 100; i++)
    {
      MPU6050_read_3axis();
      values[i] = accel_t_gyro.value.x_gyro;
      value += accel_t_gyro.value.x_gyro;
      delay(25);
    }
    // mean value
    value = value / 100;
    // calculate the standard deviation
    dev = 0;
    for (i = 0; i < 100; i++)
      dev += (values[i] - value) * (values[i] - value);
    dev = sqrt((1 / 100.0) * dev);
    Serial.print("offset: ");
    Serial.print(value);
    Serial.print("  stddev: ");
    Serial.println(dev);
    if (dev < 50.0)
      gyro_cal_ok = true;
    else
      Serial.println("Repeat, DONT MOVE!");
  }
  x_gyro_offset = value;
  // Take the first reading of angle from accels
  angle = atan2f((float)accel_t_gyro.value.y_accel, (float)accel_t_gyro.value.z_accel) * RAD2GRAD;
}

void MPU6050_setup()
{
  int error;
  uint8_t c;

  error = MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
  Serial.print("WHO_AM_I : ");
  Serial.print(c, HEX);
  Serial.print(", error = ");
  Serial.println(error, DEC);

  // RESET chip
  MPU6050_write_reg(MPU6050_PWR_MGMT_1, bit(MPU6050_DEVICE_RESET));
  delay(125);
  // Clear the 'sleep' bit to start the sensor and select clock source
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);
  //MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);

  // Config Gyro scale (500deg/seg)
  MPU6050_write_reg(MPU6050_GYRO_CONFIG, MPU6050_FS_SEL_500);
  // Config Accel scale (2g)
  MPU6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_AFS_SEL_2G);
  // Config Digital Low Pass Filter 10Hz
  MPU6050_write_reg(MPU6050_CONFIG, MPU6050_DLPF_10HZ);
  // Set Sample Rate to 100Hz
  MPU6050_write_reg(MPU6050_SMPLRT_DIV, 9);  // 100Hz : Sample Rate = 1000 / (1 + SMPLRT_DIV) Hz
  // Data ready interrupt enable
  MPU6050_write_reg(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
  // Clear the 'sleep' bit to start the sensor (and select clock source).
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0x01);

  // Clear the 'sleep' bit to start the sensor.
  //MPU6050_write_reg(MPU6050_PWR_MGMT_1,MPU6050_CLKSEL_Z);
  //MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}

void MPU6050_read_3axis()
{
  int error;

  // read 14 bytes (gyros, temp and accels)
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if (error != 0) {
    Serial.print("MPU6050 Error:");
    Serial.println(error);
  }
  // swap bytes
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  /*
    // Print the raw acceleration values
    Serial.print("ACC:");
    Serial.print(accel_t_gyro.value.x_accel, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.y_accel, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.z_accel, DEC);
    Serial.println();

    // Print the raw gyro values.
    Serial.print("GY:");
    Serial.print(accel_t_gyro.value.x_gyro, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.y_gyro, DEC);
    Serial.print(",");
    Serial.print(accel_t_gyro.value.z_gyro, DEC);
    Serial.print(",");
    Serial.println();
  */
}

void MPU6050_read_1axis()
{
  int error;

  // read X accel
  error = MPU6050_read(MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_accel_h, 6);
  if (error != 0) {
    Serial.print("MPU6050 Error:");
    Serial.println(error);
  }
  // read X gyro
  error = MPU6050_read(MPU6050_GYRO_XOUT_H, (uint8_t *) &accel_t_gyro.reg.x_gyro_h, 2);
  if (error != 0) {
    Serial.print("MPU6050 Error:");
    Serial.println(error);
  }
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);

  // Print values
  Serial.print("axis:");
  Serial.print(accel_t_gyro.value.y_accel, DEC);
  Serial.print(",");
  Serial.println(accel_t_gyro.value.x_gyro, DEC);
}

// return true on new data available
bool MPU6050_newData()
{
  uint8_t status;
  int error;

  error = MPU6050_read(MPU6050_INT_STATUS, &status, 1);
  if (error != 0) {
    Serial.print("MPU6050 Error:");
    Serial.println(error);
  }
  if (status & (0b00000001)) // Data ready?
    return true;
  else
    return false;
}

// MPU6050_read n bytes
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// MPU6050_write n bytes
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg (only 1 byte)
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

//==============================================================================================
// CONTROL
// BROBOT EVO 2 by JJROBOTS
// SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// License: GPL v2
// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
// Thực hiện bộ điều khiển PD (Tỷ lệ, đạo hàm). DT trong vài giây
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts                                                                    Kd được thực hiện trong hai phần
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
                     // Phần lớn nhất chỉ sử dụng phần đầu vào (cảm biến) chứ không phải phần đầu vào đầu vào Setpoint (t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1).
  // Và lần thứ hai sử dụng điểm đặt để làm cho nó trở nên căng thẳng hơn một chút setpoint-setPoint (t-1).
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...Chúng tôi giới hạn phần đầu vào ...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component   ....lỗi cho Kd chỉ là thành phần đầu vào
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
// Thực hiện bộ điều khiển PI (Tỷ lệ, tích phân). DT trong vài giây
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in milliseconds...DT tính bằng mili giây ...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

//  TIMER 5 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER5_COMPA_vect)
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse  ...Nếu chúng ta không di chuyển, chúng ta không tạo ra xung
    return;
  // We generate 1us STEP pulse....Chúng tôi tạo xung 1us BƯỚC
  SET(PORTE, 4); // STEP MOTOR 1
  //delay_1us();
  if (dir_M1 > 0)
    steps1--;
  else
    steps1++;
  CLR(PORTE, 4);
}
// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER3_COMPA_vect)
{
  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTE, 5); // STEP MOTOR 2
  //delay_1us();
  if (dir_M2 > 0)
    steps2--;
  else
    steps2++;
  CLR(PORTE, 5);
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse) ... tốc độ có thể là tích cực hoặc tiêu cực (đảo ngược)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors ..... CHÚNG TÔI GIỚI HẠN TỐI ĐA của động cơ
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 50; // Adjust factor from control output speed to real motor speed in steps/second
                         // Điều chỉnh hệ số từ tốc độ đầu ra điều khiển sang tốc độ động cơ thực theo các bước / giây
#else
  speed = speed_M1 * 25; // 1/8 Micro stepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    SET(PORTE, 3); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    CLR(PORTE, 3); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)...Kiểm tra tốc độ tối thiểu (thời gian tối đa không tràn)
    timer_period = ZERO_SPEED;

  OCR5A = timer_period;
  // Check  if we need to reset the timer...Kiểm tra xem chúng ta có cần đặt lại timer không ...
  if (TCNT5 > OCR5A)
    TCNT5 = 0;
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)....tốc độ có thể là tích cực hoặc tiêu cực (đảo ngược)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 50; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 25; // 1/8 Micro stepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    CLR(PORTH, 3);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    SET(PORTH, 3);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}

//========================================================================================

// INITIALIZATION  khoi tao
void setup()
{
  //PID
  SetpointX = 4;
  SetpointY = 4;
  myPIDX.SetOutputLimits(-30, 30);
  myPIDY.SetOutputLimits(-30, 30);
  
  //turn PIDs on
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  /////
  
   // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
  pinMode(8, OUTPUT); // ENABLE MOTORS
  pinMode(2, OUTPUT); // STEP MOTOR 1 
  pinMode(5, OUTPUT); // DIR MOTOR 1  
  pinMode(3, OUTPUT); // STEP MOTOR 2
  pinMode(6, OUTPUT); // DIR MOTOR 2  
  digitalWrite(8, HIGH);  // Disbale motors
  pinMode(servopinx,OUTPUT);    // declare the LED's pin as output    khai báo chân của đèn LED là đầu ra
  pinMode(servopiny,OUTPUT);    // declare the LED's pin as output

  Serial.begin(115200);
  Serial3.begin(115200);// Serial output to console           Đầu ra nối tiếp đến bàn điều khiển
  Serial2.begin(9600);
  
  //Setup servo
  servoy.attach(servopiny); 
  servox.attach(servopinx); 
   
  // Initialize I2C bus (MPU6050 is connected via I2C)     Khởi tạo bus I2C (MPU6050 được kết nối qua I2C)
  Wire.begin();

  #if DEBUG > 0
  delay(9000);
#else
  delay(1000);
#endif

  Serial.println("JJROBOTS");
  delay(200);
  Serial.println("Don't move for 10 sec...");
  MPU6050_setup();  // setup MPU6050 IMU
  delay(1000);

  // Calibrate gyros   Con quay hiệu chỉnh
  MPU6050_calibrate();
  delay(1000);

  // STEPPER MOTORS INITIALIZATION    ĐỘNG CƠ BAN ĐẦU
  Serial.println("Stepers init");
  // MOTOR1 => TIMER1
  TCCR5A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected    Timer1 CTC chế độ 4, OCxA, B đầu ra bị ngắt kết nối
  TCCR5B = (1 << WGM52) | (1 << CS51); // Prescaler=8, => 2Mhz                           Bộ đếm trước = 8, => 2Mhz
  OCR5A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT5 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;
  delay(200);

  // Enable stepper drivers and TIMER interrupts                             Cho phép trình điều khiển bước và ngắt TIMER
  digitalWrite(8, LOW);   // Enable stepper drivers                           Kích hoạt trình điều khiển bước
  // Enable TIMERs interrupts                                                // Kích hoạt ngắt TIMER
  TIMSK5 |= (1 << OCIE5A); // Enable Timer5 interrupt
  TIMSK3 |= (1 << OCIE3A); // Enable Timer3 interrupt

  // Little motor vibration and servo move to indicate that robot is ready   Rung động cơ nhỏ và di chuyển servo để chỉ ra rằng robot đã sẵn sàng
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
//    BROBOT_moveServo1(SERVO_AUX_NEUTRO + 100);
//    BROBOT_moveServo2(SERVO2_NEUTRO + 100);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
//    BROBOT_moveServo1(SERVO_AUX_NEUTRO - 100);
//    BROBOT_moveServo2(SERVO2_NEUTRO - 100);
    delay(200);
  }


  // servo initialization    
  //khởi tạo servo
  
  //servox.write(servocenterx); 
 // delay(200);
 servox.write(servocenterx); 
 servoy.write(servocentery); 

       //posx = constrain(posx + OutputX*2/3, 60, 140);
      //posy = constrain(posy + OutputY*2/3, 20, 60);
 while (1){
  /* delay(1000);
   for (int a=60;a<145; a+=10)
  {servox.write(a);
  Serial.print(a);
  Serial.println("x");
  delay(1000);}
  servox.write(servocenterx);
  delay(1000); */
   for (int a=20;a<65; a+=10)
  {servoy.write(a); 
  delay(1000);
    Serial.print(a);
  Serial.println("y");
  float b = 15* tan(90-a)+100;
  Serial.print(b);
  Serial.println("goc");
  }
   servoy.write(servocentery); 
   delay(3000);
  }
  /////////////////////////////////////
  
  Serial.println("BROBOT by JJROBOTS v2.82");
  Serial.println("Start...");
  timer_old = micros();
}


void loop()
{
   readControl() ;
   pid_servo();
   timer_value = micros();

   // New IMU data?
  if (MPU6050_newData())
  {
    MPU6050_read_3axis();
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old) * 0.000001; // dt in seconds
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)         Nhận góc định hướng mới từ IMU (MPU6050)
    float MPU_sensor_angle = MPU6050_getAngle(dt);
    angle_adjusted = MPU_sensor_angle + angle_offset;
    if ((MPU_sensor_angle>-15)&&(MPU_sensor_angle<15))
      angle_adjusted_filtered = angle_adjusted_filtered*0.99 + MPU_sensor_angle*0.01;
      
#if DEBUG==1
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(angle_offset);
    Serial.print(" ");
    Serial.print(angle_adjusted);
    Serial.print(",");
    Serial.println(angle_adjusted_filtered);
#endif
    //Serial.print("\t");

    // We calculate the estimated robot speed:       Chúng tôi tính toán tốc độ robot ước tính:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)  (góc đo bằng IMU)
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward   Tích cực: về phía trước  

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 25.0; // 25 is an empirical extracted factor to adjust for real units  
                                                                             // 25 là một yếu tố trích xuất theo kinh nghiệm để điều chỉnh cho các đơn vị thực
    int16_t estimated_speed = -actual_robot_speed + angular_velocity;
    estimated_speed_filtered = estimated_speed_filtered * 0.9 + (float)estimated_speed * 0.1; // low pass filter on estimated speed  bộ lọc thông thấp về tốc độ ước tính

#if DEBUG==2
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif

    if (positionControlMode)
    {
      // POSITION CONTROL. INPUT: Target steps for each motor. Output: motors speed
      // KIỂM SOÁT VỊ TRÍ. INPUT: Mục tiêu các bước cho mỗi động cơ. Đầu ra: tốc độ động cơ
      motor1_control = positionPDControl(steps1, target_steps1, Kp_position, Kd_position, speed_M1);
      motor2_control = positionPDControl(steps2, target_steps2, Kp_position, Kd_position, speed_M2);

      // Convert from motor position control to throttle / steering commands
      // Chuyển đổi từ điều khiển vị trí động cơ sang lệnh ga / lái
      throttle = (motor1_control + motor2_control) / 2;
      throttle = constrain(throttle, -190, 190);
      steering = motor2_control - motor1_control;
      steering = constrain(steering, -50, 50);
    }

    // ROBOT SPEED CONTROL: This is a PI controller.     // KIỂM SOÁT TỐC ĐỘ ROBOT: Đây là bộ điều khiển PI.
    //    input:user throttle(robot speed), variable: estimated robot speed, output: target robot angle to get the desired speed
    // đầu vào: bướm ga người dùng (tốc độ robot), biến số: tốc độ robot ước tính, đầu ra: góc robot mục tiêu để có được tốc độ mong muốn
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output


#if DEBUG==3
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control (100Hz loop): This is a PD controller.
                     // Điều khiển ổn định (vòng lặp 100Hz): Đây là bộ điều khiển PD.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
                     // đầu vào: góc mục tiêu của robot (từ TỐC ĐỘ KIỂM SOÁT), biến: góc robot, đầu ra: Tốc độ động cơ
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
                     // Chúng tôi tích hợp đầu ra (tính tổng), vì vậy đầu ra thực sự là gia tốc động cơ, không phải tốc độ động cơ.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly to the output   // Phần lái từ người dùng được đưa trực tiếp vào đầu ra
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    
    if ((angle_adjusted < 74) && (angle_adjusted > -74)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(8, LOW);  // Motors enable
      // NOW we send the commands to the motors         BÂY GIỜ chúng tôi gửi lệnh cho các động cơ
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
    }
    else   // Robot not ready (flat), angle > angle_ready => ROBOT OFF    Robot chưa sẵn sàng (flat), angle> angle_ yet => ROBOT OFF
    {
      digitalWrite(8, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP    KIỂM SOÁT GAIN CHO RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // RESET steps
      steps1 = 0;
      steps2 = 0;
      positionControlMode = false;
//      OSCmove_mode = false;
      throttle = 0;
      steering = 0;
    }

    // Normal condition?    Tình trạng bình thường?
    if ((angle_adjusted < 56) && (angle_adjusted > -56)) // dieu chinh goc
    {
      Kp = Kp_user;            // Default user control gains     Tăng kiểm soát người dùng mặc định
      Kd = Kd_user;
      Kp_thr = Kp_thr_user;
      Ki_thr = Ki_thr_user;
    }
    else    // We are in the raise up procedure => we use special control parameters   Chúng tôi đang trong quy trình nâng lên => chúng tôi sử dụng các tham số điều khiển đặc biệt
    {
      Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
    }

  } // End of new IMU data
//      Serial.print(motor1);
//      Serial.print("\t");
//      Serial.print(motor2);
//      Serial.print("\t");
                              //     Serial.println(angle_adjusted);
//      Serial.print("\t");
//      Serial.print("\t");
//      Serial.print(Kd);
//      Serial.print("\t");
//      Serial.print(posx);
//      Serial.print("\t");
//      Serial.print(posy);
//      Serial.println(" ");

//      Serial.print(KpX);
//      Serial.print("\t");
//      Serial.print(KiX);
//      Serial.print("\t");
//      Serial.print(KdX);
//      Serial.print("\t");
//      Serial.print("\t");
//      Serial.print(OutputX);
//      Serial.print("\t");
//      Serial.print(OutputY);
//      Serial.print("\t");
//      Serial.print("\t");
//      Serial.print(posx);
//      Serial.print("\t");
//      Serial.print(posy);
//      
//      Serial.println(" ");

}
