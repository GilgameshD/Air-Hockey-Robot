// ROBOT and USER configuration parameters
#include "Configuration.h"
#include "Definitions.h"   

void setup()
{
  // X MOTOR
  //     X-STEP: A0    (PF0)
  //     X-DIR:  A1    (PF1)
  //     X-ENABLE: D38 (PD7)
  // Y MOTOR (Y-LEFT)
  //     Y-STEP: A6    (PF6)
  //     Y-DIR:  A7    (PF7)
  //     Y-ENABLE: A2  (PF2)
  
  // STEPPER PINS
  // X_AXIS
  pinMode(38, OUTPUT); // ENABLE MOTOR
  pinMode(A0, OUTPUT); // STEP MOTOR
  pinMode(A1, OUTPUT); // DIR MOTOR
  // Y_AXIS (Y-LEFT)
  pinMode(A2, OUTPUT); // ENABLE MOTOR
  pinMode(A6, OUTPUT); // STEP MOTOR
  pinMode(A7, OUTPUT); // DIR MOTOR

  pinMode(A3, OUTPUT); // DEBUG PIN FOR OSCILLOSCOPE TIME MEASURES

  pinMode(19, INPUT);  // RX1 Serial Port 1
  pinMode(18, OUTPUT); // TX1

  //FANS and LEDS
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);

  // Disable Motors
  digitalWrite(38, HIGH);
  digitalWrite(A2, HIGH);

  Serial.begin(115200);
  Serial.println("Initializing robot...");
  delay(100);

  // Robot positions initialization
  defense_position = ROBOT_DEFENSE_POSITION;         // Robot y axis defense position
  attack_position = ROBOT_DEFENSE_ATTACK_POSITION;   // Robot y axis position for defense+attack

  // 指示灯闪三下
  for (uint8_t k = 0; k < 3; k++)
  {
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }

  // We use TIMER 1 for stepper motor X AXIS and Timer 3 for Y AXIS
  // TIMER1 CTC MODE
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  OCR1A = ZERO_SPEED;   // Motor stopped
  dir_x = 0;
  TCNT1 = 0;

  // We use TIMER 3 for stepper motor Y AXIS
  // TIMER3 CTC MODE
  TCCR3B &= ~(1 << WGM13);
  TCCR3B |=  (1 << WGM12);
  TCCR3A &= ~(1 << WGM11);
  TCCR3A &= ~(1 << WGM10);

  // output mode = 00 (disconnected)
  TCCR3A &= ~(3 << COM1A0);
  TCCR3A &= ~(3 << COM1B0);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR3B = (TCCR3B & ~(0x07 << CS10)) | (2 << CS10);

  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_y = 0;
  TCNT3 = 0;

  //Initializing init position
  position_x = 190;//ROBOT_INITIAL_POSITION_X;
  position_y = 210;//ROBOT_INITIAL_POSITION_Y;
  delay(100);

  Serial.println("Initializing Stepper motors...");
  delay(100);
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer3 interrupt

  // Enable steppers
  digitalWrite(38, LOW);  // X-axis
  digitalWrite(A2, LOW);  // Y-axis

  // Output parameters
  Serial.print("Max_acceleration_x: ");
  Serial.print(max_acceleration_x);
  delay(10);
  Serial.print("Max_acceleration_y: ");
  Serial.print(max_acceleration_y);
  delay(10);
  Serial.print("Max speed X: ");
  Serial.print(MAX_SPEED_X);
  delay(10);
  Serial.print("Max speed Y: ");
  Serial.print(MAX_SPEED_Y);
  delay(10);
  Serial.println("Moving to initial position...");
  Serial.println("Ready!!");
  delay(1000);

  // 初始化位置
  com_pos_x = ROBOT_INITIAL_X;  // 100，X方向的初始位置
  com_pos_y = ROBOT_INITIAL_Y;  // 200，Y方向的初始位置
  com_speed_x = MAX_SPEED_X;
  com_speed_y = MAX_SPEED_Y;
  setSpeedS(com_speed_x, com_speed_y);
  setPosition(com_pos_x, com_pos_y);
  oriPositionX = 512; 
  oriPositionY = 144;
  //position_x = com_pos_x * 19;//com_pos_x;
  //position_y = com_pos_y * 19;//com_pos_y;

  timer_old = micros();
  timer_packet_old = timer_old;
  micros_old = timer_old;
}

void loop()
{
  int dt;
  uint8_t logOutput = 0;
  timer_value = micros();
  
  if ((timer_value - timer_old) >= 1000) // 1Khz 的循环执行速度
  {
      timer_old = timer_value;
      loop_counter++;
      packetRead(); // 接受串口数据并处理
      if(newPacket) // 获得新数据包
      {
          dt = 16;         //60 Hz = 16.66ms
          timer_packet_old = timer_value;
          newPacket = 0;   // 接受置为0
          logOutput = 1;   // 输出置为1
          RobotMovement(); // 设置移动数据
      }
      // 串口数据输出
      if(logOutput)
      {
        logOutput = 0;
        Serial.println("-----------------");
        Serial.print("puckPixX: ");
        Serial.println(puckPixX);
        Serial.print("puckPixY: ");
        Serial.println(puckPixY);
        Serial.print("robotPixX: ");
        Serial.println(robotPixX);
        Serial.print("robotPixY: ");
        Serial.println(robotPixY);
        Serial.print("predict_x: ");
        Serial.println(predict_x);
        Serial.print("predict_y: ");
        Serial.println(predict_y);
        Serial.print("position_x: ");
        Serial.println(position_x);
        Serial.print("position_y: ");
        Serial.println(position_y);
      }
      positionControl();   // 执行操作
    } 
   
   // 测试代码，可以得到X与Y方向的最大移动距离
   // X方向为 200 时可以移动,（516 - 385）pix/200 = 0.655
   // Y方向为 100 时可以移动,（132 - 17） pix/200 = 0.575
   // X方向是正常的，Y方向是反向的
   /*
   if ((timer_value-timer_old) >= 1000)  // 1Khz loop
   {
    //dt = (timer_value-timer_old)/1000;
    timer_old = timer_value;
    loop_counter++;

    packetRead(); // 接受串口数据并处理
      if(newPacket) // 获得新数据包
      {
          dt = 16;         //60 Hz = 16.66ms
          timer_packet_old = timer_value;
          newPacket = 0;   // 接受置为0
          logOutput = 1;   // 输出置为1
          //RobotMovement(); // 设置移动数据
      }
      // 串口数据输出
      if(logOutput)
      {
        logOutput = 0;
        Serial.println("-----------------");
        Serial.print("puckPixX: ");
        Serial.println(puckPixX);
        Serial.print("puckPixY: ");
        Serial.println(puckPixY);
        Serial.print("robotPixX: ");
        Serial.println(robotPixX);
        Serial.print("robotPixY: ");
        Serial.println(robotPixY);
        Serial.print("predict_x: ");
        Serial.println(predict_x);
        Serial.print("predict_y: ");
        Serial.println(predict_y);
      }

      
    if (loop_counter == 1000)
      {
        com_pos_x  += 200;
        setPosition(com_pos_x,com_pos_y);
        setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
      }
    if (loop_counter == 2000)
      {
        com_pos_y += 200;
        setPosition(com_pos_x,com_pos_y); 
        setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
        //loop_counter = 0;
      }
     if (loop_counter == 3000)
      {
        com_pos_x  -= 200;
        setPosition(com_pos_x,com_pos_y);
        setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
      }
    if (loop_counter == 4000)
      {
        com_pos_y -= 200;
        setPosition(com_pos_x,com_pos_y); 
        setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
        loop_counter = 0;
      }
      positionControl();
  } // 1Khz loop
  */
}





