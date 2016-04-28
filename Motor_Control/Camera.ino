// Camera serial packet read

// 将相邻的两个数据的高8位和低8位合并成一个
uint16_t extractParamInt(uint8_t pos)
{
  union
  {
    unsigned char Buff[2];
    uint16_t d;
  }u;

  u.Buff[0] = (unsigned char)SBuffer[pos];
  u.Buff[1] = (unsigned char)SBuffer[pos+1];
  return(u.d); 
}

// 从串口读取上位机数据并解析
void packetRead()
{
  unsigned char i;
  if (Serial.available() > 0) 
  {
      // We rotate the Buffer
      for (i = 13;i > 0;i--)
          SBuffer[i] = SBuffer[i-1];
      SBuffer[0] = Serial.read();

      // 只有收到起始位时才开始解析数据包
      if((SBuffer[0] == 'm')&&(SBuffer[1] == 'm'))
      {
          if(readStatus == 0)
          {
              readStatus = 1;
              readCounter = 14; 
          }
          else
          {
              readStatus = 1;
              readCounter = 14;
          }
          return;
      }
      else if(readStatus == 1)
      {
          --readCounter;         // 直到将所有的数据包接受完
          if(readCounter <= 0)   // 接受完成
          {
            cam_timestamp = extractParamInt(12);
            puckPixX = extractParamInt(10);
            puckPixY = extractParamInt(8);
            robotPixX = extractParamInt(6);
            robotPixY = extractParamInt(4);  
            predict_x = extractParamInt(2);  
            predict_y = extractParamInt(0);
            
            readStatus = 0;  // 接受状态位置0
            newPacket = 1;   // 标志位置1
        }
      }
  }
}

void missingStepsDetection()
{
  int robot_position_x_mm;
  int robot_position_y_mm;

  if ((speed_x == 0) && (speed_y == 0))
  {
    robot_position_x_mm = position_x/X_AXIS_STEPS_PER_UNIT;
    robot_position_y_mm = position_y/Y_AXIS_STEPS_PER_UNIT;

    robotCoordSamples++;
    robotCoordXAverage += robotCoordX;
    robotCoordYAverage += robotCoordY;
    // When we collect 10 samples we make the correction
    if (robotCoordSamples == 10)
    {
      // X axis
      robotCoordXAverage = robotCoordXAverage/robotCoordSamples;
      robotMissingStepsErrorX = myAbs(robot_position_x_mm - robotCoordXAverage);  // in milimeters)
      if (robotMissingStepsErrorX > 8) 
      {
        position_x = robotCoordXAverage * X_AXIS_STEPS_PER_UNIT;
        Serial.print("MSX ");
        Serial.println(robotMissingStepsErrorX);
      }
      // Y AXIS
      robotCoordYAverage = robotCoordYAverage/robotCoordSamples;
      robotMissingStepsErrorY = myAbs(robot_position_y_mm - robotCoordYAverage);
      if (robotMissingStepsErrorY > 10) 
      {
         position_y = robotCoordYAverage*Y_AXIS_STEPS_PER_UNIT;
         Serial.print("MSY ");
         Serial.println(robotMissingStepsErrorY);
      }
    }    
    else
    {
      robotCoordSamples = 0;
      robotCoordXAverage = 0;
      robotCoordYAverage = 0;
      robotMissingStepsErrorX = 0;
      robotMissingStepsErrorY = 0;
    }
  }
  else
  {
    robotCoordSamples = 0;
    robotCoordXAverage = 0;
    robotCoordYAverage = 0;
    robotMissingStepsErrorX = 0;
    robotMissingStepsErrorY = 0;
  }
}
