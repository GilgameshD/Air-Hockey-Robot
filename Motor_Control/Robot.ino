// Each time a new data packet from camera is reveived this function is called

void RobotMovement()
{
  int move_x = 0, move_y = 0;
  if(puckPixX >= 180)
  {
    int destination_x;// = (abs(puckPixX-robotPixX)+abs(puckPixY-robotPixY)) <  (abs(predict_x-robotPixX)+abs(predict_y-robotPixY))? puckPixX:predict_x;
    int destination_y;// = (abs(puckPixX-robotPixX)+abs(puckPixY-robotPixY)) <  (abs(predict_x-robotPixX)+abs(predict_y-robotPixY))? puckPixY:predict_y;
    if(puckPixX > 380)
    {
        destination_x = (abs(puckPixX-robotPixX)+abs(puckPixY-robotPixY)) <  (abs(predict_x-robotPixX)+abs(predict_y-robotPixY))? puckPixX:predict_x;
        destination_y = (abs(puckPixX-robotPixX)+abs(puckPixY-robotPixY)) <  (abs(predict_x-robotPixX)+abs(predict_y-robotPixY))? puckPixY:predict_y;
    }
    else if (predict_x != 0 && predict_y != 285)
    {
        destination_x = predict_x;
        destination_y = predict_y;
    }
    else 
    {
        destination_x = oriPositionX;
        destination_y = oriPositionY;  
     }
      // 求出需要移动的二维向量（图像坐标方向）
      move_x = (predict_x - robotPixX)/MM_TO_CONTROL_X;
      move_y = (predict_y - robotPixY)/MM_TO_CONTROL_Y;
      
      com_pos_x = (robotPixX - oriPositionX - (com_pos_x - 190)*MM_TO_CONTROL_X)/MM_TO_CONTROL_X + com_pos_x;
      com_pos_y = (robotPixY - oriPositionY - (com_pos_y - 210)*MM_TO_CONTROL_Y)/MM_TO_CONTROL_Y + com_pos_y;
      //position_x = com_pos_x;
      //position_y = com_pos_y;
      destination_x = destination_x > 536 ? 536 : destination_x;
      destination_y = destination_y > 269 ? 269 : destination_y;
      destination_y = destination_y < 12 ? 12 : destination_y;
      setPosition((destination_x - oriPositionX - ( - 190)*MM_TO_CONTROL_X)/MM_TO_CONTROL_X,
                  (destination_y - oriPositionY - ( - 210)*MM_TO_CONTROL_Y)/MM_TO_CONTROL_Y);
    //setPosition(move_x + com_pos_x, move_y + com_pos_y);
    
  }
  else
  {
    // 返回到初始位置
    move_x = (oriPositionX - robotPixX)/MM_TO_CONTROL_X;
    move_y = (oriPositionY - robotPixY)/MM_TO_CONTROL_Y;
    com_pos_x = (robotPixX - oriPositionX - (com_pos_x - 190)*MM_TO_CONTROL_X)/MM_TO_CONTROL_X + com_pos_x;
    com_pos_y = (robotPixY - oriPositionY - (com_pos_y - 210)*MM_TO_CONTROL_Y)/MM_TO_CONTROL_Y + com_pos_y;
    //position_x = com_pos_x * 19;//com_pos_x;
    //position_y = com_pos_y * 19;//com_pos_y;
    //setPosition(com_pos_x + move_x, com_pos_y + move_y);
    setPosition(190,210);
    CheckMissing();
  }
}

void CheckMissing()
{ 
  // 如果已经停止 
  if(speed_x == 0 && speed_y == 0)
  {
    // 位置发生了偏移进行校正。
    robotX += robotPixX;
    robotY += robotPixY;
    if (robotAvgCount == 9) 
    {
      robotX = robotX / 10;
      robotY = robotY / 10; 
      if(abs(robotX - oriPositionX) > 10 || abs(robotY - oriPositionY) > 10)
      {
      //com_pos_x = (robotPixX - 509 - (com_pos_x - 200)*MM_TO_CONTROL_X)/MM_TO_CONTROL_X + com_pos_x;
      //com_pos_y = (robotPixY - 146 - (com_pos_y - 210)*MM_TO_CONTROL_Y)/MM_TO_CONTROL_Y + com_pos_y;
        position_x = ((robotX - oriPositionX)/MM_TO_CONTROL_X + 190)*19;
        position_y = ((robotY - oriPositionY)/MM_TO_CONTROL_Y + 210)*19;
        setPosition(190, 210);
      }
      robotAvgCount = 0;
      robotX = 0;
      robotY = 0;
    }
    else 
      robotAvgCount++;
  }
}



