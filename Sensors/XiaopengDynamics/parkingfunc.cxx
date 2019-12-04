#include "parkingfunc.hpp"
#define grid_space 0.01 


float getangle(float pointAx, float pointAy, float pointBx, float pointBy)
{
  float angle = atan2((pointBy - pointAy), (pointBx - pointAx));
  return angle;
}

float gedis(float pointAx, float pointAy, float pointBx, float pointBy)
{
  float dis = sqrt(pow(pointAx - pointBx, 2) + pow(pointAy - pointBy, 2));
  return dis;
}

float drawLine(float pointAx, float pointAy, float pointBx, float pointBy, std::vector<RoadPoint> &partpointList)
{
  float dis = gedis(pointAx, pointAy, pointBx, pointBy);
  int index = (int)(dis / grid_space);
  RoadPoint tempapspoint;
  for (int i = 0; i < index; i++)
  {
    tempapspoint.x = pointBx + (pointAx - pointBx) * i / index;
    tempapspoint.y = pointBy + (pointAy - pointBy) * i / index;
    tempapspoint.curve = 1.0e10;
    partpointList.push_back(tempapspoint);
  }
  return 0;
}

float drawArc(float x1, float y1, float alpha1, float x2, float y2, float alpha2, float &x, float &y, std::vector<RoadPoint> &partpointList)
{
  YuanXin(x1, y1, alpha1, x2, y2, alpha2, x, y);
  float temp_R = getR(x1, y1, alpha1, x2, y2, alpha2);
  float LL = temp_R * (fabs(alpha1 - alpha2));
  int index = (int)(LL / grid_space);
  RoadPoint tempapspoint;
  for (int i = 0; i < index; i++)
  {
    tempapspoint.x = x + temp_R * cos(alpha1 + (alpha2 - alpha1) * i / index); 
    tempapspoint.y = y + temp_R * sin(alpha1 + (alpha2 - alpha1) * i / index);
    tempapspoint.curve = temp_R;   
    if ((alpha1 - alpha2) > pi_aps) 
    {
      tempapspoint.curve = -tempapspoint.curve;
    }
    else if ((alpha1 - alpha2) < -pi_aps) 
    {
    }
    else
    {
      if (alpha1 < alpha2) 
      {
        tempapspoint.curve = -tempapspoint.curve;
      }
      else
      {
        tempapspoint.curve = tempapspoint.curve;
      }
    }
    partpointList.push_back(tempapspoint);
  }
  return 0;
}

float drawBezier2(float x1, float y1, float alpha1, float x2, float y2, float alpha2, float indexL ,std::vector<RoadPoint> &partpointList)
{
  float P[4][2];
 // ROS_INFO("float x1 [[%f]], float y1[%f], float alpha1[%f], float x2[%f], float y2[%f], float alpha2[%f]", x1, y1, alpha1, x2, y2, alpha2);
  P[0][0] = x1;
  P[0][1] = y1;
  P[3][0] = x2;
  P[3][1] = y2;
  float L_p = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  P[1][0] = x1 - L_p *indexL * cos(alpha1); 
  P[1][1] = y1 - L_p *indexL  * sin(alpha1);
  P[2][0] = x2 + L_p *indexL * cos(alpha2); 
  P[2][1] = y2 + L_p *indexL  * sin(alpha2);
  //ROS_INFO("P0:[%f][%f]", P[0][0], P[0][1]);
  //ROS_INFO("P1:[%f][%f]", P[1][0], P[1][1]);
  //ROS_INFO("P2:[%f][%f]", P[2][0], P[2][1]);
  int n = 3;
  int index = L_p / grid_space;
  RoadPoint tempapspoint;
  for (int i = 1; i < index; i++)
  {
    float x = 0;
    float y = 0;
    float ddxdtt = 0;
    float ddydtt = 0;
    float dxdt = 0;
    float dydt = 0;
    float dy = 0;
    float ddy = 0;
    float K = 0;
    float R_curve = 100000000;
    int j = 0;
    for (j = 0; j <= n; j++)
    {
      x = x + P[j][0] * Cij(n, j) * pow((1 - (float)i / (float)index), (n - j)) * pow((float)i / (float)index, j);
      float xxx = pow((1 - ((float)i / (float)index)), (n - j));
      y = y + P[j][1] * Cij(n, j) * pow((1 - (float)i / (float)index), (n - j)) * pow((float)i / (float)index, j);
      ddxdtt = ddxdtt + P[j][0] * Cij(n, j) * (-(n - j) * (-(n - j - 1) * pow((1 - (float)i / (float)index), (n - j - 2)) * pow((float)i / (float)index, j) + j * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j - 1)) + j * (-(n - j) * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j - 1) + (j - 1) * pow((1 - (float)i / (float)index), (n - j)) * pow((float)i / (float)index, (j - 2))));
      ddydtt = ddydtt + P[j][1] * Cij(n, j) * (-(n - j) * (-(n - j - 1) * pow((1 - (float)i / (float)index), (n - j - 2)) * pow((float)i / (float)index, j) + j * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j - 1)) + j * (-(n - j) * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j - 1) + (j - 1) * pow((1 - (float)i / (float)index), (n - j)) * pow((float)i / (float)index, (j - 2))));
      dxdt = dxdt + P[j][0] * Cij(n, j) * (-(n - j) * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j) + j * pow((1 - (float)i / (float)index), n - j) * pow((float)i / (float)index, j - 1));
      dydt = dydt + P[j][0] * Cij(n, j) * (-(n - j) * pow((1 - (float)i / (float)index), (n - j - 1)) * pow((float)i / (float)index, j) + j * pow((1 - (float)i / (float)index), n - j) * pow((float)i / (float)index, j - 1));
    }
    dy = dydt / dxdt;
    ddy = (ddydtt * dxdt - ddxdtt * dydt) / pow(dxdt, 3);
    K = ddy / pow((1 + pow(dy, 2)), 3 / 2);
    R_curve = 1 / K;
    tempapspoint.x = x;
    tempapspoint.y = y;
    tempapspoint.curve = R_curve;
    partpointList.push_back(tempapspoint);
  }
}

int changecarstatis(int status)
{
  if (status == 0)
  {
    status = 1;
  }

  else if (status == 1)
  {
    status = 2;
  }
  else if (status == 2)
  {
    status = 3;
  }
  else
  {
    std::cout << "wrong status" << std::endl;
  }
  return status;
}

void YuanXin(float x1, float y1, float alpha1, float x2, float y2, float alpha2, float &x, float &y)
{

  float R = getR(x1, y1, alpha1, x2, y2, alpha2);
  float c1 = (x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1) / (2 * (x2 - x1));
  float c2 = (y2 - y1) / (x2 - x1);
  float A = (c2 * c2 + 1);
  float B = (2 * x1 * c2 - 2 * c1 * c2 - 2 * y1);
  float C = x1 * x1 - 2 * x1 * c1 + c1 * c1 + y1 * y1 - R * R;
  float y_1 = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
  float y_2 = (-B - sqrt(B * B - 4 * A * C)) / (2 * A);
  float x_1 = c1 - c2 * y1;
  float x_2 = c1 - c2 * y2;
  float deltax = fabs(x_1 - x1);
  float deltay = fabs(y_1 - y1);
  float delt_angle = fabs(alpha1 - atan2(deltay, deltax));
  if (delt_angle > 2 * pi_aps)
  {
    delt_angle = 2 * pi_aps - delt_angle;
  }
  if (delt_angle > 998 * pi_aps / 2000 && delt_angle < 1002 * pi_aps / 2000)
  {
    x = x_1;
    y = y_1;
  }
  else
  {
    x = x_2;
    y = y_2;
  }
}

float getR(float x1, float y1, float alpha1, float x2, float y2, float alpha2) 
{
  float delta_a = fabs(alpha1 - alpha2); 
  if (delta_a > pi_aps)
  {
    delta_a = 2 * pi_aps - delta_a;
  }
  float L_temp = gedis(x1, y1, x2, y2);
  float R = L_temp / (2 * sin(delta_a / 2));
  return R;
}

void controlstr(std::vector<RoadPoint> &apspoint, float x, float y, float heading, float &strangle, int &count_now) 
{
  float Kp = 10; 
  float L_min = 3;
  float x_min = -1.0;
  float y_min = -1.0;
  float pdeng = 14.24;      
  float G;
  float L_s=0.0;
  int count_pre;
  float tempstrangle=0.0;
  float angleerror=0;
  for (int count = 0; count < apspoint.size() - 1; count++)
  {
    float L = sqrt((x - apspoint[count].x) * (x - apspoint[count].x) + (y - apspoint[count].y) * (y - apspoint[count].y));
    if (L <= L_min)
    {

      L_min = L;
      x_min = apspoint[count].x;
      y_min = apspoint[count].y;
      count_now = count;
    }
    else
    {
      L_min = L_min;
      x_min = x_min;
      y_min = y_min;
      count_now = count_now;
    }
  } 
   //ROS_INFO(" count_now==[%d]",count_now);

  // float L_s=0.0;
  int countnow;
  for(countnow=count_now;countnow<apspoint.size() - 1;countnow++)
  {
    float L_s_temp=sqrt((apspoint[countnow+1].x - apspoint[countnow].x) * (apspoint[countnow+1].x - apspoint[countnow].x) + (apspoint[countnow+1].y - apspoint[countnow].y) * (apspoint[countnow+1].y - apspoint[countnow].y));
    L_s+=L_s_temp;

    if (L_s>=0.6)
    { 
      break;
    }  
  }
  count_pre=countnow;
 // ROS_INFO(" count_pre==[%d]",count_pre); 
  //float x1 = apspoint[count_now + 5].x - apspoint[count_now].x;
  //float x2 = x - apspoint[count_now].x;
  //float y1 = apspoint[count_now + 5].y - apspoint[count_now].y;
  //float y2 = y - apspoint[count_now].y;
  float x1 = x - apspoint[count_pre].x;
  float x2 = apspoint[count_now].x - apspoint[count_pre].x;
  float y1 = y - apspoint[count_pre].y;
  float y2 = apspoint[count_now].y - apspoint[count_pre].y;;
  if ((x1 * y2 - x2 * y1) < 0) 
  {
    L_min = L_min;
    std::cout << " left L_min=" << L_min << std::endl;
  }
  else
  { 
    L_min = -L_min;
    std::cout << " right  L_min=" << L_min << std::endl;
  }
 //ROS_INFO(" L_min==[%f]",L_min);
 // ROS_INFO("heading_=[%f],roadangle_now=[%f],roadangle_pre=[%f]",heading_vehicle,apspoint[count_now].Road_angle,apspoint[count_pre].Road_angle);
  //tempstrangle = atan(h_m / (apspoint[count_now].curve));
  tempstrangle = atan(h_m / (apspoint[count_pre].curve));  
  if (strangle >= 0)
    {
      G = pdeng;
    }
  else
  {
    G = pdeng;
  }
  tempstrangle = tempstrangle * G; 
  //ROS_INFO(" tempstrangle out ==[%f]",tempstrangle * (180 / pi_aps));
  //tempstrangle = tempstrangle * (180 / pi_aps)+500*L_min+10*angleerror;
  tempstrangle = tempstrangle * (180 / pi_aps);
  strangle=tempstrangle;
  //ROS_INFO(" strangle out ==[%f]",strangle);
}

float ControlStr2(std::vector<RoadPoint> &apspoint, float _In_X_Position, float _In_Y_Position, float _In_Heading_Vehicle, int &count_now, float _In_Speed_Ego_Vehicle)
{
  float Kp = 10;
  float L_min = 10.0;
  float x_min = -1.0;
  float y_min = -1.0;
  float _Var_WheelToSteer_Pdeng = 14.24;
  float _G;
  float _Glo_Mass_Vehicle = 1705;
  float _Glo_FrontWheelDis_Vehicle = 1.255;
  float _Glo_BackWheelDis_Vehicle = 1.535;
  float _Glo_FrontCore_Vehicle = 116000;
  float _Glo_RearCore_Vehicle = 187000;
  float _Cal_Denominator;
  float _Cal_Molecular;
  float _Cal_Tran_Variable;
  float _Cal_Trans_Val;
  float _Out_TempStrangle_Steer = 0.0;
  float _D_Dis_Tmp = 0.8;
  //float _D_Dis_Tmp = 1.0;
  float _X_Preview = _In_X_Position + _D_Dis_Tmp * cos((_In_Heading_Vehicle + 180) / 180 * pi_aps);
  float _Y_Preview = _In_Y_Position + _D_Dis_Tmp * sin((_In_Heading_Vehicle + 180) / 180 * pi_aps);
  for (int count = 0; count < apspoint.size(); count++)
  {
    float L = sqrt((_X_Preview - apspoint[count].x) * (_X_Preview - apspoint[count].x) + (_Y_Preview - apspoint[count].y) * (_Y_Preview - apspoint[count].y));
    if (L <= L_min)
    {
      L_min = L;
      x_min = apspoint[count].x;
      y_min = apspoint[count].y;
      count_now = count;
    }
    else
    {
      L_min = L_min;
      x_min = x_min;
      y_min = y_min;
      count_now = count_now;
    }
  }
  if ((_X_Preview - _In_X_Position) * (y_min - _In_Y_Position) - (_Y_Preview - _In_Y_Position) * (x_min - _In_X_Position) >= 0)
    L_min = -1.0 * L_min;
  else
    L_min = L_min;
  //ROS_INFO("L_min [%f]", L_min);
  _Cal_Molecular = 2 * (_Glo_FrontWheelDis_Vehicle + _Glo_BackWheelDis_Vehicle - (_Glo_Mass_Vehicle * (_Glo_FrontWheelDis_Vehicle * _Glo_FrontCore_Vehicle - _Glo_BackWheelDis_Vehicle * _Glo_RearCore_Vehicle)) / ((_Glo_FrontWheelDis_Vehicle + _Glo_BackWheelDis_Vehicle) * _Glo_FrontCore_Vehicle * _Glo_RearCore_Vehicle)); 
  _Cal_Tran_Variable = _Glo_BackWheelDis_Vehicle - ((_Glo_FrontWheelDis_Vehicle * _Glo_Mass_Vehicle * (_In_Speed_Ego_Vehicle * _In_Speed_Ego_Vehicle)) / (_Glo_RearCore_Vehicle * (_Glo_FrontWheelDis_Vehicle + _Glo_BackWheelDis_Vehicle)));
  _Cal_Denominator = _D_Dis_Tmp * (_D_Dis_Tmp + 2 * _Cal_Tran_Variable);
  float _Cal_Trans_Par = _Cal_Molecular / _Cal_Denominator;
  //_Cal_Trans_Par=0.3;
  //ROS_INFO("_In_Speed_Ego_Vehicle:%lf",_In_Speed_Ego_Vehicle);
  //ROS_INFO("_Cal_Trans_Par:%lf",_Cal_Trans_Par);
  //重新定义分子分母
  //	_Cal_Molecular = 1/sqrt((_R_car*_R_car-_Cal_Tran_Variable*_Cal_Tran_Variable))*2 * (_Glo_FrontWheelDis_Vehicle + _Glo_BackWheelDis_Vehicle - (_Glo_Mass_Vehicle * (_Glo_FrontWheelDis_Vehicle * _Glo_FrontCore_Vehicle - _Glo_BackWheelDis_Vehicle * _Glo_RearCore_Vehicle)) / ((_Glo_FrontWheelDis_Vehicle + _Glo_BackWheelDis_Vehicle) * _Glo_FrontCore_Vehicle * _Glo_RearCore_Vehicle));
  //	_Cal_Denominator = sqrt(_D_Dis_Tmp*_D_Dis_Tmp + _R_car*_R_car + 2 * _D_Dis_Tmp*_Cal_Tran_Variable) - _R_car;
  //	_Out_Angle_Steer = _Cal_Molecular / _Cal_Denominator*L_min*(180 / pi)*G;
  // if (_Out_Angle_Steer >= 0)
  // {
  //   _G = _Var_WheelToSteer_Pdeng;
  // }
  // else
  // {
  //   _G = _Var_WheelToSteer_Pdeng;
  // }
  _Out_TempStrangle_Steer = _Cal_Trans_Par * L_min;
  _Out_TempStrangle_Steer = _Out_TempStrangle_Steer * (180 / pi_aps) * _G;
  return _Out_TempStrangle_Steer;
}


float Cij(int n, int j)
{

  int a = 1;
  int b = 1;
  int c = 1;
  int i = 0;
  for (i = 1; i <= n; i++)
    a = a * i;
  for (i = 1; i <= j; i++)
    b = b * i;
  for (i = 1; i <= (n - j); i++)
    c = c * i;

  float cij = float(a / (b * c));
  return cij;
}

void OptArconlineright(float *ArcKMap)
{
  float xmax = 20.0 + WW / 2;
  float ymax = 20 + LLL;
  float anglemax = pi_aps;
  float xnum = 100.0;
  float xgrid = 20.0 / xnum;
  float ynum = 100.0;
  float ygrid = 20.0 / ynum;
  float anglenum = 29.0;
  float anglegrid = pi_aps/30.0;
  float x1 = WW / 2;
  float y1 = LLL;  
  float angle1 = pi_aps / 2;
  float x2;
  float y2;
  float angle2;
  float Rc3temp;
  float thetaRc3;
  float R, S;
  for (int countx2 = 0; countx2 < xnum; countx2++)
  {
    for (int county2 = 0; county2 < ynum; county2++)
    {
      for (int countangle = 0; countangle < anglenum; countangle++)
      {
        x2 = (countx2+1) * xgrid + x1;
        y2 = (county2+1) * ygrid + y1;
        angle2 = float(countangle+1) * anglegrid - anglemax / 2;
        if (sin(angle2)==1)
        {
          angle2=pi_aps/2-0.01;
        }
        R = (x2 - x1) / (1 - sin(angle2));
        S = y2 - R * cos(angle2);
        if (R >= ((-WW * WW / 4 - pow((LLL - S), 2) + pow((W_a / 2 + delta_2), 2)) / ( - WW +W_a+ 2 * delta_2)))
        {
          Rc3temp = LLL - S + DD - delta_3;
          thetaRc3 = asin((L_a - L_r) / Rc3temp);
          if ((R <= Rc3temp * cos(thetaRc3) - W_a) &&( R >= R_min) && (S> (L_r+delta_1)))
          {
            //ROS_INFO("FIND GOOD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R;
          }
          else
          {
            *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R_min;
          }
        }
        else
        {
          *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R_min;
        }
        
      }
    }
  }
}
void OptArconlineleft(float *ArcKMap)
{
  float xmax = -20.0 + WW / 2;
  float ymax = 20 + LLL;
  float anglemax = pi_aps;
  float xnum = 100.0;
  float xgrid = 20.0 / xnum;
  float ynum = 100.0;
  float ygrid = 20.0 / ynum;
  float anglenum = 29.0;
  float anglegrid = pi_aps/30.0;
  float x1 = WW / 2;
  float y1 = LLL;  
  //float angle1 = pi_aps / 2;
  float x2;
  float y2;
  float angle2;
  float Rc3temp;
  float thetaRc3;
  float R, S;
  for (int countx2 = 0; countx2 < xnum; countx2++)
  {
    for (int county2 = 0; county2 < ynum; county2++)
    {
      for (int countangle = 0; countangle < anglenum; countangle++)//pi/2-3pi/2
      {
        x2 = -(countx2+1) * xgrid + x1;
        y2 = (county2+1) * ygrid + y1;
        angle2 = float(countangle+1) * anglegrid + anglemax / 2;
        if (sin(angle2)==-1)
        {
          angle2=-pi_aps/2-0.01;
        }
        R = fabs(x2 - x1) / (1 - sin(angle2));
        S = y2 + R * cos(angle2);
        if (R >= ((-WW * WW / 4 - pow((LLL - S), 2) + pow((W_a / 2 + delta_2), 2)) / ( - WW +W_a+ 2 * delta_2)))
        {
          //ROS_INFO("FIND GOOD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          Rc3temp = LLL - S + DD - delta_3;
          thetaRc3 = asin((L_a - L_r) / Rc3temp);
          if ((R <= Rc3temp * cos(thetaRc3) - W_a) &&( R >= R_min) && (S> (L_r+delta_1)))
          {
            *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R;
             //ROS_INFO("FIND GOOD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          }
          else
          {
            *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R_min;
            
          }
        }
        else
        {
          *(ArcKMap + 2900 * countx2 + county2 * 29 + countangle) = 1 / R_min;
        }
       //ROS_INFO("ArcKMap[%d] POINT[%f]",countx2,ArcKMap[countx2][county2][countangle]);
      }
    }
  }
}
void generateCurveList2(int control,float  ControlPoint2[8], Car_Status &startPoint,  std::vector<RoadPoint>  &apspointList)
{
  bool getendpoint = false;
  int status = 0;
  if (control == 0)
  {
    std::cout << "wrong contril model ::wrong position" << std::endl;
  }
  else
  {
    while (!getendpoint)
    {

      if (status == 0)
      {
        status = changecarstatis(startPoint.status);
      }
      else
      {
        status = changecarstatis(status);
      }
      int index;
      float dis;
      RoadPoint tempapspoint;
      float a = 0;
      float b = 0;
      switch (status)
      {
      case 1:
        //drawLine(ControlPoint[0][2], ControlPoint[1][2] + 2.0, ControlPoint[0][1], ControlPoint[1][1], apspointList);

        drawBezier2(startPoint.x, startPoint.y, startPoint.angle * pi_aps / 180, ControlPoint2[0], ControlPoint2[1], ControlPoint2[6], ControlPoint2[7],apspointList); 
        
        break;
      case 2:
        drawArc(ControlPoint2[0], ControlPoint2[1], pi_aps / 2+ControlPoint2[6], ControlPoint2[2], ControlPoint2[3], pi_aps, a, b, apspointList); 
        break;
      case 3:
        drawLine(ControlPoint2[4], ControlPoint2[5]-2, ControlPoint2[2], ControlPoint2[3], apspointList); 
        getendpoint = true;
        break;
      }
    }
  }
}
void generateCurveList2left(int control,float  ControlPoint2[8], Car_Status &startPoint,  std::vector<RoadPoint>  &apspointList)
{
  bool getendpoint = false;
  int status = 0;
  if (control == 0)
  {
    std::cout << "wrong contril model ::wrong position" << std::endl;
  }
  else
  {
    while (!getendpoint)
    {

      if (status == 0)
      {
        status = changecarstatis(startPoint.status);
      }
      else
      {
        status = changecarstatis(status);
      }
      int index;
      float dis;
      RoadPoint tempapspoint;
      float a = 0;
      float b = 0;
      switch (status)
      {
      case 1:
        drawBezier2(startPoint.x, startPoint.y, startPoint.angle * pi_aps / 180, ControlPoint2[0], ControlPoint2[1], ControlPoint2[6], ControlPoint2[7],apspointList); 
        break;
      case 2:
        drawArc(ControlPoint2[0], ControlPoint2[1], -pi_aps / 2+ControlPoint2[6], ControlPoint2[2], ControlPoint2[3], 0, a, b, apspointList); 
        break;
      case 3:
        drawLine(ControlPoint2[4], ControlPoint2[5]-2, ControlPoint2[2], ControlPoint2[3], apspointList); 
        getendpoint = true;
        break;
      }
    }
  }
}
float controldelay(float back,float Tdelay,int rate , float wheelout[10])
{
  int cycle;
  float tempout;
  float K;

  K=1-exp(-1/(rate*0.6));
  cycle=int(Tdelay*rate)+1;
  printf("cycle=[%d]\n",cycle);
  tempout=(wheelout[cycle-1]-back)*K+back;
  printf("cycle1=[%f]\n",tempout);
  for (int i=cycle-2 ;i>0;i--)
  {
    tempout=(wheelout[i]-tempout)*K+tempout;
    printf("cycle[2]=[%f]\n",tempout);
  }
  return tempout;
}
