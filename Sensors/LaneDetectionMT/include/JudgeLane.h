#ifndef __JUDGE_LANE_H__
#define __JUDGE_LANE_H__

#include <iostream>

#include "LaneDetection.h"


#define LANE_POSITION_X 0

class CJudgeCenter
{
  public:
    CJudgeCenter() {
      _LaneWidth =3.5;
      _deltaLW=0.5;
      _orientation =0;
      _finish_flag =false;
    }
    virtual ~CJudgeCenter() {}

  public:
    void ReSort(std::vector<LD_COEFF> &tmp);
    void CalculateDistance(std::vector<LD_COEFF> &lane_coeff);
    void Evaluate();  //generate cost matric
  //  void Get
    void SetParam(float LaneWidth, float deltaLW,  float orientation);
    std::vector<LD_COEFF> DeleteRb(bool &finish_flag);
    void Run(std::vector<LD_COEFF> &tmp);
  private: 
    std::vector<LD_COEFF> _lane_coeff;
    float _orientation;
    float _LaneWidth;
    float _deltaLW;
    bool _finish_flag;  //完成标志
    cv::Mat D; //距离矩阵 distance
    cv::Mat C; //评价矩阵 cost
    cv::Mat L; //符合标准档位(1倍车道宽,2倍车道宽,3倍车道宽,-1 不满足)level
    cv::Mat B; //与标准的差值 bias
};

#endif

