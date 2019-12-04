#include "JudgeLane.h"

bool LessSort(LD_COEFF a, LD_COEFF b)
{
    float x =0;
    return (a.a*pow(x,3)+a.b*pow(x,2)+a.c*x+a.d) > (b.a*pow(x,3)+b.b*pow(x,2)+b.c*x+b.d);
}
bool MoreSort(float a, float b)
{
    return a < b;
}
void CJudgeCenter::SetParam(float LaneWidth, float deltaLW, float orientation)
{
    _LaneWidth = LaneWidth;
    _deltaLW = deltaLW;
    _orientation = orientation;
}
void CJudgeCenter::ReSort(std::vector<LD_COEFF> &lane_coeff)
{
    sort(lane_coeff.begin(), lane_coeff.end(), LessSort);
}

void CJudgeCenter::CalculateDistance(std::vector<LD_COEFF> &lane_coeff)
{
    _lane_coeff.clear();
   // sort(lane_coeff.begin(), lane_coeff.end(), LessSort);
    int dim = lane_coeff.size();
    float x = LANE_POSITION_X; //在车前LANE_POSITION_X处 计算车道线之间的距离

    D = cv::Mat::zeros(dim, dim, CV_32FC1);

    for (int ii = 0; ii < dim; ii++)
    {
        for (int jj = ii + 1; jj < dim; jj++)
        {
            D.at<float>(ii, jj) = fabs((lane_coeff[ii].a*pow(x,3)+lane_coeff[ii].b*pow(x,2)+lane_coeff[ii].c*x+lane_coeff[ii].d - lane_coeff[jj].a*pow(x,3)-lane_coeff[jj].b*pow(x,2)-lane_coeff[jj].c*x-lane_coeff[jj].d) * cos(_orientation));
        }
    }
    _lane_coeff.assign(lane_coeff.begin(),lane_coeff.end());
}

void CJudgeCenter::Evaluate()
{
    int indexSum = D.cols * (D.cols - 1) / 2;
    C = cv::Mat::zeros(indexSum, D.cols, CV_32FC1);
    L = cv::Mat::zeros(D.cols, D.cols, CV_32FC1);
    B = cv::Mat::zeros(D.cols, D.cols, CV_32FC1);
    int index = 0;
    for (int ii = 0; ii < D.cols; ii++)
    {
        for (int jj = ii + 1; jj < D.cols; jj++)
        {
            float cost = D.at<float>(ii, jj);
            if (((_LaneWidth - _deltaLW) < cost && cost < (_LaneWidth + _deltaLW)))
            {
                L.at<float>(ii, jj) = 1;
                B.at<float>(ii, jj) = fabs(cost - _LaneWidth);
            }
            else if (2*(_LaneWidth - _deltaLW) < cost && cost < 2*(_LaneWidth + _deltaLW))
            {
                L.at<float>(ii, jj) = 2;
                B.at<float>(ii, jj) = fabs(cost - 2*_LaneWidth);
            }else if (3*(_LaneWidth - _deltaLW) < cost && cost < 3*(_LaneWidth + _deltaLW))
            {
                L.at<float>(ii, jj) = 3;
                B.at<float>(ii, jj) = fabs(cost - 3 * _LaneWidth);
            }else{
                L.at<float>(ii, jj) = -1;
                B.at<float>(ii, jj) = 0;
            }
        }
    }
    // std::cout << "D:" <<std::endl;
    // std::cout << D<< std::endl;
    // std::cout << "L:" <<std::endl;
    // std::cout << L<< std::endl;
    // std::cout << "B:" <<std::endl;

    // std::cout << B << std::endl;

    std::vector<int> a1,a2,a3,a4;
    std::vector<float> b1,b2,b3,b4;

    for (int ii = 0; ii < D.cols; ii++)
    {
        a1.clear();
        a2.clear();
        a3.clear();
        a4.clear();
        b1.clear();
        b2.clear();
        b3.clear();
        b4.clear();
        for (int jj = ii + 1; jj < D.cols; jj++)
        {
            if (L.at<float>(ii, jj)==1)
            {
                a1.push_back(jj);
                b1.push_back(B.at<float>(ii, jj));
            }
            else if (L.at<float>(ii, jj) == 2)
            {
                a2.push_back(jj);
                b2.push_back(B.at<float>(ii, jj));
            }
            else if (L.at<float>(ii, jj) == 3)
            {
                a3.push_back(jj);
                b3.push_back(B.at<float>(ii, jj));
            }
            else if (L.at<float>(ii, jj) == -1)
            {
                a4.push_back(jj);
                b4.push_back(B.at<float>(ii, jj));
            }
        }
        
        if(a1.size()>0)
        {
            auto smallest = std::min_element(std::begin(b1), std::end(b1));
            int MinIndex = std::distance(b1.begin(),smallest);
            C.at<float>(index,ii)=1;
            C.at<float>(index, a1[MinIndex]) = 1;
            a1.erase(a1.begin()+MinIndex);
            index++;
            for(int kk=0;kk<a1.size();kk++){
                C.at<float>(index,a1[kk])=-1;
                C.at<float>(index,ii)=-1;
                index++;
            }
        }
         if (a2.size() > 0)
        {
            auto smallest = std::min_element(std::begin(b2), std::end(b2));
            int MinIndex = std::distance(b2.begin(), smallest);
            C.at<float>(index, ii) = 1;
            C.at<float>(index, a2[MinIndex]) = 1;
            a2.erase(a2.begin() + MinIndex);
            index++;
            for (int kk = 0; kk < a2.size(); kk++)
            {
                C.at<float>(index, a2[kk]) = -1;
                C.at<float>(index, ii) = -1;
                index++;
            }
        } 
        if (a3.size() > 0)
        {
            auto smallest = std::min_element(std::begin(b3), std::end(b3));
            int MinIndex = std::distance(b3.begin(), smallest);
            C.at<float>(index, ii) = 1;
            C.at<float>(index, a3[MinIndex]) = 1;
            a3.erase(a3.begin() + MinIndex);
            index++;
            for (int kk = 0; kk < a3.size(); kk++)
            {
                C.at<float>(index, a3[kk]) = -1;
                C.at<float>(index, ii) = -1;
                index++;
            }
        }

        for (int kk = 0; kk < a4.size(); kk++)
        {
            C.at<float>(index, a4[kk]) = -1;
            C.at<float>(index, ii) = -1;
            index++;
        }
        // std::cout <<"ii:"<<ii<< " a1.size:" << a1.size() << std::endl;
        // std::cout <<"ii:"<<ii<< " a2.size:" << a2.size() << std::endl;
        // std::cout <<"ii:"<<ii<< " a3.size:" << a3.size() << std::endl;
        // std::cout <<"ii:"<<ii<< " a4.size:" << a4.size() << std::endl;
        // std::cout <<"ii:"<<ii<< " index:" << index << std::endl;

    }

    //std::cout<<"C:"<<std::endl;
    //std::cout<<C<<std::endl;
}
std::vector<LD_COEFF> CJudgeCenter::DeleteRb(bool &finish_flag)
{
    float score; 
    int kk= 0;
    int value =0;
    //std::cout<<"delete index :";
    for(int ii = 0;ii<C.cols;ii++){
        score = 0.0;
        for(int jj=0;jj<C.rows;jj++)
        {
            score +=C.at<float>(jj,ii);
        }
        if (score < value){
            value = score ;
            kk = ii;
        }
    }
    if (value!=0){
        //std::cout << " " << kk << std::endl;
        _lane_coeff.erase(_lane_coeff.begin() + kk);
    }else{
        finish_flag = true ;
    }
    //std::cout << std::endl;
    return _lane_coeff;
}
void CJudgeCenter::Run(std::vector<LD_COEFF> &tmp)
{
    if (tmp.size() > 0 && !_finish_flag)
    {
        ReSort(tmp);
        CalculateDistance(tmp);
        Evaluate();
        tmp = DeleteRb(_finish_flag);
        Run(tmp);
    }else{
        _finish_flag =false ;
         return;
    }
}