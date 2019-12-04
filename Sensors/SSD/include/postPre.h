#ifndef __POSTPRE_H__
#define __POSTPRE_H__
#include "profile.h"
#include "OpenICV/structure/structureBBox2D.h"

using namespace std;
using namespace cv;


/**
 * @brief Post process after the runing of DPU for YOLO-v3 network
 *
 * @param task - pointer to DPU task for running YOLO-v3
 * @param frame
 * @param sWidth
 * @param sHeight
 *
 * @return none
 */


void SSDPostTreat(DPUTask *task, Mat &frame, int8_t *conf_data, float conf_scale, int8_t *loc_data, float loc_scale, float *prior_data, BoundingBoxes2d &boxes) 
{
    // OrderArray store sorted bbox by score for every class, OrderPoint store begin and end
    struct IndexScoreBBox OrderArray[top_k];
    int OrderPoint = 0;
    int begin_point, end_point, middle_point, insert_point;
    int l, i, k;
    float fix_scale;
    int8_t *conf_point;
    float conf_total;
    float softmax_conf[num_class];
    float max_conf;
    int max_index;

    // get sorted index and score for num_class == 2
    conf_point = conf_data;
    fix_scale = conf_scale;
    for (i = 0; i < anchor_num; i++) 
    {
        // 在仅有2分类的情况下，softmax计算可以大大化简
        // 只计算包含背景和单个类别，其他情况暂时不考虑
        //max_conf = ((float)(conf_point[1]) - (float)(conf_point[0])) / fix_scale;
        //max_index = 1;
    
        //num_class = any的代码
        conf_total = exp((float)(conf_point[0]) / fix_scale);
        max_conf = 0.f;
        max_index = 0;
        for (k = 1; k < num_class; k++) 
        {
            softmax_conf[k] = exp((float)(conf_point[k]) / fix_scale);
            conf_total += softmax_conf[k];
            if (softmax_conf[k] > max_conf) 
            {
                max_conf = softmax_conf[k];
                max_index = k;
            }
        }
        max_conf = max_conf / conf_total;
    
        // filter low than confidence_threshold
        conf_point = conf_point + num_class;
        if (max_conf > confidence_threshold && max_index > 0) 
        {
            if (OrderPoint > 0) 
            {
                end_point = OrderPoint - 1;
                if (max_conf <= OrderArray[end_point].score) 
                    insert_point = end_point + 1;
                else if (max_conf <= OrderArray[0].score) 
                {
                    begin_point = 0;
                    // search point
                    while(end_point - begin_point > 1)
                    {
                        middle_point = (begin_point + end_point) >> 1;
                        if (max_conf > OrderArray[middle_point].score) 
                            end_point = middle_point;
                        else 
                            begin_point = middle_point;
                    }
                    insert_point = end_point;
                }
                else 
                    insert_point = 0;
                // insert point
                if (insert_point < top_k) 
                {
                    for (k = (OrderPoint < top_k ? OrderPoint : (top_k - 1)); k > insert_point; k--) 
                    {
                        OrderArray[k].index = OrderArray[k - 1].index;
                        OrderArray[k].score = OrderArray[k - 1].score;
                        OrderArray[k].category = OrderArray[k - 1].category;
                    }
                    OrderArray[insert_point].index = i;
                    OrderArray[insert_point].score = max_conf;
                    OrderArray[insert_point].category = max_index;
                    OrderPoint++;
                    OrderPoint = (OrderPoint < top_k ? OrderPoint : top_k);
                }
            }
            else 
            {
                OrderArray[0].index = i;
                OrderArray[0].score = max_conf;
                OrderArray[0].category = max_index;
                OrderPoint++;
            }
        }
    }
  
    int index;
    int8_t *loc_point;
    float *prior_point;
    float *prior_vari_point;
    float prior_width, prior_height;
    float decode_center_x, decode_center_y, decode_width, decode_height;
    for (k = 0; k < OrderPoint; k++) 
    {
        // get data point
        index = OrderArray[k].index;
        loc_point = loc_data + index * 4;
        prior_point = prior_data + index * 4;
        prior_vari_point = prior_point + anchor_num * 4;
        fix_scale = loc_scale;
        // get prior box width and height
        prior_width = prior_point[2] - prior_point[0];
        prior_height = prior_point[3] - prior_point[1];
        // Decode with fix_point to anchor_num
        decode_center_x = prior_vari_point[0] * (float)(loc_point[0]) * prior_width / fix_scale + (prior_point[0] + prior_point[2]) / 2.f;
        decode_center_y = prior_vari_point[1] * (float)(loc_point[1]) * prior_height / fix_scale + (prior_point[1] + prior_point[3]) / 2.f;
        decode_width = exp(prior_vari_point[2] * (float)(loc_point[2]) / fix_scale) * prior_width;
        decode_height = exp(prior_vari_point[3] * (float)(loc_point[3]) / fix_scale) * prior_height;
        // give
        OrderArray[k].xmin = decode_center_x - decode_width / 2.f;
        OrderArray[k].ymin = decode_center_y - decode_height / 2.f;
        OrderArray[k].xmax = decode_center_x + decode_width / 2.f;
        OrderArray[k].ymax = decode_center_y + decode_height / 2.f;
    }
    int keep;
    float inter_xmin, inter_ymin, inter_xmax, inter_ymax;
    float inter_area, area1, area2;
    float iou;
    // set all flag to true
    for (k = 0; k < OrderPoint; k++) 
    {
        OrderArray[k].flag = true;
    }
    keep = 0;
    while (keep < OrderPoint) 
    {
        for (k = keep + 1; k < OrderPoint; k++) 
        {
            // remove iou large than nms_threshold
            if (OrderArray[k].flag) 
            {
                // no intersection
                if (OrderArray[k].xmin > OrderArray[keep].xmax || OrderArray[k].xmax < OrderArray[keep].xmin ||
                    OrderArray[k].ymin > OrderArray[keep].ymax || OrderArray[k].ymax < OrderArray[keep].ymin) 
                {
                    continue;
                }   
                else 
                {
                    // cal iou
                    inter_xmin = OrderArray[k].xmin > OrderArray[keep].xmin ? OrderArray[k].xmin : OrderArray[keep].xmin;
                    inter_ymin = OrderArray[k].ymin > OrderArray[keep].ymin ? OrderArray[k].ymin : OrderArray[keep].ymin;
                    inter_xmax = OrderArray[k].xmax < OrderArray[keep].xmax ? OrderArray[k].xmax : OrderArray[keep].xmax;
                    inter_ymax = OrderArray[k].ymax < OrderArray[keep].ymax ? OrderArray[k].ymax : OrderArray[keep].ymax;
                    inter_area = (inter_xmax - inter_xmin) * (inter_ymax - inter_ymin);
                    area1 = (OrderArray[k].xmax - OrderArray[k].xmin) * (OrderArray[k].ymax - OrderArray[k].ymin);
                    area2 = (OrderArray[keep].xmax - OrderArray[keep].xmin) * (OrderArray[keep].ymax - OrderArray[keep].ymin);
                    iou = inter_area / (area1 + area2 - inter_area);
                    if (iou > nms_threshold) 
                        OrderArray[k].flag = false;
                }
            }
        }
        // next keep
        for (k = keep + 1; k < OrderPoint; k++) 
        {
            if (OrderArray[k].flag) 
                break;
        }
        keep = k;
    }
    
    int xmin, ymin, width, height;  
    int ind_img=0;
    BoundingBox2d box ;
    //char image_name[100];
    for (k = 0; k < OrderPoint; k++) 
    {
        if (OrderArray[k].flag) 
        {
            int imgWidth = frame.cols;
		    int imgHeight = frame.rows;
            //cout<<"xmin----"<<OrderArray[k].xmin<<endl;
            //cout<<"ymin----"<<OrderArray[k].xmin<<endl;
            // xmin = (int)(round(OrderArray[k].xmin * im_or_width * 100.) / 100.);
            // ymin = (int)(round(OrderArray[k].ymin * im_or_height * 100.) / 100.);
	        xmin = (int)(round(OrderArray[k].xmin * imgWidth * 100.) / 100.);
            ymin = (int)(round(OrderArray[k].ymin * imgHeight * 100.) / 100.);
            // width = (int)(round((OrderArray[k].xmax - OrderArray[k].xmin) * im_or_width * 100.) / 100.);
            // height = (int)(round((OrderArray[k].ymax - OrderArray[k].ymin) * im_or_height * 100.) / 100.);
            width = (int)(round((OrderArray[k].xmax - OrderArray[k].xmin) * imgWidth * 100.) / 100.);
            height = (int)(round((OrderArray[k].ymax - OrderArray[k].ymin) * imgHeight * 100.) / 100.);
            box.xmin = xmin;
            box.ymin = ymin;
            box.xmax = xmin + width;
            box.ymax = ymin + height;
            box.probability = OrderArray[k].score;
            //printf(" %d %f %d %d %d %d %d\n", OrderArray[k].index,\
                                              OrderArray[k].score,\
					                          OrderArray[k].category,\
                                              xmin, ymin, xmin + width, ymin + height);
        }
        int label = OrderArray[k].category;
        Point origin;
        origin.x = xmin;
        origin.y = ymin;
        string classname = classes[label];
        box.label = label ;
        if (label==2)
		{
			rectangle(frame, Point(xmin, ymin), Point(xmin + width, ymin + height), Scalar(255, 0, 0), 1, 1, 0);
			putText(frame, classname, origin, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);
		}
		else if (label==7)
		{
			rectangle(frame, Point(xmin, ymin), Point(xmin + width, ymin + height), Scalar(0, 255, 0), 1, 1, 0);
			putText(frame, classname, origin, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 1, 4);
		}

		else if (label==15)
		{
			rectangle(frame, Point(xmin, ymin), Point(xmin + width, ymin + height), Scalar(0, 0, 255), 1, 1, 0);
			putText(frame, classname, origin, FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1, 4);
		}
		else
		{
		}

        boxes.bounding_boxes.push_back(box);
    }
    // printf("total %d\n", bbox_count);

}

#endif
