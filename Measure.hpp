/*** 
 * @Copyright: © 2024, Liren Cao. All rights reserved.
 * @Author: Liren Cao
 * @Description: 双目相机的识别目标坐标与角度解算
 */

#ifndef _MEASURE_H_
#define _MEASURE_H_

#include "headfiles.hpp"
#include <fstream>

class Measure
{
public:
    // Measure(float fc_left_x, float fc_left_y, float cc_left_x, float cc_left_y, float fc_right_x, float fc_right_y, float cc_right_x, float cc_right_y, float b);
    Measure(std::string filename);
    void init();
    cv::Point3f getCoordinate2P(cv::Point2i leftPoint, cv::Point2i rightPoint);
    void getpyd(cv::Point3f p);
    int shoot_get(float* angles);
    float getyaw() {return this->pyd_[0];}
    float getpitch() {return this->pyd_[1];}
    float getdistance() {return this->pyd_[2];}

    template<class T>
    void xyz2pyd(T xyz[3], T pyd[3]) {
        /*
        * 工具函数：将 xyz 转化为 pitch、yaw、distance
        */
        pyd[0] = atan2(xyz[1], xyz[2]) / 2 / CV_PI * 360;  // pitch
        pyd[1] = atan2(xyz[0], xyz[2]) / 2 / CV_PI * 360;  // yaw
        pyd[2] = sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
    }
    
private:
    float bino_pixel_size_ = 3.75/1000;                            // 3.75um=3.75/1000 mm 莱娜双目相机像素物理尺寸
    cv::Size bino_image_size_ = cv::Size(IMGWID, IMGHEI);   // 双目相机图像尺寸（单个相机的）
    float fl_[2] = {0};                        // 双目左相机焦距
    float cl_[2] = {0};                       // 双目左相机光心
    float fr_[2]  = {0};                      // 双目右相机焦距
    float cr_[2]  = {0};                     // 双目右相机光心
    cv::Mat om_;                             // 旋转向量
    cv::Mat rev_;                             // 旋转矩阵
    float T_[3] = {0};                      // 平移向量
    float left_cam_in_robot_sys[3] = {-115, 4, 59};            // 左相机坐标系原点在RPY坐标系中坐标,由机械确定

    float baseline_;

    float pyd_[3];
};

#endif