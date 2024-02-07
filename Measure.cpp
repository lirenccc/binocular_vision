/*** 
 * @Copyright: © 2024, Liren Cao. All rights reserved.
 * @Author: Liren Cao
 * @Description: 双目相机的识别目标坐标与角度解算
 */

#include "components/Measure.hpp"

Measure::Measure(std::string filename)
{
    std::vector<std::vector<float>> intPara;
    std::ifstream inFile(filename, std::ios::in);
    if(!inFile)
    {
        std::cerr << "Failed to open file!" << std::endl;
        exit(1);
    }
    std::string line;
    while(getline(inFile, line))
    {
        std::vector<float> tmp;
        std::string field;
        std::istringstream sin(line);

        int i = 0;
        while(getline(sin, field, ','))
        {
            if(i>0)
                tmp.push_back(std::stof(field));
            i++;
        }
        intPara.push_back(tmp);
    }
    inFile.close();

    for(int i = 0; i < 2; i++) {
        fl_[i] = intPara[0][i] * bino_pixel_size_;
        fr_[i] = intPara[5][i] * bino_pixel_size_;
        cl_[i] = intPara[1][i];
        cr_[i] = intPara[6][i];
    }
    float om[3] = {0};
    for(int i = 0; i < 3; i++) {
        om[i] = intPara[10][i];
    }
    om_ = (cv::Mat_<float>(1, 3) << om[0], om[1], om[2]);
    for(int i = 0; i < 3; i++) {
        T_[i] = intPara[11][i];
    }
    baseline_ = -T_[0];
    cv::Rodrigues(om_, rev_);
}

void Measure::init()
{
    printf("fl=%f,%f, fr=%f,%f, b=%f,cl=%f,%f,cr=%f,%f\n", fl_[0], fl_[1], fr_[0], fr_[1], baseline_, cl_[0], cl_[1], cr_[0], cr_[1]);
}

cv::Point3f Measure::getCoordinate2P(cv::Point2i leftPoint, cv::Point2i rightPoint)
{
    if(leftPoint.x < 0 || leftPoint.y < 0 || rightPoint.x < 0 || rightPoint.y < 0) return cv::Point3f(-1, -1, -1);
/*  以下为标准平行光轴情况下的计算方法
    float laX = (leftPoint.x - cl_[0]) * bino_pixel_size_;
    float lbX = (rightPoint.x - cr_[0]) * bino_pixel_size_;
    float parallax = laX * fr_[0] - lbX * fl_[0];                                  // 视差
    float z_in_leftcamsys = (fl_[0] * fr_[0] * baseline_) / parallax;
    float x_in_leftcamsys = (laX * fr_[0] * baseline_) / parallax;
    float lY = (cl_[0] - leftPoint.y) * bino_pixel_size_;
    float y_in_leftcamsys = (lY * z_in_leftcamsys) / fl_[1];  */
    // 以下为一般双目模型的计算方法
    float xl = (leftPoint.x - cl_[0]) * bino_pixel_size_;
    float yl = (leftPoint.y - cl_[1]) * bino_pixel_size_;
    float xr = (rightPoint.x - cr_[0]) * bino_pixel_size_;
    float yr = (rightPoint.y - cr_[1]) * bino_pixel_size_;
    float z_in_leftcamsys = (fl_[0] * (fr_[0] * T_[0] - xr * T_[2])) / \
                            (xr * (rev_.at<float>(2,0) * xl + rev_.at<float>(2,1) * yl + fl_[0] * rev_.at<float>(2,2)) - \
                             fr_[0] * (rev_.at<float>(0,0) * xl + rev_.at<float>(0,1) * yl + fl_[0] * rev_.at<float>(0,2)));
    // 由于Y轴平移量非常小，因此不考虑通过Y轴进行计算
    // float z_in_leftcamsys_by_y = (fl_[1] * (fr_[1] * T_[1] - yr * T_[2])) / (yr * (rev_.at<float>(2,0) * xl + rev_.at<float>(2,1) * yl + fl_[1] * rev_.at<float>(2,2)) - fr_[1] * (rev_.at<float>(1,0) * xl + rev_.at<float>(1,1) * yl + fl_[1] * rev_.at<float>(1,2)));
    // float z_in_leftcamsys = sqrt((pow(z_in_leftcamsys, 2) + pow(z_in_leftcamsys_by_y, 2)) / 2);
    float x_in_leftcamsys = z_in_leftcamsys * xl / fl_[0];
    float y_in_leftcamsys = -z_in_leftcamsys * yl / fl_[1];
    // 返回目标在机器人RPY坐标系下的坐标
    return cv::Point3f(x_in_leftcamsys + left_cam_in_robot_sys[0], \
                       y_in_leftcamsys + left_cam_in_robot_sys[1], \
                       z_in_leftcamsys + left_cam_in_robot_sys[2]);
}

void Measure::getpyd(cv::Point3f p)
{
    float xyz[3] = {p.x, p.y, p.z};
    float pyd[3];
    this->xyz2pyd(xyz, pyd);
    this->pyd_ [0]= pyd[0];
    this->pyd_ [1]= pyd[1];
    this->pyd_ [2]= pyd[2];
}

int Measure::shoot_get(float* angles)
{
    if(angles[0]>(-CV_PI/12) && angles[0]<(CV_PI/12) && angles[1]>(-CV_PI/12) && angles[1]<(CV_PI/12))
        return 1;
    else
        return 0;
}
