/**
 * @file /src/qt_ros_pkg/main_window.hpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <ros/ros.h>
#include "../classInfo/classinfo.hpp"
#include <vector>

using namespace std;
#define PI 3.14159265358979323846
//-----Link-----//
#define d0 -3.35
#define a0 4.0
#define a2 12.5
#define a3 12.5
#define a5 3.5
////-------Right-------//
#define R_nz -1.0
#define R_oy 1.0
#define R_ax 1.0
//-------Left-------//
#define L_nz -1.0
#define L_oy 1.0
#define L_ax 1.0
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.v = to explain what is being done
 */
class Strategy
{

public:

    vector<float> theta_r0;
    vector<float> theta_r1;
    vector<float> theta_r2;
    vector<float> theta_r3;
    vector<float> theta_r4;

    vector<float> theta_l0;
    vector<float> theta_l1;
    vector<float> theta_l2;
    vector<float> theta_l3;
    vector<float> theta_l4;

    Strategy();
    ~Strategy();
    void CalculatePAndfi(float &P, float &fi);
    void R_IK(float *theta, float *Goal);
    void L_IK(float *theta, float *Goal);
    void DoIK();
    void SaveData();
    string FtoS(float value);

    //private:

};

#endif // QTUTORIALS_MAIN_WINDOW_H
