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
#include "../classInfo/classinfo.hpp"
#include <ros/ros.h>
#include <vector>

using namespace std;

#define PI 3.1415926535897932384626433832795	//pi
#define PI_2 1.5707963267948966192313216916398	//pi/2
enum MoveState
{
    Straight,Right_Shift,Left_Shift,Right_Protect,Left_Protect,Right_turn,Left_Turn
};
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.v = to explain what is being done
 */
class Strategy
{

public:

    vector<double> osc_move_x_r;
    vector<double> osc_move_x_l;
    vector<double> osc_move_y_r;
    vector<double> osc_move_y_l;
    vector<double> osc_move_z_r;
    vector<double> osc_move_z_l;
    vector<double> osc_move_com_x;
    vector<double> osc_move_com_y;
    vector<double> osc_move_com_z;
    vector<double> right_Thta;
    vector<double> left_Thta;
    vector<double> test;

    Strategy();
    ~Strategy();
    void walkingprocess(int walking_mode);
    void walkfunction();
    void continuouswalk();
    void inversekinmaticsinfo();
    void SaveData();
    string DtoS(double value);

    double OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms);
    double OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms);
    double OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms);
    double OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
    double OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms);
    double OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms);
    double OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms);
};

#endif // QTUTORIALS_MAIN_WINDOW_H
