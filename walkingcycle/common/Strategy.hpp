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

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include "../classInfo/classinfo.hpp"


#define INCREASE_SLOPE 1
#define WALK_MAX_DISTANCE 2
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.v = to explain what is being done
 */
class Strategy
{

public:

    Strategy();
    ~Strategy();
    void walkingkindfunction(int walking_mode);
    void singlestepfunction();
    void singlestepwalkingprocess();
    void continuoustepfunction();
    void continuouswalkingprocess();

    double forwardValue_;
    double forwardCounter_;
    int slopeCounter_;
    int Sample_points_quater;

    double COM_Y_tmp;
    double Lockrange_tmp;


    //private:

};

#endif // QTUTORIALS_MAIN_WINDOW_H
