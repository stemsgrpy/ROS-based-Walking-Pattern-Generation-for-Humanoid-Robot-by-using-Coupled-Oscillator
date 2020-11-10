/**
 * @file  src/qt_ros_pkg/ClassInfo/classinfo.hpp
 *
 * @brief Bridge between Ros with Task
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef CLASSINFO_HPP_
#define CLASSINFO_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

enum WalkingState
{
    StartStep,FirstStep,Repeat,StopStep
};
/*****************************************************************************
** Class
*****************************************************************************/
class Parameters
{

public:
    Parameters();
    ~Parameters();

    double X_Swing_Range;
    double Y_Swing_Range;
    double Z_Swing_Range;
    int Period_T;
    int Period_T2;
    int Period_COM_Y;
    int Sample_Time;
    double OSC_LockRange;
    double BASE_Default_Z;
    double COM_Height;
};

class Points
{

public:
    Points();
    ~Points();

    double X_Right_foot;
    double X_Left_foot;
    double Y_Right_foot;
    double Y_Left_foot;
    double Z_Right_foot;
    double Z_Left_foot;
    double X_COM;
    double Y_COM;
    double Z_COM;
    double Right_Thta;
    double Left_Thta;

    double IK_Point_RX;
    double IK_Point_RY;
    double IK_Point_RZ;
    double IK_Point_RThta;
    double IK_Point_LX;
    double IK_Point_LY;
    double IK_Point_LZ;
    double IK_Point_LThta;
};

class COMPlan
{

public:
    COMPlan();
    ~COMPlan();

    int sample_point_;
    int time_point_;
    bool isfirststep;
    bool islaststep;
    int walking_state;
};

class ClassInfo
{

public:
    ClassInfo();
    ~ClassInfo();

    Parameters parameters;
    Points points;
    COMPlan complan;

    bool rosflag;
    bool doflag;
    double X;
    double Y;
    double Z;
    double THTA;
    int walking_mode;
    int time_point_;

    double XUpdate;
    double YUpdate;
    double ZUpdate;
    double THTAUpdate;

    bool WalkFlag;
    bool IsParametersLoad;
    int counter;

};


extern ClassInfo* classinfotest;
#endif /* _CLASSINFO_H_ */
