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
    StartStep,FirstStep,Repeat,StopStep,MarkTimeStep,ForwardStep,BackwardStep
};
/*****************************************************************************
** Class
*****************************************************************************/
class Parameters
{

public:
    Parameters();
    ~Parameters();

    float X_Swing_Range;
    float Y_Swing_Range;
    float Z_Swing_Range;
    int Period_T;
    int Period_T2;
    int Period_COM_Y;
    int Sample_Time;
    float OSC_LockRange;
    float BASE_Default_Z;
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
    bool walking_stop;
};

class ClassInfo
{

public:
    ClassInfo();
    ~ClassInfo();

    Parameters parameters;
    COMPlan complan;

    bool rosflag;
    bool walkingpointflag;
    double X;
    double Y;
    double Z;
    double THTA;
    int walking_mode;
    //int walking_state;

    double XUpdate;
    double YUpdate;
    double ZUpdate;
    double THTAUpdate;

    bool WalkFlag;
    bool FpgaFlag;
    bool IsParametersLoad;
    int counter;
};


extern ClassInfo* classinfotest;
#endif /* _CLASSINFO_H_ */
