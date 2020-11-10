/**
 * @file  src/qt_ros_pkg/ClassInfo/classinfo.cpp
 *
 * @brief Bridge between Ros with Task
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "classinfo.hpp"

ClassInfo *classinfotest = new ClassInfo();
/*****************************************************************************
** Implementation
*****************************************************************************/
Parameters::Parameters()
{
    this->X_Swing_Range = 0;
    this->Y_Swing_Range = 0;
    this->Z_Swing_Range = 0;
    this->Period_T = 0;
    this->Period_T2 = 0;
    this->Period_COM_Y = 0;
    this->Sample_Time = 0;
    this->OSC_LockRange = 0;
    this->BASE_Default_Z = 0;

    this->COM_Height = 21.7;
}

Parameters::~Parameters()
{

}

Points::Points()
{

}

Points::~Points()
{

}

COMPlan::COMPlan()
{
    this->sample_point_ = 0;
    this->time_point_ = 0;
    this->walking_state = StopStep;
}

COMPlan::~COMPlan()
{

}

ClassInfo::ClassInfo()
{
    this->rosflag = false;
    this->doflag = false;
    this->X = 0;
    this->Y = 0;
    this->Z = 0;
    this->THTA = 0;
    this->walking_mode = 0;

    this->XUpdate = 0;
    this->YUpdate = 0;
    this->ZUpdate = 0;
    this->THTAUpdate = 0;

    this->WalkFlag = false;
    this->IsParametersLoad = false;
}

ClassInfo::~ClassInfo()
{

}

