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

/*****************************************************************************
** Class
*****************************************************************************/
class Parameters
{

public:
    Parameters();
    ~Parameters();

    float R_Goal[3];
    float L_Goal[3];
    float R_theta[5];
    float L_theta[5];

};


class ClassInfo
{

public:
    ClassInfo();
    ~ClassInfo();

    bool rosflag;
    Parameters parameters;

};


extern ClassInfo* classinfotest;
#endif /* _CLASSINFO_H_ */
