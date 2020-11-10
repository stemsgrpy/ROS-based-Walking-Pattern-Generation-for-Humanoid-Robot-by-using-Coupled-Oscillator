/**
 * @file /src/qt_ros_pkg/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date 2015/10/16
 *
 * @author shengru
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "Strategy.hpp"

using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
Strategy::Strategy()
{
    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Thta.clear();
    left_Thta.clear();
    test.clear();

}

Strategy::~Strategy()
{

}

void Strategy::walkingprocess(int walking_mode)
{
    switch(walking_mode)
    {
    case 0://Single Step
        walkfunction();
        break;
    case 1://Continuous
        continuouswalk();
        break;
    case 2:
    default:
        break;
    }
    inversekinmaticsinfo();
}

void Strategy::walkfunction()
{
    int time_shift = 0;
    int time_point_ = classinfotest->time_point_;
    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  classinfotest->time_point_ + 0.5 * classinfotest->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    /************************************************************************/
    /* Set flags of moving cases                                            */
    /************************************************************************/
    int moving_mode;
    if (classinfotest->Y > 0)
    {
        if (classinfotest->THTA == 0)
        {
            moving_mode = Left_Shift;
        }
        else
        {
            moving_mode = Left_Protect;
        }
    }
    else if (classinfotest->Y < 0)
    {
        if (classinfotest->THTA == 0)
        {
            moving_mode = Right_Shift;
        }
        else
        {
            moving_mode = Right_Protect;
        }
    }
    else
    {
        if (classinfotest->THTA > 0)
        {
            moving_mode = Left_Turn;
        }
        else if (classinfotest->THTA < 0)
        {
            moving_mode = Right_turn;
        }
        else
        {
            moving_mode = Straight;
        }
    }
    ///************************************************************************/
    ///* Switch moving cases from flags                                       */
    ///************************************************************************/
    //    classinfotest->complan.isfirststep = msg.isfirststep;
    //    classinfotest->complan.islaststep = msg.islaststep;
    switch (moving_mode)
    {
    case Right_turn:

        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_ )//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix



        classinfotest->points.Y_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        classinfotest->points.Y_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.X_COM = OSC_COM_X(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->X, 0, time_point_);

        classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_point_ + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (classinfotest->complan.isfirststep || classinfotest->complan.islaststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        else
        {
            classinfotest->points.Right_Thta =  OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA * -1, 0, time_shift); //Swing leg turn right
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA, 0, time_shift);
        }
        break;
    case Right_Protect:
        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_Right_foot = 0;
            classinfotest->points.Y_Left_foot = 0;
        }
        else
        {
            classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower
            classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower

        }
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.X_COM = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->parameters.X_Swing_Range, 0, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_point_ + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range-classinfotest->Y*0.25, 0, PI_2, time_point_ + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        else if (classinfotest->THTA > 0) // Turn Left
        {
            classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA * -1, 0, time_shift);
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA, 0, time_shift);	// Swing leg turn left;
        }
        else if (classinfotest->THTA < 0) // Turn Right
        {
            classinfotest->points.Right_Thta =  OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA * -1, 0, time_shift); //Swing leg turn right
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA, 0, time_shift);
        }
        else
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        break;

    case Left_Turn:

        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        classinfotest->points.Y_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        classinfotest->points.Y_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.X_COM = OSC_COM_X(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->X, 0, time_point_);
        classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_shift + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;	// Swing leg turn left
        }
        else
        {
            classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA, 0, time_shift);
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA * -1, 0, time_shift);	// Swing leg turn left;
        }
        break;
    case Left_Protect:
        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X , 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_Right_foot = 0;
            classinfotest->points.Y_Left_foot = 0;
        }
        else
        {

            classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower
            classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower
        }

        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.X_COM = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->parameters.X_Swing_Range, 0, PI_2, time_shift + classinfotest->parameters.Period_T2/4);
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_shift + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range+classinfotest->Y*0.25, 0, PI_2, time_shift + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }

        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange
        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        else if (classinfotest->THTA > 0) // Turn Left
        {
            classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA, 0, time_shift);
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA * -1, 0, time_shift);	// Swing leg turn left;
        }
        else if (classinfotest->THTA < 0) // Turn Right
        {

            classinfotest->points.Right_Thta =  OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.3 * classinfotest->THTA, 0, time_shift); //Swing leg turn right
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.7 * classinfotest->THTA * -1, 0, time_shift);
        }
        else
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        break;

    case Right_Shift:
        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T,classinfotest->X, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_LX_fix

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_Right_foot = 0;
            classinfotest->points.Y_Left_foot = 0;
        }
        else
        {
            classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower
            classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower
        }
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.X_COM = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->parameters.X_Swing_Range, 0, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_point_ + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range-classinfotest->Y*0.25, 0, PI_2, time_point_ + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        else
        {
            classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, -0/180*PI, 0, time_point_); //Swing leg turn right //Parameters->Shift_Lfoot_RTurn
            classinfotest->points.Left_Thta = 0;
        }

        break;
    case Left_Shift:

        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X , 0, PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_Right_foot = 0;
            classinfotest->points.Y_Left_foot = 0;
        }
        else
        {

            classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower
            classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower

            ROS_INFO("I heard: [%f , %f , %d]",classinfotest->parameters.OSC_LockRange,classinfotest->Y,classinfotest->counter);
        }
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.X_COM = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->parameters.X_Swing_Range, 0, PI_2, time_shift + classinfotest->parameters.Period_T2/4);

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, PI_2, time_shift + classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        }
        else
        {
            classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range+classinfotest->Y*0.25, 0, PI_2, time_shift + classinfotest->parameters.Period_T/4);//OSC_COM_LockRange
        }

        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange

        if (classinfotest->complan.isfirststep)
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = 0;
        }
        else
        {
            classinfotest->points.Right_Thta = 0;
            classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0/180*PI, PI, time_shift);//Parameters->Shift_Rfoot_LTurn
        }

        break;
    case Straight:

        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange , classinfotest->parameters.Period_T, classinfotest->X, 0 , PI_2, time_shift)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0 , PI_2, time_point_)//Parameters->BASE_delrho_x
                + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0 , PI, time_point_);//Parameters->Shift_LX_fix

        classinfotest->points.Y_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Open_value_R
        classinfotest->points.Y_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Open_value_L
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        if (classinfotest->complan.islaststep)
        {
            classinfotest->points.X_COM = 0;
        }
        else
        {
            classinfotest->points.X_COM = OSC_COM_X(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->X, 0, time_point_);
        }
        classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range,0, PI_2, time_shift+classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange

        classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_move_x_advance(0, classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0.7, PI_2, time_point_ + classinfotest->parameters.Period_T2/4);//Parameters->OSC_COM_LockRange,
        if (classinfotest->complan.islaststep)
        {
            classinfotest->points.Right_Thta = 0;//821
            classinfotest->points.Left_Thta = 0;//821
        }
        else
        {
            classinfotest->points.Right_Thta = 0;//+= OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, 0, time_shift);//Parameters->Str_Rfoot_Lturn
            classinfotest->points.Left_Thta  = 0;//+= OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, 0, time_point_);//-Parameters->Str_Lfoot_Rturn
        }
        break;
    default:
        classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_shift);//Parameters->BASE_delrho_x
        classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_);//Parameters->BASE_delrho_x
        classinfotest->points.Y_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Open_value_R
        classinfotest->points.Y_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Open_value_L
        classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
        classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);
        classinfotest->points.X_COM = OSC_COM_X(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->X , 0, time_point_);
        classinfotest->points.Y_COM = OSC_move_x_advance(0, classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range,0, PI_2, time_point_+classinfotest->parameters.Period_T/4);//Parameters->OSC_COM_LockRange
        classinfotest->points.Z_COM = 0;//classinfotest->parameters.COM_Height + OSC_move_x_advance(classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0, time_point_);
        classinfotest->points.Right_Thta = 0;
        classinfotest->points.Left_Thta = 0;
    }

    //    ROS_INFO("I heard: [%f %f %f %f %f %f %f %f %f %f %f]", classinfotest->points.X_Right_foot,classinfotest->points.X_Left_foot,
    //                                                            classinfotest->points.Y_Right_foot,classinfotest->points.Y_Left_foot,
    //                                                            classinfotest->points.Z_Right_foot,classinfotest->points.Z_Left_foot,
    //                                                            classinfotest->points.X_COM,classinfotest->points.Y_COM,classinfotest->points.Z_COM,
    //                                                            classinfotest->points.Right_Thta,classinfotest->points.Left_Thta);
    classinfotest->rosflag = true;

    osc_move_x_r.push_back(classinfotest->points.X_Right_foot);
    osc_move_x_l.push_back(classinfotest->points.X_Left_foot);
    osc_move_y_r.push_back(classinfotest->points.Y_Right_foot);
    osc_move_y_l.push_back(classinfotest->points.Y_Left_foot);
    osc_move_z_r.push_back(classinfotest->points.Z_Right_foot);
    osc_move_z_l.push_back(classinfotest->points.Z_Left_foot);
    osc_move_com_x.push_back(classinfotest->points.X_COM);
    osc_move_com_y.push_back(classinfotest->points.Y_COM);
    osc_move_com_z.push_back(classinfotest->points.Z_COM);
    right_Thta.push_back(classinfotest->points.Right_Thta);
    left_Thta.push_back(classinfotest->points.Left_Thta);
    test.push_back(classinfotest->counter);

    if(classinfotest->WalkFlag)
        SaveData();
}

void Strategy::continuouswalk()
{
    int time_shift = 0;
    int time_point_ = classinfotest->time_point_;
    //    double open_value = sin(Parameters->Open_Phase);
    time_shift =  classinfotest->time_point_ + 0.5 * classinfotest->parameters.Period_T;
    //    double Open_value_R = Parameters->R_Open;
    //    double Open_value_L = Parameters->L_Open;

    classinfotest->points.X_Right_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_shift)//Parameters->BASE_delrho_x
            + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_shift);//Parameters->Shift_RX_fix
    classinfotest->points.X_Left_foot = OSC_move_x_advance(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->X, 0, PI_2, time_point_)//Parameters->BASE_delrho_x
            + OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Parameters->Shift_LX_fix
    classinfotest->points.Z_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_shift);
    classinfotest->points.Z_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, classinfotest->Z, PI, time_point_);

    classinfotest->points.X_COM = OSC_COM_X(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T2, classinfotest->X, 0, time_point_);
    classinfotest->points.Y_COM = OSC_COM_Y(classinfotest->parameters.Period_T, classinfotest->parameters.Y_Swing_Range, 0, time_point_);
    classinfotest->points.Z_COM = classinfotest->parameters.COM_Height + OSC_COM_Z(classinfotest->parameters.Period_T2, classinfotest->parameters.Z_Swing_Range, 0, time_point_);
    // Turn
    if (classinfotest->THTA > 0) // Turn Left
    {

        classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.5 * classinfotest->THTA, 0, time_shift);
        classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.5 * classinfotest->THTA * -1, 0, time_shift);	// Swing leg turn left

    }
    else if (classinfotest->THTA < 0) // Turn Right
    {
        classinfotest->points.Right_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.5 * classinfotest->THTA * -1, 0, time_point_);//Swing leg turn right
        classinfotest->points.Left_Thta = OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0.5 * classinfotest->THTA, 0, time_point_);
    }
    else
    {
        classinfotest->points.Right_Thta = 0;
        classinfotest->points.Left_Thta = 0;
    }
    // Shift
    if (classinfotest->Y > 0)
    {
        classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower
        classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower
        classinfotest->points.Right_Thta += OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0*PI/180, 0, time_shift);	// Turn fix for shift  //Parameters->Shift_Rfoot_LTurn
        classinfotest->points.Left_Thta = 0;
    }
    else if (classinfotest->Y < 0)
    {
        classinfotest->points.Y_Right_foot = OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (0.5 - 0) * classinfotest->Y, 0, time_shift);//Parameters->Shift_delPower
        classinfotest->points.Y_Left_foot =  OSC_move_shift_y(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, (2 + 0) * classinfotest->Y * -1, 0, time_shift);//Parameters->Shift_delPower
        classinfotest->points.Right_Thta = 0;
        classinfotest->points.Left_Thta += OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0*PI/180, 0, time_point_); // Turn fix for shift  //Parameters->Shift_Lfoot_RTurn
    }
    else
    {
        classinfotest->points.Y_Right_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T,0, PI, time_shift);// Open_value_R
        classinfotest->points.Y_Left_foot = OSC_move_z(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, PI, time_point_);//Open_value_L
    }
    // Add turn offset
    classinfotest->points.Y_Right_foot += 0;//Y_Right_foot_tmp_;
    classinfotest->points.Y_Left_foot += 0;//Y_Left_foot_tmp_;
    // Forward modify
    if (classinfotest->X != 0)
    {
        classinfotest->points.Right_Thta += OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, 0, time_point_); //817(Lturn) //Parameters->Str_Rfoot_Lturn
        classinfotest->points.Left_Thta += OSC_Rotate(classinfotest->parameters.OSC_LockRange, classinfotest->parameters.Period_T, 0, 0, time_shift); //-Parameters->Str_Lfoot_Rturn
    }

    //    ROS_INFO("I heard: [%f %f %f %f %f %f %f %f %f %f %f]", classinfotest->points.X_Right_foot,classinfotest->points.X_Left_foot,
    //                                                            classinfotest->points.Y_Right_foot,classinfotest->points.Y_Left_foot,
    //                                                            classinfotest->points.Z_Right_foot,classinfotest->points.Z_Left_foot,
    //                                                            classinfotest->points.X_COM,classinfotest->points.Y_COM,classinfotest->points.Z_COM,
    //                                                            classinfotest->points.Right_Thta,classinfotest->points.Left_Thta);
    classinfotest->rosflag = true;

    osc_move_x_r.push_back(classinfotest->points.X_Right_foot);
    osc_move_x_l.push_back(classinfotest->points.X_Left_foot);
    osc_move_y_r.push_back(classinfotest->points.Y_Right_foot);
    osc_move_y_l.push_back(classinfotest->points.Y_Left_foot);
    osc_move_z_r.push_back(classinfotest->points.Z_Right_foot);
    osc_move_z_l.push_back(classinfotest->points.Z_Left_foot);
    osc_move_com_x.push_back(classinfotest->points.X_COM);
    osc_move_com_y.push_back(classinfotest->points.Y_COM);
    osc_move_com_z.push_back(classinfotest->points.Z_COM);
    right_Thta.push_back(classinfotest->points.Right_Thta);
    left_Thta.push_back(classinfotest->points.Left_Thta);
    test.push_back(classinfotest->counter);

    if(classinfotest->WalkFlag)
        SaveData();
}

void Strategy::inversekinmaticsinfo()
{
    classinfotest->points.IK_Point_RX = classinfotest->points.X_COM + classinfotest->points.X_Right_foot;
    classinfotest->points.IK_Point_RY = -1 * classinfotest->points.Y_COM + classinfotest->points.Y_Right_foot;
    classinfotest->points.IK_Point_RZ = classinfotest->points.Z_COM - classinfotest->points.Z_Right_foot;
    classinfotest->points.IK_Point_RThta = classinfotest->points.Right_Thta;
    classinfotest->points.IK_Point_LX = classinfotest->points.X_COM + classinfotest->points.X_Left_foot;
    classinfotest->points.IK_Point_LY = -1 * classinfotest->points.Y_COM + classinfotest->points.Y_Left_foot;
    classinfotest->points.IK_Point_LZ = classinfotest->points.Z_COM - classinfotest->points.Z_Left_foot;
    classinfotest->points.IK_Point_LThta = classinfotest->points.Left_Thta;
    /*
    classinfotest->points.IK_Point_RX = classinfotest->points.X_COM + classinfotest->points.X_Right_foot;
    classinfotest->points.IK_Point_RY = -1 * classinfotest->points.Y_COM + classinfotest->points.Y_Right_foot - 4;
    classinfotest->points.IK_Point_RZ = classinfotest->points.Z_COM - classinfotest->points.Z_Right_foot;
    classinfotest->points.IK_Point_RThta = classinfotest->points.Right_Thta;
    classinfotest->points.IK_Point_LX = classinfotest->points.X_COM + classinfotest->points.X_Left_foot;
    classinfotest->points.IK_Point_LY = -1 * classinfotest->points.Y_COM + classinfotest->points.Y_Left_foot + 4;
    classinfotest->points.IK_Point_LZ = classinfotest->points.Z_COM - classinfotest->points.Z_Left_foot;
    classinfotest->points.IK_Point_LThta = classinfotest->points.Left_Thta;
*/
}

string Strategy::DtoS(double value)
{
    string str;
    std::stringstream buf;
    buf << value;
    str = buf.str();

    return str;
}

void Strategy::SaveData()
{
    string savedText = "R_move_X\tL_move_X\t"
                       "R_move_Y\tL_move_Y\t"
                       "R_move_Z\tL_move_Z\t"
                       "move_COM_X\tmove_COM_Y\tmove_COM_Z\t"
                       "R_Thta\tL_Thta\tPoint\n";
    char filename[]="/home/shengru/Desktop/catkin_ws/src/test/Parameter/Trajectory_Record.ods";

    fstream fp;
    fp.open(filename, ios::out);

    fp<<savedText;

    for(int i = 0; i < test.size(); i++)
    {
        savedText = DtoS(osc_move_x_r[i]) + "\t"
                + DtoS(osc_move_x_l[i]) + "\t"
                + DtoS(osc_move_y_r[i]) + "\t"
                + DtoS(osc_move_y_l[i]) + "\t"
                + DtoS(osc_move_z_r[i]) + "\t"
                + DtoS(osc_move_z_l[i]) + "\t"
                + DtoS(osc_move_com_x[i]) + "\t"
                + DtoS(osc_move_com_y[i]) + "\t"
                + DtoS(osc_move_com_z[i]) + "\t"
                + DtoS(right_Thta[i]) + "\t"
                + DtoS(left_Thta[i]) + "\t"
                + DtoS(test[i]) + "\n";

        fp<<savedText;
    }

    osc_move_x_r.clear();
    osc_move_x_l.clear();
    osc_move_y_r.clear();
    osc_move_y_l.clear();
    osc_move_z_r.clear();
    osc_move_z_l.clear();
    osc_move_com_x.clear();
    osc_move_com_y.clear();
    osc_move_com_z.clear();
    right_Thta.clear();
    left_Thta.clear();
    test.clear();

    fp.close();
}

double Strategy::OSC_move_x_advance(double range, double period_T_ms, double rho_x, double BASE_delrho_x, double delta_x, int time_t_ms)
{
    double omega_x;
    double period_T = period_T_ms * 0.001;//sec
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_x = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < range * T_divby4)
    {
        return rho_x *(1 + BASE_delrho_x);
    }
    else if(time_t >= range * T_divby4 && time_t < (T_divby2 - (range * T_divby4)) )
    {
        if (time_t<=T_divby4)
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
        else
        {
            return rho_x* (1 - BASE_delrho_x) * sin(omega_x * (time_t - range * T_divby4)+delta_x);
        }
    }
    else if(time_t >= T_divby2 - (range * T_divby4) && time_t < T_divby2 + (range * T_divby4))
    {
        return -rho_x * (1 - BASE_delrho_x);
    }
    else if(time_t >= T_divby2 + (range * T_divby4) && time_t < period_T - (range * T_divby4))
    {
        if (time_t <= period_T - T_divby4)
        {
            return rho_x * (1 - BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
        else
        {
            return rho_x * (1 + BASE_delrho_x) * sin(omega_x * (time_t + (range * T_divby4) - period_T) + delta_x);
        }
    }
    else
    {
        return rho_x *(1 + BASE_delrho_x) ;

    }
}

double Strategy::OSC_move_z(double range, double period_T_ms, double rho_z, double delta_z, int time_t_ms)
{
    double omega_z;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % ( int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    omega_z = 2 * PI / period_T / (1-range);

    if(time_t >= 0 && time_t < T_divby2 + range * T_divby4)
    {
        return 0;
    }
    else if(time_t >= T_divby2 + range * T_divby4 && time_t < period_T-(range * T_divby4))
    {
        return rho_z * sin(omega_z * (time_t - range * 3 * T_divby4) + delta_z);
    }
    else
    {
        return 0;
    }
}

double Strategy::OSC_COM_X(double range, double period_T_ms, double rho_com_x, double delta_com_x, int time_t_ms)
{
    double omega_com_x;
    double rho_x;
    double period_T = period_T_ms * 0.001;
    double T_divbylock = period_T * (range/2);
    double T_divbyunlock = period_T * (1-range)/2;
    omega_com_x = 2 * PI / period_T;
    rho_x = rho_com_x * (range);
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    if(time_t >= 0 && time_t <  T_divbylock)
    {
        return rho_x * sin((2 * PI / (T_divbylock * 4)) * time_t + PI );
    }
    else if(time_t < period_T && time_t >  (period_T - T_divbylock))
    {
        return rho_x * sin( (1.5 * PI) + (2 * PI / (T_divbylock * 4)) * (time_t - (period_T - T_divbylock))+ PI );
    }
    else
    {
        return rho_x * sin( (PI / 2) + (2 * PI / (T_divbyunlock * 4)) * (time_t - T_divbylock)+ PI );
    }
}

double Strategy::OSC_move_shift_y(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    double omega_y;
    double period_T = period_T_ms * 0.001;
    double T_divby2 = period_T / 2;
    double T_divby4 = period_T / 4;
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;
    // new
    double r1 = T_divby4;
    double r2 = T_divby2 - (range * T_divby4);
    double r3 = T_divby2 + (range * T_divby4);
    double r4 = 3*T_divby4;
    omega_y = 2 * PI / period_T / (1-range);
    if (time_t >= 0 && time_t < range * T_divby4  )
    {
        return 0;
    }
    else if(time_t >=  range * T_divby4  && time_t < r1)
    {
        return rho_y * sin(omega_y * (time_t - range * T_divby4) + delta_y);
    }
    else if (time_t >= r1 && time_t < r3)
    {
        return rho_y;
    }
    else if (time_t >= r3 && time_t <= r4)
    {
        //return rho_y * sin(omega_y * (time_t - range * 3 * T_divby4 - period_T) + delta_y);
        return rho_y * sin(omega_y * (time_t - 3*T_divby4) + delta_y + PI);
    }
    else
    {
        return 0;
    }
}

double Strategy::OSC_COM_Y(double period_T_ms, double rho_com_y, double delta_com_y, int time_t_ms)
{
    double omega_com_y;
    double period_T = period_T_ms * 0.001;
    omega_com_y = 2 * PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_y * sin(omega_com_y * time_t + delta_com_y);
}

double Strategy::OSC_COM_Z(double period_T_ms, double rho_com_z, double delta_com_z, int time_t_ms)
{
    double omega_com_z;
    double period_T = period_T_ms * 0.001;
    omega_com_z =  PI / period_T;
    //int ms_period_T = (int)(period_T*1000);
    int time_t_temp = time_t_ms % (int)period_T_ms;
    double time_t = (double)time_t_temp / 1000;

    return rho_com_z * sin(omega_com_z * time_t);
}

double Strategy::OSC_Rotate(double range, double period_T_ms, double rho_y, double delta_y, int time_t_ms)
{
    return OSC_move_shift_y( range,  period_T_ms,  rho_y,  delta_y,  time_t_ms);
}
