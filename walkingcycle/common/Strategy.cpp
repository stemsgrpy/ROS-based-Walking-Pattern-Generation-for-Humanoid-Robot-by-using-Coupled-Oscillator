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

    //    classaa = new Classtestaa();
    //    classbb = new Classtestbb();
    //    QObject::connect(qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    //    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    Sample_points_quater = 0;

}

Strategy::~Strategy()
{

}

void Strategy::walkingkindfunction(int walking_mode)
{
    //    ROS_INFO("switch");

    switch(walking_mode)
    {
    case 0://Single Step
        singlestepfunction();
        break;
    case 1://Continuous
        continuoustepfunction();
        break;
    case 2:
    default:
        break;
    }
}

void Strategy::singlestepfunction()
{
    //    ROS_INFO("singlestepfunction");

    classinfotest->complan.sample_point_++;
    classinfotest->complan.time_point_ = classinfotest->complan.sample_point_*(classinfotest->parameters.Period_T/classinfotest->parameters.Sample_Time);

    singlestepwalkingprocess();

    classinfotest->rosflag = true;
}

void Strategy::continuoustepfunction()
{
    //    ROS_INFO("continuoustepfunction");

    classinfotest->complan.sample_point_++;
    classinfotest->complan.time_point_ = classinfotest->complan.sample_point_*(classinfotest->parameters.Period_T/classinfotest->parameters.Sample_Time);

    //ROS_INFO("%d  %d",classinfotest->complan.time_point_,classinfotest->complan.sample_point_);
    continuouswalkingprocess();

    classinfotest->rosflag = true;
}

void Strategy::singlestepwalkingprocess()
{
    //    ROS_INFO("singlestepwalkingprocess");

    //wchar_t* statetext;
    switch(classinfotest->complan.walking_state)
    {
    case StartStep:
        //ROS_INFO("StartStep");
        //statetext = L"StartStep";
        classinfotest->complan.walking_stop = false;

        forwardValue_ = 0;
        slopeCounter_ = 0;
        forwardCounter_ = 0;

        if (!classinfotest->IsParametersLoad)//!WalkingProcess->IsParametersLoad
        {
            classinfotest->complan.isfirststep = true;
            classinfotest->IsParametersLoad = true;
            classinfotest->complan.islaststep = false;
            //            WalkingProcess->COM_Plan->isLfootfirst = false;
            //            WalkingProcess->Parameters = WalkingProcess->WlakingStateParameters[etMarkTimeParameter]; // Reload mark time parameters
            //            WalkingProcess->WalkingUpdate(); // Update walking value
            //            process_Value->X = WalkingProcess->X;
            //            process_Value->Y = WalkingProcess->Y;
            //            process_Value->Z = WalkingProcess->Z;
            //            process_Value->Theta = WalkingProcess->Thta;

            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->THTAUpdate = classinfotest->THTA;

            Lockrange_tmp = classinfotest->parameters.OSC_LockRange;
            COM_Y_tmp = classinfotest->parameters.Y_Swing_Range;
            classinfotest->parameters.OSC_LockRange  = classinfotest->parameters.OSC_LockRange;
            classinfotest->parameters.Y_Swing_Range = classinfotest->parameters.Y_Swing_Range + 0.5;

        }
        if (classinfotest->complan.time_point_ == classinfotest->parameters.Period_T2)
        {

            classinfotest->parameters.Y_Swing_Range = COM_Y_tmp;
            classinfotest->parameters.OSC_LockRange = Lockrange_tmp;
            //            WalkingProcess->Y = process_Value->Y;
            classinfotest->complan.isfirststep = false;

        }
        if (classinfotest->complan.time_point_ == classinfotest->parameters.Period_T*3/4) // The first Step
        {
            classinfotest->complan.walking_state = FirstStep;
            classinfotest->IsParametersLoad = false;
            slopeCounter_++;
            forwardValue_ = slopeCounter_*INCREASE_SLOPE;
            forwardCounter_ += forwardValue_;
            //            WalkingProcess->Y = process_Value->Y;
            //            WalkingProcess->Thta = process_Value->Theta;
        }
        else
        {
            classinfotest->XUpdate = forwardValue_;
        }

        break;
    case FirstStep:
        //ROS_INFO("FirstStep");
        //statetext = L"FirstStep";
        classinfotest->complan.isfirststep = false;

        //		// Load Mark Time Parameters
        //		if (!WalkingProcess->IsParametersLoad) {
        //			WalkingProcess->IsParametersLoad = true;
        //			WalkingProcess->Parameters = WalkingProcess->WlakingStateParameters[etMarkTimeParameter];
        //		}

        if (classinfotest->X > forwardCounter_ )
        {
            classinfotest->XUpdate = forwardValue_;
            if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T / 4)
            {
                classinfotest->complan.walking_state = Repeat;
                //				enable_repeat = false;
                classinfotest->IsParametersLoad = false;
                slopeCounter_++;
                forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                forwardCounter_ += forwardValue_;
            }
        }
        else
        {
            classinfotest->XUpdate = classinfotest->X;

            if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T / 4)
            {
                classinfotest->complan.walking_state = StopStep;
                classinfotest->IsParametersLoad = false;
                classinfotest->complan.islaststep = true;//rotate compensation
            }
        }
        break;
    case Repeat:
        //ROS_INFO("Repeat");
        //statetext = L"Repeat";
        if (classinfotest->X > forwardCounter_ )
        {
            classinfotest->XUpdate = forwardValue_;
            if(classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T/4
                    || classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T*3/4)
            {
                classinfotest->complan.walking_state = Repeat;
                //				//enable_repeat = false;
                classinfotest->IsParametersLoad = false;
                if (forwardValue_ >= WALK_MAX_DISTANCE)
                {
                    forwardValue_ = WALK_MAX_DISTANCE;
                }
                else
                {
                    slopeCounter_++;
                    forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                }
                forwardCounter_ += forwardValue_;
            }
        }
        else
        {
            classinfotest->THTAUpdate /= 1000;
            classinfotest->YUpdate /= 1000;      //JJ
            if(classinfotest->X == forwardCounter_)
            {
                classinfotest->XUpdate = forwardValue_;

            }
            else
            {
                classinfotest->XUpdate = classinfotest->X - (forwardCounter_ - forwardValue_);
            }

            if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T/ 4
                    || classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T*3/4)
            {
                classinfotest->complan.walking_state = StopStep;
                classinfotest->IsParametersLoad = false;
                classinfotest->complan.islaststep = true;//rotate compensation
            }
        }
        break;
    case StopStep:
        //ROS_INFO("StopStep");
        //statetext = L"StopStep";
        classinfotest->XUpdate = forwardCounter_ = 0;
        if(Sample_points_quater == classinfotest->parameters.Sample_Time/4 - 1)
        {
            classinfotest->complan.walking_state = StopStep;
            classinfotest->complan.walking_stop = true;
            classinfotest->IsParametersLoad = false;
            //classinfotest->WalkFlag = false;
            classinfotest->FpgaFlag = false;
            //        			this->WalkEnable = 0;
            //                  WalkingProcess->Parameters->Initial();
            Sample_points_quater = 0;
            classinfotest->complan.sample_point_ = 0;
        }
        else
        {
            Sample_points_quater++;
        }
        break;
    default:
        classinfotest->complan.walking_state = StopStep;
        break;
    }

    classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z ;//+ max(abs(classinfotest->THTAUpdate * 6),max(abs(classinfotest->XUpdate),abs(classinfotest->YUpdate)));

    classinfotest->counter++;
    //ROS_INFO("value !!!!  [%f %f %f %f    %d]", classinfotest->XUpdate,classinfotest->YUpdate,classinfotest->ZUpdate,classinfotest->THTAUpdate,classinfotest->counter);

    //ROS_INFO("%ls %d",statetext,classinfotest->counter);
}

void Strategy::continuouswalkingprocess()
{
    int ww;
    int rr;
    //    ROS_INFO("continuouswalkingprocess");
    switch (classinfotest->complan.walking_state){
    case StartStep:
        classinfotest->complan.walking_stop = false;
        if (!classinfotest->IsParametersLoad) {
            classinfotest->IsParametersLoad = true;

            //            WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z ;
            //            WalkingProcess->WalkingUpdate();

            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->THTAUpdate = classinfotest->THTA;

        }
        if (classinfotest->complan.time_point_ == classinfotest->parameters.Period_T / 4)
        {
            classinfotest->complan.walking_state = MarkTimeStep;
            classinfotest->IsParametersLoad = false;
        }
        else
        {
            classinfotest->XUpdate = 0;
        }

        break;
    case MarkTimeStep:
        //        // Load Mark Time Parameters
        if (!classinfotest->IsParametersLoad)
        {
            classinfotest->IsParametersLoad = true;
        }
        if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T / 2)
        {
            classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;//+ max(abs(classinfotest->THTAUpdate * 6),max(abs(classinfotest->XUpdate),abs(classinfotest->YUpdate)));

            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;//20160214
            classinfotest->THTAUpdate = classinfotest->THTA;

            classinfotest->XUpdate = 0;
            break;
        }
        if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T * 3 / 4)
        {
            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;//20160214
            classinfotest->THTAUpdate = classinfotest->THTA;

            if (classinfotest->XUpdate != 0)
            {
                classinfotest->complan.walking_state = ForwardStep;
                classinfotest->IsParametersLoad = false;
            } else
            {
                classinfotest->complan.walking_state = classinfotest->complan.walking_state;
            }
        }
    case ForwardStep:
        if (!classinfotest->IsParametersLoad)
        {
            classinfotest->IsParametersLoad = true;
        }

        if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T / 2)
        {
            //WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z + max(abs(WalkingProcess->Thta*4),max(abs(WalkingProcess->X),abs(WalkingProcess->Y)));
            //classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;
            classinfotest->parameters.OSC_LockRange = 0.25-(abs(classinfotest->XUpdate)/10);//  JJ
            if (classinfotest->parameters.OSC_LockRange < 0)
            {
                classinfotest->parameters.OSC_LockRange = 0;
            }

            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;//20160214
            classinfotest->THTAUpdate = classinfotest->THTA;

        }
        if ( classinfotest->complan.time_point_ % classinfotest->parameters.Period_T ==  classinfotest->parameters.Period_T *3 / 4)
        {
            classinfotest->XUpdate = classinfotest->X;
            classinfotest->YUpdate = classinfotest->Y;
            classinfotest->ZUpdate = classinfotest->Z;
            classinfotest->ZUpdate = classinfotest->parameters.BASE_Default_Z;//20160214
            classinfotest->THTAUpdate = classinfotest->THTA;

            if (classinfotest->XUpdate == 0) {
                classinfotest->complan.walking_state = MarkTimeStep;
                classinfotest->IsParametersLoad = false;
            } else {
                classinfotest->complan.walking_state = classinfotest->complan.walking_state;

                ROS_INFO("sample_point_ = %d",classinfotest->complan.sample_point_);
                rr = classinfotest->complan.sample_point_;
                /*if(classinfotest->complan.sample_point_ == 95)//25
                {
                    classinfotest->complan.walking_state = StopStep;

                    ww = (classinfotest->complan.time_point_/classinfotest->parameters.Period_T);
                    ROS_INFO("times = %d",ww);
                }*/
            }
        }
        break;
    case BackwardStep:
        //        if (!WalkingProcess->IsParametersLoad) {
        //            WalkingProcess->IsParametersLoad = true;
        //            WalkingProcess->Parameters = WalkingProcess->WlakingStateParameters[etBackwardParameter];
        //            WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z;
        //        }

        if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T / 2)
        {
            //WalkingProcess->WalkingUpdate();
        }
        if (classinfotest->complan.time_point_ % classinfotest->parameters.Period_T == classinfotest->parameters.Period_T *3 / 4)
        {
            //WalkingProcess->WalkingUpdate();
            if (classinfotest->XUpdate > 0) {
                classinfotest->complan.walking_state = ForwardStep;
                classinfotest->IsParametersLoad = false;
            } else if (classinfotest->XUpdate == 0) {
                classinfotest->complan.walking_state = MarkTimeStep;
                classinfotest->IsParametersLoad = false;
            } else {
                classinfotest->complan.walking_state = classinfotest->complan.walking_state;
            }
        }
        break;
    case StopStep:
        if (!classinfotest->IsParametersLoad) {
            classinfotest->IsParametersLoad = true;
        }
        classinfotest->XUpdate = 0;
        classinfotest->YUpdate = 0;
        classinfotest->THTAUpdate = 0;

        if((classinfotest->complan.time_point_/classinfotest->parameters.Period_T) == 4)// if(Times == 1)  JJ
        {
            classinfotest->complan.walking_state = StopStep;
            classinfotest->complan.walking_stop = true;
            classinfotest->IsParametersLoad = false;
            //classinfotest->WalkFlag = false;
            classinfotest->FpgaFlag = false;
            ROS_INFO("walking_stop = false");
            classinfotest->complan.sample_point_ = 0;
        }
        break;
    }

    ROS_INFO("times = %d",ww);
    ROS_INFO("sample_point_ = %d",rr);
    classinfotest->counter++;
    //ROS_INFO("value !!!!  [%f %f %f %f    %d]", classinfotest->XUpdate,classinfotest->YUpdate,classinfotest->ZUpdate,classinfotest->THTAUpdate,classinfotest->counter);
    ROS_INFO("%d\n",classinfotest->counter);

}

