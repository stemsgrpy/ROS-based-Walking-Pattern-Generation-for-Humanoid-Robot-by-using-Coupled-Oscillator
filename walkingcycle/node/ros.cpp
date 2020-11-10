/**
 * @file /qlistener/listener.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros.hpp"

#define PI 3.1415926535897932384626433832795	//pi

/*****************************************************************************
** Implementation
*****************************************************************************/
Strategy strategy;

float readvalue(fstream &fin,string title,bool mode)
{
    char line[100];
    char equal;
    while(1)
    {
        fin.getline(line,sizeof(line),' ');
        if((string)line == title)
        {
            fin.get(equal);
            if(equal == '=')
            {
                fin.getline(line,sizeof(line),'\n');
                break;
            }
        }
    }
    return (mode == false)?atoi(line):atof(line);
}

void Getparameter(const walkinggait::Interface& msg) 
{

    char line[SIZE];

    fstream fin;

    ROS_INFO("Getparameter");

    classinfotest->X = (double)msg.x/1000;
    classinfotest->Y = (double)msg.y/1000;
    classinfotest->Z = (double)msg.z/1000;
    classinfotest->THTA = (double)msg.thta/180*PI;
    classinfotest->walking_mode = msg.walking_mode;
    //classinfotest->complan.walking_state = msg.walking_state; //20160218


    if( classinfotest->walking_mode )//reload
    {
        fin.open("/home/shengru/Desktop/catkin_ws/src/test/Parameter/Continuous_Parameter.ini",ios::in);
        fin.getline(line,sizeof(line),'\n');

        classinfotest->parameters.X_Swing_Range = readvalue(fin,"X_Swing_Range",1);
        classinfotest->parameters.Y_Swing_Range = readvalue(fin,"Y_Swing_Range",1);
        classinfotest->parameters.Z_Swing_Range = readvalue(fin,"Z_Swing_Range",1);
        classinfotest->parameters.Period_T = readvalue(fin,"Period_T",0);
        classinfotest->parameters.Period_T2 = readvalue(fin,"Period_T2",0);
        classinfotest->parameters.Period_COM_Y = readvalue(fin,"Period_COM_Y",0);
        classinfotest->parameters.Sample_Time = readvalue(fin,"Sample_Time",0);
        classinfotest->parameters.OSC_LockRange = readvalue(fin,"OSC_LockRange",1);
        classinfotest->parameters.BASE_Default_Z = readvalue(fin,"BASE_Default_Z",1);
    }
    else
    {
        fin.open("/home/shengru/Desktop/catkin_ws/src/test/Parameter/Single_Parameter.ini",ios::in);
        fin.getline(line,sizeof(line),'\n');

        classinfotest->parameters.X_Swing_Range = readvalue(fin,"X_Swing_Range",1);
        classinfotest->parameters.Y_Swing_Range = readvalue(fin,"Y_Swing_Range",1);
        classinfotest->parameters.Z_Swing_Range = readvalue(fin,"Z_Swing_Range",1);
        classinfotest->parameters.Period_T = readvalue(fin,"Period_T",0);
        classinfotest->parameters.Period_T2 = readvalue(fin,"Period_T2",0);
        classinfotest->parameters.Period_COM_Y = readvalue(fin,"Period_COM_Y",0);
        classinfotest->parameters.Sample_Time = readvalue(fin,"Sample_Time",0);
        classinfotest->parameters.OSC_LockRange = readvalue(fin,"OSC_LockRange",1);
        classinfotest->parameters.BASE_Default_Z = readvalue(fin,"BASE_Default_Z",1);
    }

    //    cout<<classinfotest->parameters.X_Swing_Range<<endl;
    //    cout<<classinfotest->parameters.Y_Swing_Range<<endl;
    //    cout<<classinfotest->parameters.Z_Swing_Range<<endl;
    //    cout<<classinfotest->parameters.Period_T<<endl;
    //    cout<<classinfotest->parameters.Period_T2<<endl;
    //    cout<<classinfotest->parameters.Period_COM_Y<<endl;
    //    cout<<classinfotest->parameters.Sample_Time<<endl;
    //    cout<<classinfotest->parameters.OSC_LockRange<<endl;
    //    cout<<classinfotest->parameters.BASE_Default_Z<<endl;
    ROS_INFO("I heard: [%d %d %d %d %d %d]", msg.x,msg.y,msg.z,msg.thta,msg.walking_mode,msg.walking_state);
    ROS_INFO("I heard: [%f %f %f %d %d %d %d %f %f]", classinfotest->parameters.X_Swing_Range,classinfotest->parameters.Y_Swing_Range,classinfotest->parameters.Z_Swing_Range
             , classinfotest->parameters.Period_T,classinfotest->parameters.Period_T2,classinfotest->parameters.Period_COM_Y
             , classinfotest->parameters.Sample_Time,classinfotest->parameters.OSC_LockRange,classinfotest->parameters.BASE_Default_Z);
    if(classinfotest->complan.walking_state == StopStep)//20160218
    {
        classinfotest->complan.walking_state = StartStep;
        classinfotest->counter = 0;
        classinfotest->FpgaFlag = true;
        strategy.walkingkindfunction(msg.walking_mode);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WalkingCycle");
    ros::NodeHandle n;

    ros::Subscriber chatter_subscriber = n.subscribe("SendBodyAuto_Topic", 1000, Getparameter);

    ros::ServiceClient test_client = n.serviceClient<walkinggait::Update>("test_client");

    ros::Rate loop_rate(1000);

    walkinggait::Update srv;

    while (ros::ok())
    {
        if(srv.response.cpgack == true )//&& classinfotest->WalkFlag == true)//if(srv.response.cpgack == true)
        {
            ROS_INFO("cycle 1");
            srv.response.cpgack = false;
            classinfotest->WalkFlag = false;
            strategy.walkingkindfunction(classinfotest->walking_mode);
        }
        else if(classinfotest->rosflag)
        {
            ROS_INFO("cycle 2");
            classinfotest->rosflag = false;
            srv.request.x = classinfotest->XUpdate;
            srv.request.y = classinfotest->YUpdate;
            srv.request.z = classinfotest->ZUpdate;
            srv.request.thta = classinfotest->THTAUpdate;
            srv.request.time_point_ = classinfotest->complan.time_point_;
            srv.request.walking_mode = classinfotest->walking_mode;//classinfotest->counter;
            srv.request.counter = classinfotest->counter;
            //data.recordflag = classinfotest->WalkFlag;
            srv.request.stopflag = classinfotest->complan.walking_stop;
            //srv.request.recordflag = classinfotest->FpgaFlag;
            srv.request.isfirststep = classinfotest->complan.isfirststep;
            srv.request.islaststep = classinfotest->complan.islaststep;

            test_client.call(srv);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    return 0;
}
