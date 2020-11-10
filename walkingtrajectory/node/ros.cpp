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

/*****************************************************************************
** Implementation
*****************************************************************************/
Strategy strategy;
ros::ServiceClient ik_client;

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

bool test(walkinggait::Update::Request  &req, walkinggait::Update::Response &res) {
ROS_INFO("tra");

    char line[SIZE];

    fstream fin;

    if( req.walking_mode )//reload
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

    classinfotest->X = req.x;
    classinfotest->Y = req.y;
    classinfotest->Z = req.z;
    classinfotest->THTA = req.thta;
    classinfotest->time_point_ = req.time_point_;
    classinfotest->walking_mode = req.walking_mode;
    classinfotest->counter = req.counter;
    classinfotest->WalkFlag = req.stopflag;
    classinfotest->complan.isfirststep = req.isfirststep;
    classinfotest->complan.islaststep = req.islaststep;

    strategy.walkingprocess(classinfotest->walking_mode);

    walkinggait::Serial srv;
    srv.request.X_Right_foot = classinfotest->points.X_Right_foot;
    srv.request.X_Left_foot = classinfotest->points.X_Left_foot;
    srv.request.Y_Right_foot = classinfotest->points.Y_Right_foot;
    srv.request.Y_Left_foot = classinfotest->points.Y_Left_foot;
    srv.request.Z_Right_foot = classinfotest->points.Z_Right_foot;
    srv.request.Z_Left_foot = classinfotest->points.Z_Left_foot;
    srv.request.X_COM = classinfotest->points.X_COM;
    srv.request.Y_COM = classinfotest->points.Y_COM;
    srv.request.Z_COM = classinfotest->points.Z_COM;
    srv.request.Right_Thta = classinfotest->points.Right_Thta;
    srv.request.Left_Thta = classinfotest->points.Left_Thta;
    srv.request.counter = classinfotest->counter;
    srv.request.stopflag = classinfotest->WalkFlag;
    srv.request.IK_Point_RX = classinfotest->points.IK_Point_RX;
    srv.request.IK_Point_RY = classinfotest->points.IK_Point_RY;
    srv.request.IK_Point_RZ = classinfotest->points.IK_Point_RZ;
    srv.request.IK_Point_RThta = classinfotest->points.IK_Point_RThta;
    srv.request.IK_Point_LX = classinfotest->points.IK_Point_LX;
    srv.request.IK_Point_LY = classinfotest->points.IK_Point_LY;
    srv.request.IK_Point_LZ = classinfotest->points.IK_Point_LZ;
    srv.request.IK_Point_LThta = classinfotest->points.IK_Point_LThta;

    srv.response.serialack = false;

    ik_client.call(srv);

    ROS_INFO("counter : %d",srv.request.counter);

    //    if(srv.response.serialack)
    //    {
    //        srv.response.serialack = false;
    //    }
    //    else
    //    {
    //        ik_client.call(srv);
    //    }
        ROS_INFO("%d  while counter : %d",srv.response.serialack,srv.request.counter);
    while(srv.response.serialack != srv.request.counter)
    {
        ROS_INFO("%d  while counter : %d",srv.response.serialack,srv.request.counter);
        if(classinfotest->WalkFlag)
            break;
        ik_client.call(srv);
    }
    srv.response.serialack = false;

    (req.stopflag)?res.cpgack = false:res.cpgack = true;

    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "WalkingTrajectory");
    ros::NodeHandle n;

    ros::ServiceServer test_server = n.advertiseService("test_client", test);
    ik_client = n.serviceClient<walkinggait::Serial>("ik_client");

    ros::Rate loop_rate(1000);

    //    classinfotest->doflag = false;
    //    classinfotest->counter = 100;
    //    WalkingCPG::Serial srv;

    //    WalkingCPG::Update aa;
    //    WalkingCPG::Update::Request  req;
    //    WalkingCPG::Update::Response res;

    while (ros::ok())
    {

        //        if(classinfotest->counter == 1)
        //        {
        //            ROS_INFO("11111111111");
        //            classinfotest->doflag = true;
        //            ROS::test(req,res);
        //            classinfotest->doflag = false;
        //        }
        //        else if(srv.response.serialack)
        //        {
        //            classinfotest->doflag = true;
        //            ROS::test(req,res);
        //            classinfotest->doflag = false;
        //            srv.response.serialack = false;
        //        }
        //        else
        //            if(classinfotest->rosflag)//&& srv.response.serialack
        //        {
        //            classinfotest->rosflag = false;
        //            srv.request.X_Right_foot = classinfotest->points.X_Right_foot;
        //            srv.request.X_Left_foot = classinfotest->points.X_Left_foot;
        //            srv.request.Y_Right_foot = classinfotest->points.Y_Right_foot;
        //            srv.request.Y_Left_foot = classinfotest->points.Y_Left_foot;
        //            srv.request.Z_Right_foot = classinfotest->points.Z_Right_foot;
        //            srv.request.Z_Left_foot = classinfotest->points.Z_Left_foot;
        //            srv.request.X_COM = classinfotest->points.X_COM;
        //            srv.request.Y_COM = classinfotest->points.Y_COM;
        //            srv.request.Z_COM = classinfotest->points.Z_COM;
        //            srv.request.Right_Thta = classinfotest->points.Right_Thta;
        //            srv.request.Left_Thta = classinfotest->points.Left_Thta;
        //            srv.request.counter = classinfotest->counter;
        //            srv.request.stopflag = classinfotest->WalkFlag;
        //            srv.request.IK_Point_RX = classinfotest->points.IK_Point_RX;
        //            srv.request.IK_Point_RY = classinfotest->points.IK_Point_RY;
        //            srv.request.IK_Point_RZ = classinfotest->points.IK_Point_RZ;
        //            srv.request.IK_Point_RThta = classinfotest->points.IK_Point_RThta;
        //            srv.request.IK_Point_LX = classinfotest->points.IK_Point_LX;
        //            srv.request.IK_Point_LY = classinfotest->points.IK_Point_LY;
        //            srv.request.IK_Point_LZ = classinfotest->points.IK_Point_LZ;
        //            srv.request.IK_Point_LThta = classinfotest->points.IK_Point_LThta;

        //            ik_client.call(srv);
        //        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    return 0;
}
