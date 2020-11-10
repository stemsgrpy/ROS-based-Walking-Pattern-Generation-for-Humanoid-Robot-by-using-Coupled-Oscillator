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
int counter;

bool motionCallback(walkinggait::Serial::Request  &req, walkinggait::Serial::Response &res)
{
    ROS_INFO("Getparameter");

    classinfotest->parameters.R_Goal[0] = req.IK_Point_RX;
    classinfotest->parameters.R_Goal[1] = req.IK_Point_RY - 4;
    classinfotest->parameters.R_Goal[2] = 21.7 - req.IK_Point_RZ;
    //msg.IK_Point_RThta;
    classinfotest->parameters.L_Goal[0] = req.IK_Point_LX;
    classinfotest->parameters.L_Goal[1] = req.IK_Point_LY + 4;
    classinfotest->parameters.L_Goal[2] = 21.7 - req.IK_Point_LZ;
    //msg.IK_Point_LThta;

    strategy.DoIK();

    res.serialack = ++counter;
    classinfotest->rosflag = true;
    
    if(req.stopflag)
    {
        counter = 0;
        strategy.SaveData();
        printf("\nfinish\n");
    }

    return true;
}

int main(int argc, char **argv)
{
    counter = 0;
    ros::init(argc, argv, "IK");
    ros::NodeHandle n;

    ros::ServiceServer ik_server = n.advertiseService("ik_client", motionCallback);

    ros::Rate loop_rate(1000);

    while (ros::ok())
    {
        if(classinfotest->rosflag)
        {
            classinfotest->rosflag = false;
        }
        ros::spinOnce();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    return 0;
}
