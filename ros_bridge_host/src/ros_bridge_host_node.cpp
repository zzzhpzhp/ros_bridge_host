/**
 * @Description: Copyright (C) SHENZHEN Joylife Robot Co.
 * @Version: v0.01
 * @Author: luxi
 * @Date: 2021-04-05 14:04:21
 * @LastEditors: luxi
 * @LastEditTime: 2021-04-13 15:45:56
 */
#include <ros_bridge_host/ros_bridge_host.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSBridgeHost");

    ros::NodeHandle nh("~");

    ros_bridge_host::ROSBridgeHost ros_bridge_host;
    ros_bridge_host.start();

    return 0;
}

