#pragma once
#include "msg_transformer.h"

namespace ros_bridge_host
{

class ROSBridgeHost
{
public:

    ROSBridgeHost();
    virtual ~ROSBridgeHost();

    void start();

private:

    std::string sub_address{};
    std::string pub_address{};
    float frequency_{100.0f}, interval_{1.0f / frequency_};
    ros::NodeHandle nh_;
    std::unordered_map<std::string, std::shared_ptr<ros::Publisher> > topic_to_publisher_;
    ros::Subscriber cmd_sub_, goal_sub_;

protected:

    std::string namespace_{"robot"};

    void *context_, *subscriber_, *publisher_;
    Json::Reader reader_;
    Json::Value sub_root_;
    Json::FastWriter fw_;

    int queue_size_{10};

    std::mutex publisher_mtx_;

    void _cmd_cb(const geometry_msgs::Twist::ConstPtr &msg);

    void _goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);

    static char *_recv_data(void *socket, int opt = 0);

    bool _subscriber_initialize(std::string type, std::string topic_name);

    void _trans_and_pub(std::string type, std::string topic_name);
};

}

