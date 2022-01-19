#include <ros_bridge_host/ros_bridge_host.h>

namespace ros_bridge_host
{
    ROSBridgeHost::ROSBridgeHost()
    {
        nh_ = ros::NodeHandle("~");

        nh_.param("sub_address", sub_address, std::string("tcp://localhost:5556"));
        nh_.param("pub_address", pub_address, std::string("tcp://*:5557"));
        nh_.param("frequency", frequency_, 100.0f);
        interval_ = 1.0f / frequency_;
        ROS_INFO_STREAM("Subscribe address " << sub_address);
        ROS_INFO_STREAM("Publish address " << pub_address);
        ROS_INFO_STREAM("Message check frequency " << frequency_ << " Hz");
    }

    void ROSBridgeHost::start()
    {
        context_ = zmq_ctx_new();
        publisher_ = zmq_socket(context_, ZMQ_PUB);
        auto res = zmq_bind(publisher_, pub_address.c_str());

        subscriber_ = zmq_socket(context_, ZMQ_SUB);
        res = zmq_connect(subscriber_, sub_address.c_str());
        res = zmq_setsockopt(subscriber_, ZMQ_SUBSCRIBE, namespace_.data(), namespace_.size());

        cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&ROSBridgeHost::_cmd_cb, this, _1));
        goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(&ROSBridgeHost::_goal_cb, this, _1));

        while(ros::ok())
        {
            ros::spinOnce();
            sleep(interval_);

            auto header = _recv_data(subscriber_, ZMQ_DONTWAIT);
            if (!header)
            {
                continue;
            }
            free(header);
            auto body = _recv_data(subscriber_, ZMQ_DONTWAIT);
            if (!body)
            {
                continue;
            }

            sub_root_.clear();
            bool parse_res = reader_.parse(body, sub_root_);
            free(body);
            if (!parse_res)
            {
                std::cerr << "Environment config file parse failed." << std::endl;
            }
            else
            {
                auto type = sub_root_["type"].asString();
                auto topic_name = sub_root_["topic_name"].asString();

                if (!_subscriber_initialize(type, topic_name))
                {
                    continue;
                }
                _trans_and_pub(type, topic_name);
            }
        }
    }

    void ROSBridgeHost::_cmd_cb(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lg(publisher_mtx_);

        Json::Value pub_root;
        pub_root[TYPE_STR] = "geometry_msgs::Twist";
        pub_root[TOPIC_NAME_STR] = "/cmd_vel";
        pub_root["velocity"] = msg->linear.x;
        pub_root["angular"] = msg->angular.z;

        auto body = fw_.write(pub_root);

        zmq_send(publisher_, namespace_.data(), namespace_.size(), ZMQ_DONTWAIT | ZMQ_SNDMORE);
        zmq_send(publisher_, body.data(), body.size(), ZMQ_DONTWAIT);
    }

    void ROSBridgeHost::_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lg(publisher_mtx_);

        Json::Value pub_root;
        pub_root[TYPE_STR] = "geometry_msgs::PoseStamped";
        pub_root[TOPIC_NAME_STR] = "/move_base_simple/goal";
        pub_root[HEADER_STR][FRAME_ID_STR] = msg->header.frame_id;
        pub_root["x"] = msg->pose.position.x;
        pub_root["y"] = msg->pose.position.y;
        pub_root["theta"] = tf::getYaw(msg->pose.orientation);

        auto body = fw_.write(pub_root);
        zmq_send(publisher_, namespace_.data(), namespace_.size(), ZMQ_DONTWAIT | ZMQ_SNDMORE);
        zmq_send(publisher_, body.data(), body.size(), ZMQ_DONTWAIT);
    }

    ROSBridgeHost::~ROSBridgeHost()
    {
    }

    char *ROSBridgeHost::_recv_data(void *socket, int opt)
    {
        // 创建zmq_msg_t对象接收数据
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int size = zmq_msg_recv(&msg, socket, opt);
        if(size == -1)
        {
            zmq_msg_close(&msg);
            return nullptr;
        }

        // 将zmq_msg_t对象中的数据保存到字符串中
        char *data = (char*)malloc(size + 1);
        memcpy(data, zmq_msg_data(&msg), size);

        zmq_msg_close(&msg);
        data[size] = 0;

        return data;
    }

    bool ROSBridgeHost::_subscriber_initialize(std::string type, std::string topic_name)
    {
        if (topic_to_publisher_[topic_name])
        {
            return true;
        }

        bool res{false};

        if (type == "geometry_msgs::PointStamped")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<geometry_msgs::PointStamped>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "nav_msgs::Path")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<nav_msgs::Path>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "tf::tfMessage")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<tf::tfMessage>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "sensor_msgs::LaserScan")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::LaserScan>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "geometry_msgs::PoseStamped")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<geometry_msgs::PoseStamped>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "geometry_msgs::PoseArray")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<geometry_msgs::PoseArray>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "visualization_msgs::Marker")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<visualization_msgs::Marker>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "geometry_msgs::PolygonStamped")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<geometry_msgs::PolygonStamped>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else if (type == "nav_msgs::OccupancyGrid")
        {
            topic_to_publisher_[topic_name] =
                    std::make_shared<ros::Publisher>(nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, queue_size_));
            ROS_INFO_STREAM_ONCE("Received topic: " << topic_name);
            res = true;
        }
        else
        {
            std::cerr << "Don't support the topic type: " << type << std::endl;
        }


        return res;
    }

    void ROSBridgeHost::_trans_and_pub(std::string type, std::string topic_name)
    {
        if (type == "geometry_msgs::PointStamped")
        {
            _deserialize_point_stamped(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "nav_msgs::Path")
        {
            _deserialize_path(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "tf::tfMessage")
        {
            _deserialize_tf_msg(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "sensor_msgs::LaserScan")
        {
            _deserialize_scan(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "geometry_msgs::PoseStamped")
        {
            _deserialize_pose_stamped(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "geometry_msgs::PoseArray")
        {
            _deserialize_pose_array(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "visualization_msgs::Marker")
        {
            _deserialize_marker(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "geometry_msgs::PolygonStamped")
        {
            _deserialize_polygon(sub_root_, *topic_to_publisher_[topic_name]);
        }
        else if (type == "nav_msgs::OccupancyGrid")
        {
            _deserialize_occupancy_grid(sub_root_, *topic_to_publisher_[topic_name]);
        }
    }
}

