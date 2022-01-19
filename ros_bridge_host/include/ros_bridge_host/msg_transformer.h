#pragma once

// c++
// http://www.cplusplus.com/reference/
//#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
//#include <tuple>
//#include <atomic>
#include <mutex>
//#include <thread>
#include <unordered_map>
//#include <codecvt>
//#include <complex>
//#include <exception>
//#include <functional>
//#include <initializer_list>
//#include <iterator>
//#include <limits>
//#include <locale>
//#include <new>
//#include <numeric>
//#include <random>
//#include <ratio>
//#include <regex>
//#include <stdexcept>
//#include <system_error>
//#include <typeindex>
//#include <typeinfo>
#include <type_traits>
//#include <utility>
//#include <valarray>
//#include <condition_variable>
//#include <future>

#include <zmq.h>

#include <jsoncpp/json/json.h>

// http://docs.ros.org/diamondback/api/roscpp/html/classros_1_1NodeHandle.html
#include <ros/ros.h>

// http://docs.ros.org/api/rosconsole/html/dir_16d3284317caf78c0e25eb64bbba3d38.html
#include <ros/assert.h>

// http://docs.ros.org/jade/api/tf/html/c++/namespacetf.html
#include <tf/tf.h>

// geometry_msgs
// http://docs.ros.org/api/geometry_msgs/html/index-msg.html
//#include <geometry_msgs/Accel.h>
//#include <geometry_msgs/AccelStamped.h>
//#include <geometry_msgs/AccelWithCovariance.h>
//#include <geometry_msgs/AccelWithCovarianceStamped.h>
//#include <geometry_msgs/Inertia.h>
//#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseWithCovariance.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <geometry_msgs/QuaternionStamped.h>
//#include <geometry_msgs/Transform.h>
//#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/TwistWithCovariance.h>
//#include <geometry_msgs/TwistWithCovarianceStamped.h>
//#include <geometry_msgs/Vector3.h>
//#include <geometry_msgs/Vector3Stamped.h>
//#include <geometry_msgs/Wrench.h>
//#include <geometry_msgs/WrenchStamped.h>

// nav_msgs
// http://docs.ros.org/kinetic/api/nav_msgs/html/index-msg.html
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//#include <nav_msgs/GridCells.h>
// srv
//#include <nav_msgs/GetMap.h>
//#include <nav_msgs/GetPlan.h>
//#include <nav_msgs/SetMap.h>

// http://docs.ros.org/jade/api/angles/html/namespaceangles.html
#include <angles/angles.h>

// sensor_msgs
// http://docs.ros.org/jade/api/sensor_msgs/html/index-msg.html
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
//#include <sensor_msgs/BatteryState.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/ChannelFloat32.h>
//#include <sensor_msgs/CompressedImage.h>
//#include <sensor_msgs/FluidPressure.h>
//#include <sensor_msgs/Illuminance.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/Joy.h>
//#include <sensor_msgs/JoyFeedback.h>
//#include <sensor_msgs/JoyFeedbackArray.h>
//#include <sensor_msgs/LaserEcho.h>
//#include <sensor_msgs/MagneticField.h>
//#include <sensor_msgs/MultiDOFJointState.h>
//#include <sensor_msgs/MultiEchoLaserScan.h>
//#include <sensor_msgs/NavSatFix.h>
//#include <sensor_msgs/NavSatStatus.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/PointField.h>
//#include <sensor_msgs/Range.h>
//#include <sensor_msgs/RegionOfInterest.h>
//#include <sensor_msgs/RelativeHumidity.h>
//#include <sensor_msgs/Temperature.h>
//#include <sensor_msgs/TimeReference.h>
//srv
//#include <sensor_msgs/SetCameraInfo.h>

// visualization_msgs
// http://docs.ros.org/api/visualization_msgs/html/index-msg.html
//#include <visualization_msgs/ImageMarker.h>
//#include <visualization_msgs/InteractiveMarker.h>
//#include <visualization_msgs/InteractiveMarkerControl.h>
//#include <visualization_msgs/InteractiveMarkerFeedback.h>
//#include <visualization_msgs/InteractiveMarkerInit.h>
//#include <visualization_msgs/InteractiveMarkerPose.h>
//#include <visualization_msgs/InteractiveMarkerUpdate.h>
//#include <visualization_msgs/MenuEntry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//#include <dynamic_reconfigure/server.h>

// tf
// http://docs.ros.org/en/api/tf/html/index-msg.html
#include <tf/tfMessage.h>

// eigen
// #include "Eigen/Dense"

// pluginlib
#include <pluginlib/class_list_macros.h>

namespace ros_bridge_host
{
    const std::string TYPE_STR = "type";
    const std::string TOPIC_NAME_STR = "topic_name";
    const std::string HEADER_STR = "header";
    const std::string FRAME_ID_STR = "frame_id";

    inline void _deserialize_point_stamped(Json::Value &root, const ros::Publisher &publisher)
    {
        geometry_msgs::PointStamped ps;
        ps.point.x = root["point"]["x"].asFloat();
        ps.point.y = root["point"]["y"].asFloat();
        ps.point.z = root["point"]["z"].asFloat();

        ps.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        ps.header.stamp = ros::Time::now();
        publisher.publish(ps);
    }

    inline void _deserialize_path(Json::Value &root, const ros::Publisher &publisher)
    {
        nav_msgs::Path msg;
        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped ps;
        ps.pose.position.z = 0.0;
        const auto &poses = root["poses"];
        for (const auto &p : poses)
        {
            ps.pose.position.x = p["x"].asFloat();
            ps.pose.position.y = p["y"].asFloat();
            msg.poses.emplace_back(ps);
        }

        publisher.publish(msg);
    }

    inline void _deserialize_tf_msg(Json::Value &root, const ros::Publisher &publisher)
    {
        tf::tfMessage msg;
        geometry_msgs::TransformStamped ts;
        ts.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        ts.header.stamp = ros::Time::now();
        ts.child_frame_id = root["child_frame_id"].asString();
        auto &transforms = root["transforms"];
        for (auto &t : transforms)
        {
            ts.transform.rotation = tf::createQuaternionMsgFromYaw(t["rotation"]["theta"].asFloat());
            ts.transform.translation.x = t["translation"]["x"].asFloat();
            ts.transform.translation.y = t["translation"]["y"].asFloat();
            ts.transform.translation.z = t["translation"]["z"].asFloat();

            msg.transforms.emplace_back(ts);
        }

        publisher.publish(msg);
    }

    inline void _deserialize_scan(Json::Value &root, const ros::Publisher &publisher)
    {
        sensor_msgs::LaserScan msg;
        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        msg.scan_time = root["scan_time"].asFloat();
        msg.time_increment = root["time_increment"].asFloat();
        msg.angle_increment = root["angle_increment"].asFloat();
        msg.angle_max = root["angle_max"].asFloat();
        msg.angle_min = root["angle_min"].asFloat();
        msg.range_max = root["range_max"].asFloat();
        msg.range_min = root["range_min"].asFloat();
        auto &ranges = root["ranges"];
        for (auto &r : ranges)
        {
            msg.ranges.emplace_back(r.asFloat());
        }

        auto &intensities = root["intensities"];
        for (auto &i : intensities)
        {
            msg.intensities.emplace_back(i.asFloat());
        }

        publisher.publish(msg);
    }

    inline void _deserialize_marker(Json::Value &root, const ros::Publisher &publisher)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        msg.ns = root["ns"].asString();
        msg.id = root["id"].asInt();
        msg.action = root["action"].asInt();
        msg.type = root["marker_type"].asInt();
        msg.text = root["text"].asString();
        msg.lifetime = ros::Duration(root["lifetime"].asFloat());

        msg.color.r = root["color"]["r"].asFloat();
        msg.color.g = root["color"]["g"].asFloat();
        msg.color.b = root["color"]["b"].asFloat();
        msg.color.a = root["color"]["a"].asFloat();

        msg.scale.x = root["scale"]["x"].asFloat();
        msg.scale.y = root["scale"]["y"].asFloat();
        msg.scale.z = root["scale"]["z"].asFloat();

        const auto &colors = root["colors"];
        visualization_msgs::Marker::_color_type color;
        for (const auto &c : colors)
        {
            color.r = colors["r"].asFloat();
            color.g = colors["g"].asFloat();
            color.b = colors["b"].asFloat();
            color.a = colors["a"].asFloat();

            msg.colors.emplace_back(color);
        }

        msg.pose.position.x = root["pose"]["position"]["x"].asFloat();
        msg.pose.position.y = root["pose"]["position"]["y"].asFloat();
        msg.pose.position.z = root["pose"]["position"]["z"].asFloat();
        msg.pose.orientation.x = root["pose"]["orientation"]["x"].asFloat();
        msg.pose.orientation.y = root["pose"]["orientation"]["y"].asFloat();
        msg.pose.orientation.z = root["pose"]["orientation"]["z"].asFloat();
        msg.pose.orientation.w = root["pose"]["orientation"]["w"].asFloat();

        const auto &points = root["points"];
        geometry_msgs::Point point;
        for (const auto &p : points)
        {
            point.x = p["x"].asFloat();
            point.y = p["y"].asFloat();
            point.z = p["z"].asFloat();
            msg.points.emplace_back(point);
        }

        publisher.publish(msg);
    }

    inline void _deserialize_pose_stamped(Json::Value &root, const ros::Publisher &publisher)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        msg.pose.position.x = root["pose"]["position"]["x"].asFloat();
        msg.pose.position.y = root["pose"]["position"]["y"].asFloat();
        msg.pose.position.z = root["pose"]["position"]["z"].asFloat();
        msg.pose.orientation = tf::createQuaternionMsgFromYaw(root["pose"]["orientation"]["theta"].asFloat());

        publisher.publish(msg);
    }

    inline void _deserialize_pose_array(Json::Value &root, const ros::Publisher &publisher)
    {
        geometry_msgs::PoseArray msg;
        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        geometry_msgs::Pose ps;
        auto &poses = root["poses"];
        for (auto &p : poses)
        {
            ps.position.x = p["x"].asFloat();
            ps.position.y = p["y"].asFloat();
            ps.position.z = p["z"].asFloat();
            ps.orientation = tf::createQuaternionMsgFromYaw(p["theta"].asFloat());

            msg.poses.emplace_back(ps);
        }

        publisher.publish(msg);
    }

    inline void _deserialize_polygon(Json::Value &root, const ros::Publisher &publisher)
    {
        geometry_msgs::PolygonStamped msg;

        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        geometry_msgs::Point32 point;

        auto &points = root["points"];
        for (auto &p : points)
        {
            point.x = p["x"].asFloat();
            point.y = p["y"].asFloat();
            point.z = 0.0f;

            msg.polygon.points.emplace_back(point);
        }

        publisher.publish(msg);
    }

    inline void _deserialize_occupancy_grid(Json::Value &root, const ros::Publisher &publisher)
    {
        nav_msgs::OccupancyGrid msg;

        msg.header.frame_id = root[HEADER_STR][FRAME_ID_STR].asString();
        msg.header.stamp = ros::Time::now();

        msg.info.height = root["height"].asInt();
        msg.info.width = root["width"].asInt();
        msg.info.resolution = root["resolution"].asFloat();
        msg.info.origin.position.x = root["position"]["x"].asFloat();
        msg.info.origin.position.y = root["position"]["y"].asFloat();
        msg.info.origin.position.z = root["position"]["z"].asFloat();
        msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(root["orientation"]["theta"].asFloat());

        const auto& str = root["data"].asString();
        msg.data.assign(str.begin(), str.end());

        publisher.publish(msg);
    }
}