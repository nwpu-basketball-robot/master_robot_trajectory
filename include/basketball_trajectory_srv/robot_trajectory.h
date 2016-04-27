/*
*Team Unware Basketball Robot NWPU
*
*根据机器人实时里程计信息来生成机器人已经移动的路径
*
*Author = liao-zhihan
*
*first_debug_date:2016-01-23
*测试通过:date 2016-03
*
*/


#ifndef ROBOT_TRAJECTORY
#define ROBOT_TRAJECTORY

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <algorithm>
#include <string>
#include <basketball_msgs/basketball_nav_srv.h>

using namespace std ;

class RobotPathSrv
{
public:
    RobotPathSrv(ros::NodeHandle &nh) ;
    ~RobotPathSrv() ;
protected:
private:
    //路径更新定时器的回调函数
    void updateTrajectoryCallBack(const ros::TimerEvent& event) ;
    //发布新路径定时器的回调函数
    void publishTrajectoryCallBack(const ros::TimerEvent& event) ;
    //向原有路径中，添加新的点
    void addNewPose() ;
    void waitTF() ;
    //服务回调函数
    bool trajectorySrvCallBack(basketball_msgs::basketball_nav_srv::Request &req,
                               basketball_msgs::basketball_nav_srv::Response &rep) ;
private:
    double trajectory_update_rate_ ;
    double trajectory_publish_rate_ ;
    basketball_msgs::basketball_nav_srv::Response rep_ ;

    geometry_msgs::PoseStamped current_robot_pose_ ;
    ros::NodeHandle nh_ ;
    string target_frame_name_ ;
    string source_frame_name_ ;

    ros::Publisher moved_trajectory_pub_ ;

    ros::Timer trajectory_update_timer_ ;
    ros::Timer moved_trajectory_moved_timer_ ;
    ros::ServiceServer moved_trajectory_srv_ ;

    tf::TransformListener tf_listener_ ;
} ;
#endif // ROBOT_TRAJECTORY
