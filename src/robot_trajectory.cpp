//author : rescuer liao
//https://github.com/rescuer-liao
//date : 2016 - 1 - 24
//Team Explorer(rescue robot)
//Team Unware (NWPU Basketball robot)
//this package record the trajectory which robot has passed
//via getting transform between odom and base_link

//#define DEBUG
#include <basketball_trajectory_srv/robot_trajectory.h>

RobotPathSrv::RobotPathSrv(ros::NodeHandle &nh)
    :nh_(nh)
{
    ros::NodeHandle private_nh_("~") ;

    private_nh_.param("target_frame_name",target_frame_name_ , string("/odom")) ;
    private_nh_.param("source_frame_name",source_frame_name_ , string("/base_link")) ;
    private_nh_.param("trajectory_update_rate",trajectory_update_rate_,5.0) ;
    private_nh_.param("trajectory_publish_rate",trajectory_publish_rate_,10.0) ;
    
    waitTF();
    
    moved_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/moved_trajectory",10) ;
    
    moved_trajectory_srv_ = nh_.advertiseService("/moved_trajectory" , &RobotPathSrv::trajectorySrvCallBack,this) ; 

    trajectory_update_timer_ = nh_.createTimer(ros::Duration(1/trajectory_update_rate_) , &RobotPathSrv::updateTrajectoryCallBack,
                                               this , false) ; 
    moved_trajectory_moved_timer_ = nh_.createTimer(ros::Duration(1/trajectory_publish_rate_),&RobotPathSrv::publishTrajectoryCallBack,
                                                    this , false) ;
    rep_.moved_trajectory.header.frame_id = target_frame_name_ ;

    current_robot_pose_.header.frame_id = source_frame_name_ ;
    current_robot_pose_.pose.orientation.w = 1.0 ;
}

RobotPathSrv::~RobotPathSrv()
{
    nh_.shutdown(); 
}

void RobotPathSrv::waitTF()
{
    ros::Time start_time = ros::Time::now() ; 
    
    ROS_INFO("[basketball_trajectory_srv] waiting for transform between %s and %s become available",
             target_frame_name_.c_str() , source_frame_name_.c_str()) ; 
    bool transform_successed = false ; 
    while(!transform_successed)
    {
        transform_successed = tf_listener_.canTransform(target_frame_name_,source_frame_name_,ros::Time()) ;
        if(transform_successed)break ; 
        ros::Time now = ros::Time::now() ; 
        
        if((now-start_time).toSec()>20)
        {
            ROS_WARN_ONCE("[basketball_trajectory_srv] no transform find between %s and %s" , target_frame_name_.c_str() ,source_frame_name_.c_str()) ; 
        }
    }
    ROS_INFO("[basketball_trajectory_srv] find the transform between %s and %s!!!" , target_frame_name_.c_str() , source_frame_name_.c_str()) ;
}


void RobotPathSrv::addNewPose()
{
    current_robot_pose_.header.stamp = ros::Time(0) ;

    geometry_msgs::PoseStamped p_out ;
    tf_listener_.transformPose(target_frame_name_,current_robot_pose_,p_out) ;

    if(rep_.moved_trajectory.poses.size()!=0)
    {
	#ifdef DEBUG
        ROS_INFO("[basketball_trajectory_srv] get new pose !! and the current pose size is %lu\n" , rep_.moved_trajectory.poses.size()) ; 
	#endif        
	if(p_out.header.stamp != rep_.moved_trajectory.poses.back().header.stamp)
        {
            rep_.moved_trajectory.poses.push_back(p_out) ;
        }
    }
    else
    {
	rep_.moved_trajectory.poses.push_back(p_out) ;
    }
    rep_.moved_trajectory.header.stamp = p_out.header.stamp ;
}

void RobotPathSrv::publishTrajectoryCallBack(const ros::TimerEvent &event)
{
    moved_trajectory_pub_.publish(rep_.moved_trajectory) ;
}

void RobotPathSrv::updateTrajectoryCallBack(const ros::TimerEvent &event)
{
    addNewPose();
}

bool RobotPathSrv::trajectorySrvCallBack(basketball_msgs::basketball_nav_srv::Request &req, basketball_msgs::basketball_nav_srv::Response &rep)
{
    rep  = rep_ ;
    return true ;
}
int main(int argc , char **argv)
{
    ros::init(argc , argv , "robot_trajectory_server") ;
    ros::NodeHandle node ;
    RobotPathSrv path_srv(node) ;
    ros::spin() ;
    return  0 ;
}
