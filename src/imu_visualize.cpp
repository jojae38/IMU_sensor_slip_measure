#include "imu_visualize.hpp"

IMU_visual::IMU_visual()
{
    ros::NodeHandle _nh("~");
    _nh.param("Slip", _slip_rate, 0.3);
    _nh.param("Slip", _no_init, 1);
    slip=false;
    slip_occured=false;
    init_finish=false;
    timer_start=false;

    adjust_occured=false;

    imu_callback=false;
    robot_callback=true;
    
    init_diff=0;
    adjust_diff=0;

    vector<double> temp(3);
    vector<sensor_msgs::Imu> temp_val(4);
    slip_timer=temp;
    slip_d_val=temp_val;
    index=0;

    memset(&diff_info_,0,sizeof(struct diff_info));
    // _pub_robot_adjust=_nh.advertise<geometry_msgs::Pose>("/robot_adjust_byslip",1);
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);
    _sub_imu_data=_nh.subscribe("/imu_read",1, &IMU_visual::update_IMU, this);
    _sub_robot_data=_nh.subscribe("/odom",1,&IMU_visual::update_ROBOT,this);
    //test purpose//
    // _sub_robot_adjust=_nh.subscribe("/robot_adjust_byslip",5,&IMU_visual::adjust_ROBOT,this);

    run_sequence();
}
IMU_visual::~IMU_visual()
{

}
void IMU_visual::run_sequence()
{   
    ros::Rate rate(30);
    
    while(ros::ok())
    {
        if(imu_callback&&robot_callback)
        {
            calc_acc();
            if(_no_init)
            {
                    get_init_val();
                if(init_finish==true&&time_duration(1.0))
                    calc_slip_time();
            }
            else
            {
                calc_slip_time();
                
            }
            show_slip();
            turn_curr_to_prev();
        }
        rate.sleep();
        ros::spinOnce();
    }
}
void IMU_visual::get_init_val()
{
    if(init_finish==false)
    {
        if(init_th.size()<=30)
        {
            init_th.push_back(diff_info_.curr_Q_yaw);
        }
        else
        {
            for(int i=0;i<init_th.size();i++)
            {
                init_diff+=init_th[i]/init_th.size();
            }
            init_finish=true;
            ROS_INFO("Delete init difference between robot and imu Sensor Value: %f",init_diff);
        }
    }
}
void IMU_visual::update_IMU(const sensor_msgs::Imu::ConstPtr &msg)
{
    diff_info_.Imu_val=*msg;
    imu_callback=true;
}
void IMU_visual::update_ROBOT(const nav_msgs::Odometry::ConstPtr &msg)
{
    diff_info_.Robot_val=*msg;
    robot_callback=true;
    
    //testing purpose - not connected to robot
    // diff_info_.adj_Robot_val.ppo.x=diff_info_.Robot_val.ppo.x;
    // diff_info_.adj_Robot_val.ppo.y=diff_info_.Robot_val.ppo.y;
    // diff_info_.adj_Robot_val.ppo.z=diff_info_.Robot_val.ppo.z;
    // diff_info_.adj_Robot_val.ppo.w=diff_info_.Robot_val.ppo.w;
    ROS_INFO("x: %f y: %f z: %f w: %f ",diff_info_.Robot_val.ppo.x,diff_info_.Robot_val.ppo.y,diff_info_.Robot_val.ppo.z,diff_info_.Robot_val.ppo.w);
    //connected to robot
    diff_info_.adj_Robot_val.ppo.x=diff_info_.Robot_val.ppo.x+diff_info_.add_Robot_val.ppo.x;
    diff_info_.adj_Robot_val.ppo.y=diff_info_.Robot_val.ppo.y+diff_info_.add_Robot_val.ppo.y;
    diff_info_.adj_Robot_val.ppo.z=diff_info_.Robot_val.ppo.z+diff_info_.add_Robot_val.ppo.z;
    diff_info_.adj_Robot_val.ppo.w=diff_info_.Robot_val.ppo.w+diff_info_.add_Robot_val.ppo.w;
}
void IMU_visual::adjust_ROBOT(const geometry_msgs::Pose::ConstPtr &msg)
{
    //testing purpose - not connected to robot
    // diff_info_.Robot_val.ppo.x+=msg->orientation.x;
    // diff_info_.Robot_val.ppo.y+=msg->orientation.y;
    // diff_info_.Robot_val.ppo.z+=msg->orientation.z;
    // diff_info_.Robot_val.ppo.w+=msg->orientation.w;

    //connected to robot
}
void IMU_visual::calc_slip_time()
{
    if(slip==false)
    {
        if(abs(diff_info_.curr_ddQ_yaw)>_slip_rate)//?
        {
            slip=true;
            slip_timer[0]=return_current_time();
            slip_d_val[0]=diff_info_.Imu_val;
            index++;
        }
    }
    else
    {
        bool run=false;
        if(diff_info_.curr_ddQ_yaw==0)
        {
            run=true;
        }
        else if(diff_info_.curr_ddQ_yaw>0&&diff_info_.prev_ddQ_yaw<0)
        {
            run=true;
        }
        else if(diff_info_.curr_ddQ_yaw<0&&diff_info_.prev_ddQ_yaw>0)
        {
            run=true;
        }
        if(run)
        {
            slip_timer[index]=return_current_time();
            slip_d_val[index]=diff_info_.Imu_val;
            index++;
            run=false;
        }
        if(slip_timer[2]!=0)
        {
            slip=false;
            index=0;
            slip_time=slip_timer[2]-slip_timer[0];
            #define Ori slip_d_val[3].orientation 
            
            Ori.x=slip_d_val[2].orientation.x-slip_d_val[0].orientation.x;
            Ori.y=slip_d_val[2].orientation.y-slip_d_val[0].orientation.y;
            Ori.z=slip_d_val[2].orientation.z-slip_d_val[0].orientation.z;
            Ori.w=slip_d_val[2].orientation.w-slip_d_val[0].orientation.w;
            publish_adjust(Ori.x,Ori.y,Ori.z,Ori.w);
            ROS_INFO("change x: %f y: %f z: %f w: %f",Ori.x,Ori.y,Ori.z,Ori.w);
        }
    }
}
void IMU_visual::calc_acc()
{
    //testing - diff_info_.Robot_val | Real - diff_info_.adj_Robot_val
    nav_msgs::Odometry temp_ROBOT_=diff_info_.adj_Robot_val;
    sensor_msgs::Imu temp_IMU_=diff_info_.Imu_val;
    tf::Pose pose;

    diff_info_.curr_Q=return_sub_pose(temp_ROBOT_,temp_IMU_);
    tf::poseMsgToTF(diff_info_.curr_Q, pose);
    diff_info_.curr_Q_yaw=tf::getYaw(pose.getRotation())-init_diff;
    if(adjust_occured)
    {
        diff_info_.prev_Q_yaw=diff_info_.curr_Q_yaw;
        adjust_occured=false;
    }
        if(diff_info_.curr_Q_yaw>Gap&&diff_info_.prev_Q_yaw<-Gap)
        {
            diff_info_.curr_Q_yaw=diff_info_.prev_Q_yaw;
        }
        else if(diff_info_.curr_Q_yaw<-Gap&&diff_info_.prev_Q_yaw>Gap)
        {
            diff_info_.curr_Q_yaw=diff_info_.prev_Q_yaw;
        }
    // tf::poseMsgToTF(diff_info_.prev_Q, pose);
    // diff_info_.prev_Q_yaw=tf::getYaw(pose.getRotation())-init_diff;

    diff_info_.curr_dQ_yaw=(diff_info_.curr_Q_yaw-diff_info_.prev_Q_yaw)*5;
    diff_info_.curr_ddQ_yaw=diff_info_.curr_dQ_yaw-diff_info_.prev_dQ_yaw;

    diff.angular.x=diff_info_.curr_Q_yaw;
    diff.angular.y=diff_info_.curr_dQ_yaw;
    diff.angular.z=diff_info_.curr_ddQ_yaw;

    _pub_differ.publish(diff);
}
bool IMU_visual::time_duration(double sec)
{
    if(init_finish==true)
    {
        if(timer_start==false)
        {
            start_time=ros::Time::now();
            timer_start=true;
        }
        else
        {
            ros::Time now_time=ros::Time::now();
            if(start_time.sec+sec<=now_time.sec)
                return true;
        }
    }
    return false;
}
void IMU_visual::publish_adjust(double x,double y, double z, double w)
{
    // geometry_msgs::Pose temp;
    // temp.orientation.x=x;
    // temp.orientation.y=y;
    // temp.orientation.z=z;
    // temp.orientation.w=w;
    // _pub_robot_adjust.publish(temp);

    diff_info_.add_Robot_val.ppo.x+=x;
    diff_info_.add_Robot_val.ppo.y+=y;
    diff_info_.add_Robot_val.ppo.z+=z;
    diff_info_.add_Robot_val.ppo.w+=w;
    adjust_occured=true;
    ROS_WARN("ADJUST_Needed");
    fill(slip_timer.begin(),slip_timer.end(),0);
}
double IMU_visual::return_current_time()
{
    return ros::Time::now().toSec();
}
geometry_msgs::Pose IMU_visual::return_sub_pose(geometry_msgs::Pose &A, geometry_msgs::Pose &B)
{
    geometry_msgs::Pose temp;

    temp.orientation.x=A.orientation.x-B.orientation.x;
    temp.orientation.y=A.orientation.y-B.orientation.y;
    temp.orientation.z=A.orientation.z-B.orientation.z;
    temp.orientation.w=A.orientation.w-B.orientation.w;

    return temp;
}
geometry_msgs::Pose IMU_visual::return_sub_pose(nav_msgs::Odometry &A, sensor_msgs::Imu &B)
{
    geometry_msgs::Pose temp;

    temp.orientation.x=A.pose.pose.orientation.x-B.orientation.x;
    temp.orientation.y=A.pose.pose.orientation.y-B.orientation.y;
    temp.orientation.z=A.pose.pose.orientation.z-B.orientation.z;
    temp.orientation.w=A.pose.pose.orientation.w-B.orientation.w;

    return temp;
}
void IMU_visual::turn_curr_to_prev()
{
    diff_info_.prev_Q=diff_info_.curr_Q;

    diff_info_.prev_Q_yaw=diff_info_.curr_Q_yaw;
    diff_info_.prev_dQ_yaw=diff_info_.curr_dQ_yaw;
    diff_info_.prev_ddQ_yaw=diff_info_.curr_ddQ_yaw;
}
void IMU_visual::show_slip()
{
    cv::namedWindow("Slip_Occur==RED");
    cv::moveWindow("Slip_Occur==RED",60,0);
    if(slip)
    {
        cv::Mat Red_map(cv::Size(100,100),CV_8UC3,{0,0,200});
        cv::imshow("Slip_Occur==RED",Red_map);
    }
    else
    {
        cv::Mat Green_map(cv::Size(100,100),CV_8UC3,{0,200,0});
        cv::imshow("Slip_Occur==RED",Green_map);
    }
    cv::waitKey(10)==27;
}