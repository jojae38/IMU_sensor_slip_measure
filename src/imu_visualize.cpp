#include "imu_visualize.hpp"

IMU_visual::IMU_visual()
{
    ros::NodeHandle _nh("~");
    _nh.param("Slip", _slip_rate, 0.3);
    slip=false;
    slip_occured=false;
    init_finish=false;
    timer_start=false;

    imu_callback=false;
    robot_callback=true;
    
    init_diff=0;
    adjust_diff=0;

    curr_dp_th=0;
    prev_dp_th=0;
    dpp_th=0;

    // Robot_Color_={100,220,100};
    // IMU_Color_={220,100,100};
    //  word={255,204,255};

    IMU_.orientation.x=0;
    IMU_.orientation.y=0;
    IMU_.orientation.z=0;
    IMU_.orientation.w=0;

    ROBOT_.orientation.x=0;
    ROBOT_.orientation.y=0;
    ROBOT_.orientation.z=0;
    ROBOT_.orientation.w=0;

   
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);
    _sub_imu_data=_nh.subscribe("/imu_read",1, &IMU_visual::update_IMU, this);
    _sub_robot_data=_nh.subscribe("/robot_read",1,&IMU_visual::update_ROBOT,this);
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
            get_init_val();
            if(init_finish==true&&time_duration(1.0))
                calc_slip();
        }
        rate.sleep();
        ros::spinOnce();
    }
}
void IMU_visual::update_IMU(const sensor_msgs::Imu::ConstPtr &msg)
{
    IMU_=*msg;
    imu_callback=true;
}
void IMU_visual::get_init_val()
{
    if(init_finish==false)
    {
        if(init_th.size()<=30)
        {
            init_th.push_back(curr_th_diff);
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
void IMU_visual::update_ROBOT(const geometry_msgs::Pose::ConstPtr &msg)
{
    ROBOT_=*msg;
    robot_callback=true;
}
void IMU_visual::calc_slip()
{
    if(abs(curr_dp_th)>_slip_rate)
    {
        if(slip==false)
        {
            adjust_diff=prev_th_diff;
            slip=true;
        }
        ROS_WARN("SLIP OCCUR");
    }
    else
    {
        if(slip==true)
        {
            adjust_diff-=curr_th_diff;
            slip_occured=true;
            slip=false;
        }
    }
}
void IMU_visual::calc_acc()
{
    geometry_msgs::Pose temp_ROBOT_=ROBOT_;
    sensor_msgs::Imu temp_IMU_=IMU_;

    tf::Pose pose;
    geometry_msgs::Pose temp_pose;

    temp_pose.orientation.x=temp_ROBOT_.orientation.x-temp_IMU_.orientation.x;
    temp_pose.orientation.y=temp_ROBOT_.orientation.y-temp_IMU_.orientation.y;
    temp_pose.orientation.z=temp_ROBOT_.orientation.z-temp_IMU_.orientation.z;
    temp_pose.orientation.w=temp_ROBOT_.orientation.w-temp_IMU_.orientation.w;
    tf::poseMsgToTF(temp_pose, pose);
    curr_th_diff=tf::getYaw(pose.getRotation())-init_diff;
    curr_dp_th=(curr_th_diff-prev_th_diff);
    dpp_th=(curr_dp_th-prev_dp_th);

    // if(slip_occured==true&&init_finish==true)
    // {
    //     ROS_INFO("%f",adjust_diff);
    //     slip_occured=false;
    // }
    diff.angular.x=curr_th_diff;
    diff.angular.y=curr_dp_th;
    diff.angular.z=dpp_th;

    prev_th_diff=curr_th_diff;
    prev_dp_th=curr_dp_th;

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