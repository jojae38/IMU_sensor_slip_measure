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

    imu_callback=false;
    robot_callback=true;
    
    init_diff=0;
    adjust_diff=0;

    curr_dp_th=0;
    prev_dp_th=0;

    curr_dpp_th=0;
    prev_dpp_th=0;

    vector<double> temp(3);
    vector<IMU_val> temp_val(3);
    slip_timer=temp;
    slip_d_val=temp_val;
    index=0;

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

    _pub_robot_adjust=_nh.advertise<geometry_msgs::Pose>("/robot_adjust_byslip",1);
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);
    _sub_imu_data=_nh.subscribe("/imu_read",1, &IMU_visual::update_IMU, this);
    _sub_robot_data=_nh.subscribe("/robot_read",1,&IMU_visual::update_ROBOT,this);
    //test purpose//
    _sub_robot_adjust=_nh.subscribe("/robot_adjust_byslip",1,&IMU_visual::adjust_ROBOT,this);

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
void IMU_visual::adjust_ROBOT(const geometry_msgs::Pose::ConstPtr &msg)
{
    ROBOT_.orientation.x+=msg->orientation.x;
    ROBOT_.orientation.y+=msg->orientation.y;
    ROBOT_.orientation.z+=msg->orientation.z;
    ROBOT_.orientation.w+=msg->orientation.w;
}
void IMU_visual::calc_slip_time()
{
    if(slip==false)
    {
        if(abs(curr_dpp_th)>_slip_rate)
        {
            slip=true;
            slip_timer[0]=return_current_time();
            slip_d_val[0].ori_x=IMU_.orientation.x;
            slip_d_val[0].ori_y=IMU_.orientation.y;
            slip_d_val[0].ori_z=IMU_.orientation.z;
            slip_d_val[0].ori_w=IMU_.orientation.w;
            index++;
        }
    }
    else
    {
        bool run=false;
        if(curr_dpp_th==0)
        {
            run=true;
        }
        else if(curr_dpp_th>0&&prev_dpp_th<0)
        {
            run=true;
        }
        else if(curr_dpp_th<0&&prev_dpp_th>0)
        {
            run=true;
        }
        if(run)
        {
            slip_timer[index]=return_current_time();
            slip_d_val[index].ori_x=IMU_.orientation.x;
            slip_d_val[index].ori_y=IMU_.orientation.y;
            slip_d_val[index].ori_z=IMU_.orientation.z;
            slip_d_val[index].ori_w=IMU_.orientation.w;
            index++;
            run=false;
        }
        if(slip_timer[2]!=0)
        {
            slip=false;
            index=0;
            slip_time=slip_timer[2]-slip_timer[0];

            slip_d.ori_x=slip_d_val[2].ori_x-slip_d_val[0].ori_x;
            slip_d.ori_y=slip_d_val[2].ori_y-slip_d_val[0].ori_y;
            slip_d.ori_z=slip_d_val[2].ori_z-slip_d_val[0].ori_z;
            slip_d.ori_w=slip_d_val[2].ori_w-slip_d_val[0].ori_w;
            publish_adjust(slip_d.ori_x,slip_d.ori_y,slip_d.ori_z,slip_d.ori_w);
            ROS_INFO("change x: %f y: %f z: %f w: %f",slip_d.ori_x,slip_d.ori_y,slip_d.ori_z,slip_d.ori_w);
        }
    }

    // if(abs(curr_dp_th)>_slip_rate)
    // {
    //     if(slip==false)
    //     {
    //         adjust_diff=prev_th_diff;
    //         slip=true;
    //     }
    //     ROS_WARN("SLIP OCCUR");
    // }
    // else
    // {
    //     if(slip==true)
    //     {
    //         adjust_diff-=curr_th_diff;
    //         slip_occured=true;
    //         slip=false;
    //     }
    // }
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
    curr_dp_th=(curr_th_diff-prev_th_diff)*10;
    prev_dpp_th=curr_dpp_th;
    curr_dpp_th=(curr_dp_th-prev_dp_th);

    
    // if(slip_occured==true&&init_finish==true)
    // {
    //     ROS_INFO("%f",adjust_diff);
    //     slip_occured=false;
    // }

    diff.angular.x=curr_th_diff;
    diff.angular.y=curr_dp_th;
    diff.angular.z=curr_dpp_th;

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
void IMU_visual::publish_adjust(double x,double y, double z, double w)
{
    geometry_msgs::Pose temp;
    temp.orientation.x=x;
    temp.orientation.y=y;
    temp.orientation.z=z;
    temp.orientation.w=w;
    _pub_robot_adjust.publish(temp);
    ROS_WARN("ADJUST_Needed");
    fill(slip_timer.begin(),slip_timer.end(),0);
    
}
double IMU_visual::return_current_time()
{
    return ros::Time::now().toSec();
}