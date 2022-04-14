#include "imu_visualize.hpp"

IMU_visual::IMU_visual()
{
    //ROS_TOPIC
    ros::NodeHandle _nh("~");
    _nh.param("Ros_rate", Ros_rate, 15);
    _nh.param("Slip_ddq", _slip_ddq, 0.3);
    _nh.param("Slip_ddq", _slip_dq, 0.3);
    _nh.param("notinit", _no_init, 1);
    _nh.param("Slip_Delay", Slip_Delay, 1);
    _nh.param("dQ_sum_size", dQ_sum_size, 4);
    _nh.param("Q_con_size", Q_con_size, 10);


    slip=false;
    slip_occured=false;
    init_finish=false;
    timer_start=false;
    adjust_occured=false;

    imu_callback=false;
    robot_callback=false;
    
    init_diff=0;
    adjust_diff=0;

    count=0;
    slip_count=0;
    
    index=0;
    Q_index=0;
    Slip_Delay_index=0;
    vector<double>empty_con1(Q_con_size);
    Q_con=empty_con1;
    dQ_con=empty_con1;
    ddQ_con=empty_con1;
    slip_timer=empty_con1;
    slip_yaw=empty_con1;
    vector<sensor_msgs::Imu> empty_con2(3);
    slip_d_val=empty_con2;

    memset(&diff_info_,0,sizeof(struct diff_info));
    // _pub_robot_adjust=_nh.advertise<geometry_msgs::Pose>("/robot_adjust_byslip",1);
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);

    //time sync
    imu_data_sub.subscribe(_nh,"/imu_read",1);
    robot_data_sub.subscribe(_nh,"/odom",1);
    sync_.reset(new Sync(MySyncPolicy(10),imu_data_sub,robot_data_sub));
    sync_->registerCallback(boost::bind(&IMU_visual::callback,this,_1,_2));    

    //Run function
    run_sequence();
}
IMU_visual::~IMU_visual()
{

}
void IMU_visual::run_sequence()
{   
    ros::Rate rate(Ros_rate);
    
    while(ros::ok())
    {
        if(imu_callback&&robot_callback)
        {   
            // init_prev();
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
                show_slip();
            }
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
        if(init_th.size()<=Ros_rate)
        {
            init_th.push_back(Q_con[Q_index]);
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
    if(slip==not_slip)
    {
        if(abs(ddQ_con[Q_index])>_slip_ddq)//?
        {
            // ROS_INFO("ddQ: %f",diff_info_.curr_ddQ_yaw);
            // ROS_INFO("cu_IMU : %f cu_ROB : %f ",diff_info_.curr_Imu_yaw,diff_info_.curr_Robot_yaw);
            // ROS_INFO("cu_m_IMU : %f cu_m_ROB : %f ",diff_info_.curr_modified_Imu_yaw,diff_info_.curr_modified_Robot_yaw);
            slip=ddQ_high;
            slip_timer[0]=return_current_time();
            slip_yaw[0]=Q_con[Q_index];
            slip_d_val[0]=diff_info_.Imu_val;
            index++;
        }
    }
    else if(slip==ddQ_high)
    {
        bool run=false;
        if(slip_count!=0)
            slip_count=count;
        if(count-slip_count>=dQ_sum_size/2)
        {
            double sum=0;
            for(int i=0;i<(dQ_sum_size);i++)
            {
                sum+=dQ_con[calc_index(Q_index,-i)]/15.0;
            }
            if(sum>_slip_dq)
            {
                slip=not_slip;
                slip_timer[2]=0;
            }
            slip_count=0;
        }
        if(ddQ_con[Q_index]==0)
        {
            run=true;
        }
        else if(ddQ_con[Q_index]>0&&ddQ_con[calc_index(Q_index,-1)]<0)
        {
            run=true;
        }
        else if(ddQ_con[Q_index]<0&&ddQ_con[calc_index(Q_index,-1)]>0)
        {
            run=true;
        }
        if(run)
        {
            slip_timer[index]=return_current_time();
            slip_yaw[index]=Q_con[Q_index];
            slip_d_val[index]=diff_info_.Imu_val;
            index++;
            run=false;
        }
        if(slip_timer[2]!=0)
        {
            index=0;
            slip_time=slip_timer[2]-slip_timer[0];
            diff_info_.adjust_yaw=slip_yaw[2]-slip_yaw[0];
            slip_occured=true;
        }
    }
    else if(slip==dQ_sum_high&&slip_occured==true)
    {
        diff_info_.slip_d_val.orientation.x=slip_d_val[2].orientation.x-slip_d_val[0].orientation.x;
        diff_info_.slip_d_val.orientation.y=slip_d_val[2].orientation.y-slip_d_val[0].orientation.y;
        diff_info_.slip_d_val.orientation.z=slip_d_val[2].orientation.z-slip_d_val[0].orientation.z;
        diff_info_.slip_d_val.orientation.w=slip_d_val[2].orientation.w-slip_d_val[0].orientation.w;

        diff_info_.adjust_d_val.orientation.x+=diff_info_.slip_d_val.orientation.x;
        diff_info_.adjust_d_val.orientation.y+=diff_info_.slip_d_val.orientation.y;
        diff_info_.adjust_d_val.orientation.z+=diff_info_.slip_d_val.orientation.z;
        diff_info_.adjust_d_val.orientation.w+=diff_info_.slip_d_val.orientation.w;

        tf::Pose pose;
        tf::poseMsgToTF(return_pose(diff_info_.adjust_d_val), pose);
        diff_info_.adjust_yaw=tf::getYaw(pose.getRotation());
        ROS_INFO("change yaw: %f",diff_info_.adjust_yaw);
        adjust_occured=true;
        slip_occured=false;
        slip=not_slip;
    }
}
void IMU_visual::calc_acc()
{
    //testing - diff_info_.Robot_val | Real - diff_info_.adj_Robot_val
    nav_msgs::Odometry temp_ROBOT_=diff_info_.Robot_val;
    sensor_msgs::Imu temp_IMU_=diff_info_.Imu_val;
    tf::Pose pose;
// -init_diff;


    tf::poseMsgToTF(return_pose(diff_info_.Robot_val), pose);
    diff_info_.curr_Robot_yaw=tf::getYaw(pose.getRotation());
    tf::poseMsgToTF(return_pose(diff_info_.Imu_val), pose);
    diff_info_.curr_Imu_yaw=tf::getYaw(pose.getRotation());
    //IMU 센서 값이 로봇보다 높을 시 시나리오 0 그 외에는 시나리오 1
    determine_overshoot();
    calc_modified_yaw();

    //로봇 보정 10->6.28
    diff_info_.curr_modified_Robot_yaw*=0.628;
    Q_con[Q_index]=diff_info_.curr_modified_Imu_yaw-diff_info_.curr_modified_Robot_yaw;

    // 두 각도의 차는 일정하지만 단순 회전만 하는데 dq값과 ddq값이 튀는 현상 관측
    // 일부러 슬립을 줄 경우 두 각도의 차가 변함 그리고 dq값과 ddq값도 튐 그러나 간혈적으로 슬립이 일어났다고 인식을 못하는 경우 관측
    
    dQ_con[Q_index]=(Q_con[Q_index]-Q_con[calc_index(Q_index,-1)])*Ros_rate;
    ddQ_con[Q_index]=(dQ_con[Q_index]-dQ_con[calc_index(Q_index,-1)])*Ros_rate;
    
    // ROS_INFO("DDQ: %f",diff_info_.curr_ddQ_yaw);
    // ROS_INFO("IMU : %f ROB : %f ",diff_info_.curr_Imu_yaw,diff_info_.curr_Robot_yaw);
    // ROS_INFO("adj : %f ini : %f ",diff_info_.adjust_yaw,init_diff);
    
    //양쪽 다 360도가 넘어갈 경우 처리과정?

    // if(adjust_occured)
    // {
    //     diff_info_.prev_Q_yaw=diff_info_.curr_Q_yaw;
    //     adjust_occured=false;
    // }
    //     if(diff_info_.curr_Q_yaw>Gap&&diff_info_.prev_Q_yaw<-Gap)
    //     {
    //         diff_info_.curr_Q_yaw=diff_info_.prev_Q_yaw;
    //     }
    //     else if(diff_info_.curr_Q_yaw<-Gap&&diff_info_.prev_Q_yaw>Gap)
    //     {
    //         diff_info_.curr_Q_yaw=diff_info_.prev_Q_yaw;
    //     }
    
    diff.angular.x=Q_con[calc_index(Q_index,-2)];
    diff.angular.y=dQ_con[calc_index(Q_index,-1)];
    diff.angular.z=ddQ_con[Q_index];
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
// void IMU_visual::adjust_slip(double x,double y, double z, double w)
// {
//     // diff_info_.curr_Q.orientation.x-=x;
//     // diff_info_.curr_Q.orientation.y-=y;
//     // diff_info_.curr_Q.orientation.z-=z;
//     // diff_info_.curr_Q.orientation.w-=w;
//     adjust_occured=true;

//     ROS_WARN("ADJUST_Needed");
//     fill(slip_timer.begin(),slip_timer.end(),0);
// }
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
    diff_info_.prev_modified_Imu_yaw=diff_info_.curr_modified_Imu_yaw;
    diff_info_.prev_modified_Robot_yaw=diff_info_.curr_modified_Robot_yaw;
    diff_info_.prev_Imu_yaw=diff_info_.curr_Imu_yaw;
    diff_info_.prev_Robot_yaw=diff_info_.curr_Robot_yaw;
    count++;
    Q_index++;
    if(Q_index>=Q_con_size)
        Q_index=0;
}
void IMU_visual::show_slip()//modify needed
{
    cv::namedWindow("Slip_Occur==RED");
    cv::moveWindow("Slip_Occur==RED",60,0);
    
    if(slip==ddQ_high)
    {
        cv::Mat Yellow_map(cv::Size(100,100),CV_8UC3,{200,0,200});
        cv::imshow("Slip_Occur==RED",Yellow_map);
    }
    else if(slip==dQ_sum_high)
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
geometry_msgs::Pose IMU_visual::return_pose(sensor_msgs::Imu &A)
{
    geometry_msgs::Pose temp;
    temp.orientation.x=0;
    temp.orientation.y=0;
    temp.orientation.z=A.orientation.z;
    temp.orientation.w=A.orientation.w;
    return temp;
}
geometry_msgs::Pose IMU_visual::return_pose(nav_msgs::Odometry &A)
{
    geometry_msgs::Pose temp;
    temp.orientation.x=0;
    temp.orientation.y=0;
    temp.orientation.z=A.pose.pose.orientation.z;
    temp.orientation.w=A.pose.pose.orientation.w;
    return temp;
}

geometry_msgs::Pose IMU_visual::return_plus_pose(nav_msgs::Odometry &A, sensor_msgs::Imu &B)
{
    geometry_msgs::Pose temp;

    temp.orientation.x=A.pose.pose.orientation.x+B.orientation.x;
    temp.orientation.y=A.pose.pose.orientation.y+B.orientation.y;
    temp.orientation.z=A.pose.pose.orientation.z+B.orientation.z;
    temp.orientation.w=A.pose.pose.orientation.w+B.orientation.w;

    return temp;
}
void IMU_visual::init_prev()
{
    if(first_on)
    {
        tf::Pose pose;
        tf::poseMsgToTF(return_pose(diff_info_.Robot_val), pose);
        diff_info_.curr_Robot_yaw=tf::getYaw(pose.getRotation());
        tf::poseMsgToTF(return_pose(diff_info_.Imu_val), pose);
        diff_info_.curr_Imu_yaw=tf::getYaw(pose.getRotation());
        diff_info_.prev_Robot_yaw=diff_info_.curr_Robot_yaw;
        diff_info_.prev_Imu_yaw=diff_info_.curr_Imu_yaw;
        first_on=false;
    }
}
void IMU_visual::calc_modified_yaw()
{
    diff_info_.curr_modified_Imu_yaw=diff_info_.overshoot_imu*PI*2+diff_info_.curr_Imu_yaw;
    diff_info_.curr_modified_Robot_yaw=diff_info_.overshoot_robot*PI*2+diff_info_.curr_Robot_yaw;
}
void IMU_visual::determine_overshoot()
{
    if(diff_info_.prev_Imu_yaw<-2.5 && diff_info_.curr_Imu_yaw>2.5)
    diff_info_.overshoot_imu--;
    else if(diff_info_.prev_Imu_yaw>2.5&&diff_info_.curr_Imu_yaw<-2.5)
    diff_info_.overshoot_imu++;

    if(diff_info_.prev_Robot_yaw<-2.5 && diff_info_.curr_Robot_yaw>2.5)
    diff_info_.overshoot_robot--;
    else if(diff_info_.prev_Robot_yaw>2.5&&diff_info_.curr_Robot_yaw<-2.5)
    diff_info_.overshoot_robot++;
}
void IMU_visual::callback(const sensor_msgs::ImuConstPtr &Imu,const nav_msgs::OdometryConstPtr &Robot)
{
    diff_info_.Robot_val=*Robot;
    diff_info_.Imu_val=*Imu;
    imu_callback=true;
    robot_callback=true;
    
    // ROS_INFO("z: %f w: %f",diff_info_.Robot_val.pose.pose.orientation.z,diff_info_.Robot_val.pose.pose.orientation.w);
    // ROS_INFO("z: %f w: %f",diff_info_.Imu_val.orientation.z,diff_info_.Imu_val.orientation.w);      
}
int IMU_visual::calc_index(int index,int num)
{
    if(index+num>=Q_con_size)
    {
        return (index+num)-Q_con_size;
    }
    else if(index+num<0)
    {
        return Q_con_size+(index+num);
    }
    else
    {
        return 0;
    }
}