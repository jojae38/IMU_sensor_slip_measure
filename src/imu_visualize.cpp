#include "imu_visualize.hpp"

IMU_visual::IMU_visual()
{
    ros::NodeHandle _nh("~");
    _nh.param("Ros_rate", Ros_rate, 15);
    _nh.param("Slip_ddq", _slip_rate, 0.3);
    _nh.param("notinit", _no_init, 1);
    _nh.param("Slip_Delay", Slip_Delay, 1);
    _nh.param("Slip_dq_over", Slip_dq_over, 0.5);

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
    vector<double> empty_con1(3);
    slip_timer=empty_con1;
    slip_yaw=empty_con1;
    index=0;
    Slip_Delay_index=0;
    vector<sensor_msgs::Imu> empty_con2(3);
    slip_d_val=empty_con2;

    vector<double> empty_con3(Slip_Delay+1);
    dQ_yaw_container=empty_con3;
    memset(&diff_info_,0,sizeof(struct diff_info));
    // _pub_robot_adjust=_nh.advertise<geometry_msgs::Pose>("/robot_adjust_byslip",1);
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);
    // _sub_imu_data=_nh.subscribe("/imu_read",1, &IMU_visual::update_IMU, this);
    // _sub_robot_data=_nh.subscribe("/odom",1,&IMU_visual::update_ROBOT,this);

    //time sync
    imu_data_sub.subscribe(_nh,"/imu_read",1);
    robot_data_sub.subscribe(_nh,"/odom",1);
    // robot_data_sub
    
    // message_filters::Subscriber<sensor_msgs::Imu>imu_data_sub(_nh,"/imu_read",1);
    // message_filters::Subscriber<nav_msgs::Odometry>robot_data_sub(_nh,"/odom",1);
    sync_.reset(new Sync(MySyncPolicy(10),imu_data_sub,robot_data_sub));
    sync_->registerCallback(boost::bind(&IMU_visual::callback,this,_1,_2));    
    //test purpose//
    // _sub_robot_adjust=_nh.subscribe("/robot_adjust_byslip",5,&IMU_visual::adjust_ROBOT,this);
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
        {   init_prev();
            calc_acc();
            if(_no_init)
            {
                    get_init_val();
                if(init_finish==true&&time_duration(1.0))
                    calc_slip_time();
            }
            else
            {
                calc_slip_time_2();   
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
        if(init_th.size()<=Ros_rate)
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
            ROS_INFO("ddQ: %f",diff_info_.curr_ddQ_yaw);
            // ROS_INFO("cu_IMU : %f cu_ROB : %f ",diff_info_.curr_Imu_yaw,diff_info_.curr_Robot_yaw);
            // ROS_INFO("cu_m_IMU : %f cu_m_ROB : %f ",diff_info_.curr_modified_Imu_yaw,diff_info_.curr_modified_Robot_yaw);
            slip=true;
            slip_timer[0]=return_current_time();
            slip_yaw[0]=diff_info_.curr_Q_yaw;
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
            slip_yaw[index]=diff_info_.curr_Q_yaw;
            slip_d_val[index]=diff_info_.Imu_val;
            index++;
            run=false;
        }
        if(slip_timer[2]!=0)
        {
            slip=false;
            index=0;
            slip_time=slip_timer[2]-slip_timer[0];
            diff_info_.adjust_yaw=slip_yaw[2]-slip_yaw[0];

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
            // ROS_INFO("cu_IMU : %f cu_ROB : %f ",diff_info_.curr_Imu_yaw,diff_info_.curr_Robot_yaw);
            // ROS_INFO("cu_m_IMU : %f cu_m_ROB : %f ",diff_info_.curr_modified_Imu_yaw,diff_info_.curr_modified_Robot_yaw);
            // adjust_slip(Ori.x,Ori.y,Ori.z,Ori.w);
            ROS_INFO("change yaw: %f",diff_info_.adjust_yaw);
        }
    }
}
void IMU_visual::calc_slip_time_2()
{
    if(slip==false)
    {
        if(abs(diff_info_.curr_dQ_yaw)>_slip_rate)//?
        {
            slip=true;
            slip_timer[0]=return_current_time();
            slip_yaw[0]=diff_info_.curr_Q_yaw;
            slip_d_val[0]=diff_info_.Imu_val;
            slip_contain_.dQ_count=2;
            slip_contain_.dQ_sum+=(diff_info_.prev_dQ_yaw+diff_info_.curr_dQ_yaw)/float(Slip_Delay);
            index++;
        }
    }
    else
    {   
        if(slip_contain_.dQ_count>=4)
        {
            if(abs(slip_contain_.dQ_sum)>Slip_dq_over)
            {
                slip_occured=true;
            }
            else
            {
                slip=false;
                slip_contain_.dQ_sum=0;
                slip_contain_.dQ_count=0;
            }
            // index++;
            
            // slip_yaw[index]=diff_info_.curr_Q_yaw;
            // slip_d_val[index]=diff_info_.Imu_val;
        }
        else
        {
            slip_contain_.dQ_count++;
            slip_contain_.dQ_sum+=diff_info_.curr_dQ_yaw/float(Slip_Delay);
        }
        
        if(slip_contain_.dQ_sum>0&&slip==true)
        {
            if(diff_info_.prev_dQ_yaw>0&&diff_info_.curr_dQ_yaw<=0)
            {
                slip_contain_.end_time=return_current_time();
                slip_contain_.end_Q_yaw=diff_info_.curr_Q_yaw;
            }
        }
        else if(slip_contain_.dQ_sum<0&&slip==true)
        {
            if(diff_info_.prev_dQ_yaw<0&&diff_info_.curr_dQ_yaw>=0)
            {
                slip_contain_.end_time=return_current_time();
                slip_contain_.end_Q_yaw=diff_info_.curr_Q_yaw;
            }
        }
        else if(diff_info_.curr_dQ_yaw==0&&slip==true)
        {
            slip_contain_.end_time=return_current_time();
            slip_contain_.end_Q_yaw=diff_info_.curr_Q_yaw;
        }
    }
    if(slip_occured==true&&slip_contain_.end_time!=0)
    {   
        // slip_time=slip_timer[2]-slip_timer[0];
        diff_info_.adjust_yaw=slip_contain_.end_Q_yaw-slip_contain_.start_Q_yaw;

        diff_info_.slip_d_val.orientation.x=slip_d_val[2].orientation.x-slip_d_val[0].orientation.x;
        diff_info_.slip_d_val.orientation.y=slip_d_val[2].orientation.y-slip_d_val[0].orientation.y;
        diff_info_.slip_d_val.orientation.z=slip_d_val[2].orientation.z-slip_d_val[0].orientation.z;
        diff_info_.slip_d_val.orientation.w=slip_d_val[2].orientation.w-slip_d_val[0].orientation.w;

        diff_info_.adjust_d_val.orientation.x+=diff_info_.slip_d_val.orientation.x;
        diff_info_.adjust_d_val.orientation.y+=diff_info_.slip_d_val.orientation.y;
        diff_info_.adjust_d_val.orientation.z+=diff_info_.slip_d_val.orientation.z;
        diff_info_.adjust_d_val.orientation.w+=diff_info_.slip_d_val.orientation.w;
        
        index=0;
        slip=false;
        slip_occured=false;
        slip_contain_.end_time=0;
        tf::Pose pose;
        tf::poseMsgToTF(return_pose(diff_info_.adjust_d_val), pose);
        diff_info_.adjust_yaw=tf::getYaw(pose.getRotation());
        ROS_INFO("change yaw: %f",diff_info_.adjust_yaw);
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
    diff_info_.curr_Q_yaw=diff_info_.curr_modified_Imu_yaw-diff_info_.curr_modified_Robot_yaw;

    //slip delay active
    // Q_yaw_container[Slip_Delay_index]=diff_info_.curr_Q_yaw;
    // Slip_Delay_index++;

    // if(Slip_Delay_index>=Slip_Delay)
    // {
    //     diff_info_.curr_dQ_yaw=0;
    //     for(int i=0;i<Slip_Delay;i++)
    //     {
    //         diff_info_.curr_dQ_yaw+=Q_yaw_container[i]/float(Slip_Delay);
    //     }
    //     // diff_info_.curr_dQ_yaw=(Q_yaw_container[Slip_Delay-1]-Q_yaw_container[0])*10;
    //     diff_info_.curr_ddQ_yaw=diff_info_.curr_dQ_yaw-diff_info_.prev_dQ_yaw;
    //     Slip_Delay_index=0;
    // }
    //slip delay not active

    // 두 각도의 차는 일정하지만 단순 회전만 하는데 dq값과 ddq값이 튀는 현상 관측
    // 일부러 슬립을 줄 경우 두 각도의 차가 변함 그리고 dq값과 ddq값도 튐 그러나 간혈적으로 슬립이 일어났다고 인식을 못하는 경우 관측
    diff_info_.curr_dQ_yaw=(diff_info_.curr_Q_yaw-diff_info_.prev_Q_yaw)*Ros_rate;
    diff_info_.curr_ddQ_yaw=(diff_info_.curr_dQ_yaw-diff_info_.prev_dQ_yaw)*Ros_rate;

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
void IMU_visual::adjust_slip(double x,double y, double z, double w)
{
    // diff_info_.curr_Q.orientation.x-=x;
    // diff_info_.curr_Q.orientation.y-=y;
    // diff_info_.curr_Q.orientation.z-=z;
    // diff_info_.curr_Q.orientation.w-=w;
    adjust_occured=true;

    ROS_WARN("ADJUST_Needed");
    fill(slip_timer.begin(),slip_timer.end(),0);
}
double IMU_visual::return_current_time()
{
    return ros::Time::now().toSec();
}
void IMU_visual::turn_curr_to_prev()
{
    diff_info_.prev_Q_yaw=diff_info_.curr_Q_yaw;
    diff_info_.prev_dQ_yaw=diff_info_.curr_dQ_yaw;
    diff_info_.prev_ddQ_yaw=diff_info_.curr_ddQ_yaw;
    diff_info_.prev_modified_Imu_yaw=diff_info_.curr_modified_Imu_yaw;
    diff_info_.prev_modified_Robot_yaw=diff_info_.curr_modified_Robot_yaw;
    diff_info_.prev_Imu_yaw=diff_info_.curr_Imu_yaw;
    diff_info_.prev_Robot_yaw=diff_info_.curr_Robot_yaw;
}
void IMU_visual::show_slip()//modify needed
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
        diff_info_.curr_dQ_yaw=0;
        diff_info_.curr_ddQ_yaw=0;
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
    count++;
}