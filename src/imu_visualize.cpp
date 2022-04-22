#include "imu_visualize.hpp"

IMU_visual::IMU_visual()
{
    memset(&diff_info_,0,sizeof(struct diff_info));
    memset(&rqt_plot_,0,sizeof(struct rqt_plot));
    ros::NodeHandle _nh("~");
    _nh.param("Ros_rate", Ros_rate, 15);
    _nh.param("Slip_ddq", _slip_rate, 0.3);
    _nh.param("notinit", _no_init, 1);
    _nh.param("do_visualize", _do_visualize, 1);
    _nh.param("visual_length", _visual_length, 20);
    _nh.param("Slip_Delay", Slip_Delay, 1);
    _nh.param("Slip_dq_over", Slip_dq_over, 0.5);

    _nh.param("ADC_y", rqt_plot_.gird_block_y, 50);
    _nh.param("ADC_x", rqt_plot_.grid_block_x, 100);
    _nh.param("size_y", rqt_plot_.y_size, 10);
    _nh.param("size_x", rqt_plot_.x_size, 10);
    _nh.param("pixel_y", rqt_plot_.pixel_y_size, 600);
    _nh.param("pixel_x", rqt_plot_.pixel_x_size, 1100);

    string imu_temp="/imu_read";
    string robot_temp="/odom";
    string pub_temp="/pub_slip_info";
    _nh.param("subscribe_imu", imu_topic_name, imu_temp);
    _nh.param("subscribe_robot", robot_topic_name, robot_temp);
    _nh.param("publish_slip_topic_name", publish_slip_topic_name, pub_temp);

    slip=false;
    slip_occured=false;

    slip_start=false;
    slip_end=false;
    init_finish=false;
    timer_start=false;

    adjust_occured=false;

    imu_callback=true;
    robot_callback=true;
    
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

    prev_color=0;//0 = green;
    curr_color=0;//0 = green;
    
    vector<cv::Point> empty_con3(Ros_rate*10,{0,rqt_plot_.pixel_y_size/2});
    Q_line={Ros_rate*10,0,empty_con3};
    dQ_line={Ros_rate*10,0,empty_con3};
    ddQ_line={Ros_rate*10,0,empty_con3};
    Robot_line={Ros_rate*10,0,empty_con3};
    Imu_line={Ros_rate*10,0,empty_con3};

    _publish_slip=_nh.advertise<std_msgs::String>(publish_slip_topic_name,1);
    _pub_differ=_nh.advertise<geometry_msgs::Twist>("/acc_diff",1);
    time_pos=0;
    //time sync
    imu_data_sub.subscribe(_nh,imu_topic_name,1);
    robot_data_sub.subscribe(_nh,robot_topic_name,1);

    vector<int> empty_con5(Ros_rate*10);
    time = empty_con5;

    filter_size=3;
    filter_array={0.25,0.25,0.5};
    Robot_val_array={1,1,1};

    state_val="N";
    slip_Isth_val=0.0;
    slip_Ieth_val=0.0;
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
            if(_do_visualize==1)
            {
                rqt_plot_demo();
            }
            determine_collapse();
            publish_slip_value();
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
        slip_start=false;
        slip_end=false;
    }
    else
    {   
        if(slip_contain_.dQ_count>=4)
        {
            if(abs(slip_contain_.dQ_sum)>Slip_dq_over)
            {
                slip_occured=true;
                slip_start=true;
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
        
        slip_end=true;
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

    Robot_val_array.push_back(diff_info_.curr_Robot_yaw);
    Robot_val_array.erase(Robot_val_array.begin());
    double temp=0;
    if(abs(Robot_val_array[2]-Robot_val_array[1])>5||abs(Robot_val_array[2]-Robot_val_array[0])>5)
    {
        Robot_val_array[0]=Robot_val_array[2];
        Robot_val_array[1]=Robot_val_array[2];
    }
    for(int i=0;i<filter_size;i++)
    {
        temp+=Robot_val_array[i]*filter_array[i];
    }
    diff_info_.curr_Robot_yaw=temp;

    //-3.14에서 3.14로 변하거나 3.14에서 -3.14로 변할 때 값 보정
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
    prev_color=curr_color;
}
void IMU_visual::show_slip()//modify needed
{
    cv::namedWindow("Slip_Occur==RED");
    cv::moveWindow("Slip_Occur==RED",60,0);
    if(slip)
    {
        cv::Mat Red_map(cv::Size(100,100),CV_8UC3,{0,0,200});
        cv::imshow("Slip_Occur==RED",Red_map);
        curr_color=1;
    }
    else
    {
        cv::Mat Green_map(cv::Size(100,100),CV_8UC3,{0,200,0});
        cv::imshow("Slip_Occur==RED",Green_map);
        curr_color=0;
    }
    
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

        Robot_val_array={diff_info_.curr_Robot_yaw,diff_info_.curr_Robot_yaw,diff_info_.curr_Robot_yaw};
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
void IMU_visual::rqt_plot_demo()
{
    cv::namedWindow("rqt_plot_demo");
    cv::moveWindow("rqt_plot_demo",170,0);
    rqt_plot_.graph=cv::Mat::zeros(rqt_plot_.pixel_y_size,rqt_plot_.pixel_x_size,CV_8UC3);
    
    cv::line(rqt_plot_.graph,cv::Point(50,50),cv::Point(50,rqt_plot_.pixel_y_size-50),cv::Scalar(255,255,255),2,cv::LINE_AA,0);
    cv::line(rqt_plot_.graph,cv::Point(rqt_plot_.pixel_x_size-50,50),cv::Point(rqt_plot_.pixel_x_size-50,rqt_plot_.pixel_y_size-50),cv::Scalar(255,255,255),2,cv::LINE_AA,0);
    cv::line(rqt_plot_.graph,cv::Point(50,50),cv::Point(rqt_plot_.pixel_x_size-50,50),cv::Scalar(255,255,255),2,cv::LINE_AA,0);
    cv::line(rqt_plot_.graph,cv::Point(50,rqt_plot_.pixel_y_size-50),cv::Point(rqt_plot_.pixel_x_size-50,rqt_plot_.pixel_y_size-50),cv::Scalar(255,255,255),2,cv::LINE_AA,0);
    cv::line(rqt_plot_.graph,cv::Point(50,rqt_plot_.pixel_y_size/2),cv::Point(rqt_plot_.pixel_x_size-50,rqt_plot_.pixel_y_size/2),cv::Scalar(255,255,255),1,cv::LINE_AA,0);
    rqt_plot_update_line(Q_line,diff_info_.curr_Q_yaw);
    rqt_plot_update_line(dQ_line,diff_info_.curr_dQ_yaw);
    rqt_plot_update_line(ddQ_line,diff_info_.curr_ddQ_yaw);
    rqt_plot_update_line(Robot_line,diff_info_.curr_Robot_yaw);
    rqt_plot_update_line(Imu_line,diff_info_.curr_Imu_yaw);
    rqt_plot_line(rqt_plot_.graph,Q_line,255,255,20);
    rqt_plot_line(rqt_plot_.graph,dQ_line,50,200,50);
    rqt_plot_line(rqt_plot_.graph,ddQ_line,50,50,200);
    rqt_plot_line(rqt_plot_.graph,Robot_line,0,255,255);
    rqt_plot_line(rqt_plot_.graph,Imu_line,238,104,123);
    rqt_plot_val_text(rqt_plot_.graph);
    rqt_plot_simple_time_line(rqt_plot_.graph);

    cv::imshow("rqt_plot_demo",rqt_plot_.graph);
    cv::waitKey(10)==27;
}
void IMU_visual::rqt_plot_line(cv::Mat &graph, line &line_,uchar B,uchar G,uchar R)
{
    cv::polylines(graph,line_.dst,false,cv::Scalar(B,G,R),1,cv::LINE_AA,0);
}
void IMU_visual::rqt_plot_update_line(line &line_,double val)
{
    for(int i=0;i<line_.data_max;i++)
    {
        line_.dst[i].x-=Ros_rate;
    }
    cv::Point temp(rqt_plot_.pixel_x_size-50,rqt_plot_.pixel_y_size/2-val*rqt_plot_.gird_block_y);
    line_.dst.push_back(temp);
    line_.dst.erase(line_.dst.begin());
}
void IMU_visual::rqt_plot_val_text(cv::Mat &graph)
{
    cv::putText(graph,"0",cv::Point(30,rqt_plot_.pixel_y_size/2+5),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"2",cv::Point(30,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(-1)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"4",cv::Point(30,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(-2)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"6",cv::Point(30,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(-3)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"8",cv::Point(30,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(-4)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"10",cv::Point(20,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(-5)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"-2",cv::Point(17,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(1)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"-4",cv::Point(17,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(2)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"-6",cv::Point(17,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(3)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"-8",cv::Point(17,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(4)),1,1,cv::Scalar(150,150,150),1,8,false);
    cv::putText(graph,"-10",cv::Point(12,rqt_plot_.pixel_y_size/2+5+rqt_plot_.gird_block_y*(5)),1,1,cv::Scalar(150,150,150),1,8,false);
}

void IMU_visual::rqt_plot_simple_time_line(cv::Mat &graph)
{
    if(curr_color==1&&prev_color==0)
    {
        time.push_back(0);
        time[98]=1;
        state_val="S";
        slip_Isth_val=diff_info_.prev_Imu_yaw;
    }
    else if(curr_color==0&&prev_color==1)
    {
        time.push_back(2);
        state_val="N";
        slip_Ieth_val=diff_info_.curr_Imu_yaw;
    }
    else
    {
        time.push_back(0);
    }
    for(int i=0;i<100;i++)
    {
        if(time[i]==1)
        {
            cv::line(graph,cv::Point(i*10+50,150),cv::Point(i*10+50,rqt_plot_.pixel_y_size-150),cv::Scalar(0,200,200),2,cv::LINE_AA,0);
        }
        else if(time[i]==2)
        {
            cv::line(graph,cv::Point(i*10+50,150),cv::Point(i*10+50,rqt_plot_.pixel_y_size-150),cv::Scalar(200,0,200),2,cv::LINE_AA,0);
        }
    }
    time.erase(time.begin());
}

void IMU_visual::publish_slip_value()
{
    vector<string> state={state_val,to_string(diff_info_.curr_Imu_yaw),to_string(diff_info_.curr_Robot_yaw),to_string(slip_Isth_val),to_string(slip_Ieth_val)};
    string data="["+ state[0] +"], Ith: ["+ state[1] +"], Wth: ["+ state[2] +"], Isth: ["+ state[3] +"], Ieth: ["+ state[4] +"]";
    std_msgs::String temp;
    temp.data=data;
    cout <<state_val<<endl;
    _publish_slip.publish(temp);    
}
void IMU_visual::determine_collapse()
{
    #define a diff_info_.Imu_val.orientation
    
    //roll pitch 가 일정 넘어설때 F
    if(abs(a.x)>0.5||abs(a.y)>0.5)
    {
        state_val="F";
    }
    else if(state_val!="S")
    {
        state_val="N";
    }
}