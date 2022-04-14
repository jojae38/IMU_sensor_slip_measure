#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace std;
using namespace message_filters;
const double PI = 3.141592;
const double Gap = 3.1;
enum SlIP_pos {not_slip, ddQ_high ,dQ_sum_high};
struct diff_info
{
    nav_msgs::Odometry Robot_val;
    sensor_msgs::Imu Imu_val;

    double prev_Robot_yaw;
    double curr_Robot_yaw;
    double prev_Imu_yaw;
    double curr_Imu_yaw;

    int overshoot_robot;
    int overshoot_imu;

    double prev_modified_Robot_yaw;
    double curr_modified_Robot_yaw;

    double prev_modified_Imu_yaw;
    double curr_modified_Imu_yaw;

    double adjust_yaw;
    sensor_msgs::Imu adjust_d_val;
    sensor_msgs::Imu slip_d_val;
};

class IMU_visual
{   public:
        IMU_visual();
        ~IMU_visual();
        void callback(const sensor_msgs::ImuConstPtr &Imu,const nav_msgs::OdometryConstPtr &Robot);
    private:
        // ros
        ros::NodeHandle _nh;
        ros::Publisher _pub_differ;
        geometry_msgs::Twist diff;

        //time sync
        message_filters::Subscriber<sensor_msgs::Imu> imu_data_sub;
        message_filters::Subscriber<nav_msgs::Odometry> robot_data_sub;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        //init
        // SlIP_pos SlIP_pos_;
        diff_info diff_info_;
        bool imu_callback;
        bool robot_callback;
        bool init_finish;
        bool timer_start;

        int slip;
        bool slip_occured;
        bool adjust_occured;
        
        bool first_on;
        //ros param
        double _slip_dq;
        double _slip_ddq;
        int Ros_rate;
        int _no_init;
        int Slip_Delay;
        int Q_con_size;
        int dQ_sum_size;

        int Slip_Delay_index;
        
        double init_diff;
        double adjust_diff;
        
        ros::Time start_time;
        
        int index;
        double slip_time;
        bool slip_time_measured;
        int count;
        int slip_count;
        vector<double> init_th;
        int Q_index;
        vector<double> Q_con;
        vector<double> dQ_con;
        vector<double> ddQ_con;

        vector<double> slip_timer;
        vector<double> slip_yaw;
        vector<sensor_msgs::Imu> slip_d_val;

        //function
        void update_IMU(const sensor_msgs::Imu::ConstPtr &msg);
        void update_ROBOT(const nav_msgs::Odometry::ConstPtr &msg);
        void adjust_ROBOT(const geometry_msgs::Pose::ConstPtr &msg);
        void calc_slip_time();
        void calc_acc();
        bool time_duration(double sec);
        void get_init_val();
        void run_sequence();
        void adjust_slip(double x,double y, double z, double w);
        void show_slip();
        void init_prev();
        void determine_overshoot();
        void calc_modified_yaw();
        double return_current_time();
        double turn_Quaternion_to_yaw(geometry_msgs::Pose &x,geometry_msgs::Pose &y);
        geometry_msgs::Pose return_sub_pose(geometry_msgs::Pose &A, geometry_msgs::Pose &B);
        geometry_msgs::Pose return_sub_pose(nav_msgs::Odometry &A, sensor_msgs::Imu &B);
        geometry_msgs::Pose return_plus_pose(nav_msgs::Odometry &A, sensor_msgs::Imu &B);

        geometry_msgs::Pose return_pose(sensor_msgs::Imu &A);
        geometry_msgs::Pose return_pose(nav_msgs::Odometry &A);
        void turn_curr_to_prev();
        int calc_index(int index,int num);
};
