#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
using namespace std;

struct diff_info
{
    geometry_msgs::Pose Robot_val;
    sensor_msgs::Imu Imu_val;

    geometry_msgs::Pose prev_Q;
    geometry_msgs::Pose curr_Q;
    double curr_Q_yaw;
    double prev_Q_yaw;

    double curr_dQ_yaw;
    double prev_dQ_yaw;

    double curr_ddQ_yaw;
    double prev_ddQ_yaw;
};

class IMU_visual
{
    private:
        // ros
        ros::NodeHandle _nh;
        ros::Publisher _pub_differ;
        ros::Publisher _pub_robot_adjust;
        ros::Subscriber _sub_imu_data;
        ros::Subscriber _sub_robot_data;
        ros::Subscriber _sub_robot_adjust;
        
        geometry_msgs::Twist diff;

        diff_info diff_info_;
        //init
        bool imu_callback;
        bool robot_callback;
        bool init_finish;
        bool timer_start;

        bool slip;
        bool slip_occured;
        
        double _slip_rate;
        int _no_init;

        double init_diff;
        double adjust_diff;
        
        ros::Time start_time;

        vector<double> slip_timer;
        vector<sensor_msgs::Imu> slip_d_val;
        geometry_msgs::Pose slip_d;
        int index;
        double slip_time;
        bool slip_time_measured;
        vector<double> init_th;

        void update_IMU(const sensor_msgs::Imu::ConstPtr &msg);
        void update_ROBOT(const geometry_msgs::Pose::ConstPtr &msg);
        void adjust_ROBOT(const geometry_msgs::Pose::ConstPtr &msg);
        void calc_slip_time();
        void calc_acc();
        bool time_duration(double sec);
        void get_init_val();
        void run_sequence();
        void publish_adjust(double x,double y, double z, double w);
        double return_current_time();
        double turn_Quaternion_to_yaw(geometry_msgs::Pose &x,geometry_msgs::Pose &y);
        geometry_msgs::Pose return_sub_pose(geometry_msgs::Pose &A, geometry_msgs::Pose &B);
        geometry_msgs::Pose return_sub_pose(geometry_msgs::Pose &A, sensor_msgs::Imu &B);
        void turn_curr_to_prev();
    public:
        IMU_visual();
        ~IMU_visual();
};
