#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
using namespace std;

struct Map
{
    int row;
    int col;
    int cross;
    int block_row;
    int block_col;
};

struct IMU_val
{
    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;
};

struct ROBOT_val
{
    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;
};

struct Color
{
    unsigned char B;
    unsigned char G;
    unsigned char R;
};

class IMU_visual
{
    private:
        ros::NodeHandle _nh;
        ros::Publisher _pub_differ;
        ros::Publisher _pub_robot_adjust;
        ros::Subscriber _sub_imu_data;
        ros::Subscriber _sub_robot_data;
        ros::Subscriber _sub_robot_adjust;
        
        geometry_msgs::Twist diff;
        geometry_msgs::Pose ROBOT_;
        sensor_msgs::Imu IMU_;
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
        
        double curr_th_diff;
        double prev_th_diff;
       
        double curr_dp_th;
        double prev_dp_th;

        double curr_dpp_th;
        double prev_dpp_th;
        
        ros::Time start_time;

        vector<double> slip_timer;
        vector<IMU_val> slip_d_val;
        IMU_val slip_d;
        int index;
        double slip_time;
        bool slip_time_measured;
        vector<double> init_th;

        // struct Color word;
        // struct Color Robot_Color_;
        // struct Color IMU_Color_;

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
    public:
        IMU_visual();
        ~IMU_visual();
};
