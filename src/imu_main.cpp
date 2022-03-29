#include "imu_visualize.hpp"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Imu_visualize");
    IMU_visual IMU_visual_;
    ros::spin();
    return 0;
}

//not using//

// void IMU_visual::visualize()
// {
//     ROS_INFO("start");
//     cv::Mat map(cv::Size(Map_.row,Map_.col),CV_8UC3,{0,0,0});
//     //  cv::Mat map(cv::Size(100,100),CV_8UC3,{0,0,0});
//     for(int i=0;i<Map_.row;i++)
//     {
//         map.at<cv::Vec3b>(Map_.col/2-1,i)={255,255,255};
//         map.at<cv::Vec3b>(i,Map_.row/2-1)={255,255,255};
//     }

//     draw_th(Robot_Color_,ROBOT_val_[index_robot].ROBOT_th,map);
//     draw_th(IMU_Color_,IMU_val_[index_imu].IMU_th,map);


//     int gap=20;
//     string s_1="ROBOT_th ["+to_string(ROBOT_val_[index_robot].ROBOT_th)+"]";
//     string s_2="IMU_th ["+to_string(IMU_val_[index_imu].IMU_th)+"]";
//     string s_3="d_th ["+to_string(IMU_val_[index_imu].IMU_th-ROBOT_val_[index_robot].ROBOT_th)+"]";
//     // string s_4="dp_th ["+to_string()+"]";
//     // string s_5="dpp_th ["+to_string()+"]";
//     put_text(map,s_1,cv::Point(10,30),4,0.6,cv::Scalar(word.B,word.G,word.R));
//     put_text(map,s_2,cv::Point(10,30+gap*1),4,0.6,cv::Scalar(word.B,word.G,word.R));
//     put_text(map,s_3,cv::Point(10,30+gap*2),4,0.6,cv::Scalar(word.B,word.G,word.R));
//     // put_text(map,s_4,cv::Point(10,30+gap*3),4,0.6,cv::Scalar(word.B,word.G,word.R));
//     // put_text(map,s_5,cv::Point(10,30+gap*4),4,0.6,cv::Scalar(word.B,word.G,word.R));


//     cv::namedWindow("map");
//     cv::moveWindow("map",20,0);
//     cv::imshow("map",map);
//     ROS_INFO("finish");
//     cv::waitKey(0)==3;
    
    
// }

// void IMU_visual::draw_th(Color &color,double th,cv::Mat &map)
// {
//     for(int i=10;i<250;i++)
//     {
//         for(int j=-3;j<=3;j++)
//         {
//             map.at<cv::Vec3b>(map.rows/2+i*(-sin(th))+j*cos(th),map.cols/2+i*cos(th)+j*sin(th))={color.B,color.G,color.R};
//         }
//     }
// }


// void IMU_visual::put_text(cv::Mat& map,string text,cv::Point point,int font,double size,cv::Scalar value)
// {
//     cv::putText(map,text,point,font,size,value);
// }
