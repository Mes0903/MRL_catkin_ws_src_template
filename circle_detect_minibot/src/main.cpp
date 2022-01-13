/*
 Copyright 2021 NCU MATH.
 Developer: Kuo-Shih Tseng (kuoshih@math.ncu.edu.tw)
 Description: This code activate a node "main." 
 This node subscribes three topics -- imu, odom, and scan. 
 You can access data from three Callback functions.
 $Revision: 1.0 $,  2021.08.29, revise the file from minibot
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
     http://www.apache.org/licenses/LICENSE-2.0
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

// %Tag(FULLTEXT)%
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen/Eigen"
using namespace Eigen;

//##################################
MatrixXd AppendData(MatrixXd &A, MatrixXd &B)
{
    MatrixXd D(A.rows() + B.rows(), A.cols());
    D << A, B;
    return D;
}
//##################################
float distance(std::vector<float> xy1, std::vector<float> xy2)
{
    return sqrt((xy1[0] - xy2[0]) * (xy1[0] - xy2[0]) + (xy1[1] - xy2[1]) * (xy1[1] - xy2[1]));
}
//##################################
std::vector<MatrixXd> Segment(float origin_data[][2], int size)
{
    std::vector<std::vector<float>> data;
    std::vector<MatrixXd> Returndata;

    for (int i = 0; i < size; i++) {
        if (((origin_data[i][0]!=0) || (origin_data[i][1]!=0)) && (std::isnormal(origin_data[i][0]) && std::isnormal(origin_data[i][1]))){data.push_back({origin_data[i][0], origin_data[i][1]});}
    }
    float threshold = 0.4;
    int validsize = data.size(), ReturnEndIDX = 0;
    bool one_is_end = distance(data[0], data[validsize - 1]) < threshold;

    MatrixXd tmp(1, 2);
    tmp << data[0][0], data[0][1];
    Returndata.push_back(tmp);
    for (int i = 1; i < validsize; i++) {
        tmp << data[i][0], data[i][1];
        if (distance(data[i - 1], data[i]) < threshold)
            Returndata[ReturnEndIDX] = AppendData(Returndata[ReturnEndIDX], tmp);

        else {
            ReturnEndIDX++;
            Returndata.push_back(tmp);
        }
    }

    if (one_is_end) {
        Returndata[0] = AppendData(Returndata[ReturnEndIDX], Returndata[0]);
        Returndata.pop_back();
    }
    return Returndata;
}
//##################################
struct s_r {
    float s;
    float r;
};
//##################################
struct s_r circle_fit(MatrixXd &data)
{
    MatrixXd A(data.rows(), data.cols() + 1);
    MatrixXd b(data.rows(), 1);
    b << (-1 * data.array().square().matrix()) * MatrixXd::Ones(data.cols(), 1);
    A << -2 * data, MatrixXd::Ones(data.rows(), 1);
    MatrixXd solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);

    MatrixXd center(1, data.cols());
    for (int i = 0; i < data.cols(); i++) {
        center(0, i) = solution(i, 0);
    }

    float r = solution(data.cols(), 0), s = 0;
    r = sqrt(pow(center.norm(), 2) - r);
    for (int i = 0; i < data.rows(); i++) {
        s = s + pow((data.row(i) - center).norm() - r, 2);
    }
    s = sqrt(s);
    struct s_r Returndata;
    Returndata.s = s;
    Returndata.r = r;
    return Returndata;
}
//##################################

#define RAD2DEG(x) ((x) *180. / M_PI)

void callback1(const ros::TimerEvent &);
void init_marker(void);

visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CYLINDER;
ros::Publisher marker_pub;
ros::Publisher markerArray_pub;
int counter = 0;

void callback1(const ros::TimerEvent &)
{    // update maker location and publish it.
    float x = 1 * cos(0.174 * counter);
    float y = 1 * sin(0.174 * counter);
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    //ROS_INFO("x=%f,y=%f\n",x,y);
    counter++;

    marker_pub.publish(marker);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    /* 
    ROS_INFO("V x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    ROS_INFO("W x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    */
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    visualization_msgs::MarkerArray markerArray;    //.markers.push_back(marker);
    float xy[720][2];
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    printf("[YDLIDAR INFO]: angle_increment : [%f] degree\n", RAD2DEG(scan->angle_increment));
    // 把距離資料轉換成 x,y 資料
    for (int i = 0; i < 720; i++) {
        xy[i][0] = scan->ranges[i] * cos(scan->angle_min + scan->angle_increment * i);
        xy[i][1] = scan->ranges[i] * sin(scan->angle_min + scan->angle_increment * i);
    }
    // 切分段
    std::vector<MatrixXd> Seg_data = Segment(xy, 720);
    for (int i = 0; i < Seg_data.size(); i++) {
        std::cout << i + 1 << "-th segment\n"
                  << Seg_data[i] << '\n';
        if (Seg_data[i].rows() >= 3) {
            struct s_r s_and_r = circle_fit(Seg_data[i]);    // 計算每個分段的徵圓度與半徑
            std::cout << "s :" << s_and_r.s << " , r :" << s_and_r.r << "\n\n";
            if (s_and_r.s < 0.06 && s_and_r.r < 0.15 && Seg_data[i].rows() >= 3) { // choice the parameter by yourself s:circularity , r:radius , number of point in this segment
                marker.pose.position.x = Seg_data[i].col(0).mean();
                marker.pose.position.y = Seg_data[i].col(1).mean();
                markerArray.markers.push_back(marker);
            }
        }
        else {
            std::cout << "number of point less than 3.\n";
        }
    }
    for (int i = 0; i < markerArray.markers.size(); i++) {
        markerArray.markers[i].id = i;
    }
    markerArray_pub.publish(markerArray);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");


    ros::NodeHandle n;

    ros::Subscriber sub2 = n.subscribe("/imu", 1000, imuCallback);
    ros::Subscriber sub3 = n.subscribe("/odom", 1000, odomCallback);
    ros::Subscriber sub4 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    // create a timer callback
    ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
    // create a topic "visualization_marker"
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    markerArray_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_markerArray", 1000);

    init_marker();

    /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

    ros::spin();


    return 0;
}

void init_marker(void)
{
    // Initialize maker's setting.
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "laser_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // Tag(ACTION)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //Tag(POSE)
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //Tag(LIFETIME)
    marker.lifetime = ros::Duration();
}
