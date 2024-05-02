#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<string>
#include<iostream>
#include<sstream>
#include<cmath>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <iostream>


#define POSE_TOPIC1 "/orb_slam3/camera_pose"      //Less accurate, like orb_slam3_ros
#define POSE_TOPIC2 "/vrpn_client_node/Azure/pose"     //more accurate, like OptiTrack

#define Traj_save_path "/home/xingtian/catkin_kin/save/traj.txt"

int camera_num = 1;

void deal_msg(const geometry_msgs::PoseStamped::ConstPtr& pose){
    // 打印stamp
    ROS_INFO("stamp: %lf", pose->header.stamp.toSec());

    ROS_INFO("fram_id: %s", pose->header.frame_id.c_str());
    ROS_INFO("position_x: %lf", pose->pose.position.x);
    ROS_INFO("position_y: %lf", pose->pose.position.y);
    ROS_INFO("position_z: %lf", pose->pose.position.z);
    ROS_INFO("orientation_x: %lf", pose->pose.orientation.x);
    ROS_INFO("orientation_y: %lf", pose->pose.orientation.y);
    ROS_INFO("orientation_z: %lf", pose->pose.orientation.z);
    ROS_INFO("orientation_w: %lf", pose->pose.orientation.w);
}

// 用于将四元数转换为旋转矩阵
void quat2rot(const double qx, const double qy, const double qz, const double qw, double &r11, double &r12, double &r13, double &r21, double &r22, double &r23, double &r31, double &r32, double &r33)
{
    double sqx = qx * qx;
    double sqy = qy * qy;
    double sqz = qz * qz;
    double sqw = qw * qw;

    // 旋转矩阵的第一列
    r11 = sqx - sqy - sqz + sqw;
    r21 = 2 * (qx * qy - qz * qw);
    r31 = 2 * (qx * qz + qy * qw);

    // 旋转矩阵的第二列
    r12 = 2 * (qx * qy + qz * qw);
    r22 = -sqx + sqy - sqz + sqw;
    r32 = 2 * (qy * qz - qx * qw);

    // 旋转矩阵的第三列
    r13 = 2 * (qx * qz - qy * qw);
    r23 = 2 * (qy * qz + qx * qw);
    r33 = -sqx - sqy + sqz + sqw;
}

/**
 * 根据相机在世界坐标系中的位置和朝向，计算相机的外参矩阵。
 * @param cam_id 第几个相机
 * @param position_x 相机在x轴方向上相对世界坐标系的位置，单位为米。
 * @param position_y 相机在y轴方向上相对世界坐标系的位置，单位为米。
 * @param position_z 相机在z轴方向上相对世界坐标系的位置，单位为米。
 * @param orientation_x 相机固有坐标系x轴与世界坐标系之间的旋转角度，单位为弧度。
 * @param orientation_y 相机固有坐标系y轴与世界坐标系之间的旋转角度，单位为弧度。
 * @param orientation_z 相机固有坐标系z轴与世界坐标系之间的旋转角度，单位为弧度。
 * @param orientation_w 用于表示旋转的四元数的w分量，没有单位。
 * @return 返回相机的外参矩阵（4x4），其中第1-3行前3列为旋转矩阵R，第4列为平移向量T，第4行为[0, 0, 0, 1]。
 */
void getExtrinsicMatrix(const double position_x, const double position_y, const double position_z, const double orientation_x, const double orientation_y, const double orientation_z, const double orientation_w, double *extrinsicMatrix)
{
    // 将四元数转换为旋转矩阵
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    quat2rot(orientation_x, orientation_y, orientation_z, orientation_w, r11, r12, r13, r21, r22, r23, r31, r32, r33);

    // 将平移向量转换为矩阵形式
    extrinsicMatrix[0] = r11;
    extrinsicMatrix[1] = r12;
    extrinsicMatrix[2] = r13;
    extrinsicMatrix[3] = position_x;

    extrinsicMatrix[4] = r21;
    extrinsicMatrix[5] = r22;
    extrinsicMatrix[6] = r23;
    extrinsicMatrix[7] = position_y;

    extrinsicMatrix[8] = r31;
    extrinsicMatrix[9] = r32;
    extrinsicMatrix[10] = r33;
    extrinsicMatrix[11] = position_z;

    // 矩阵的最后一行
    extrinsicMatrix[12] = 0;
    extrinsicMatrix[13] = 0;
    extrinsicMatrix[14] = 0;
    extrinsicMatrix[15] = 1;

    
}

void create_RT (const geometry_msgs::PoseStamped::ConstPtr& pose, double *RT) {
    // 回调函数的主题部分
    // 每个相机都会生成一个独立的函数，所以使用lambda的形式来写
    double position_x = pose->pose.position.x;        //声明相机参数
    double position_y = pose->pose.position.y;
    double position_z = pose->pose.position.z;
    double orientation_x = pose->pose.orientation.x;
    double orientation_y = pose->pose.orientation.y;
    double orientation_z = pose->pose.orientation.z;
    double orientation_w = pose->pose.orientation.w;

    // 生成外参矩阵，保存到全局变量中
    getExtrinsicMatrix(position_x, position_y, position_z, orientation_x, 
                       orientation_y, orientation_z, orientation_w, RT);
};

void print_RT(double *RT)
{
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            std::cout << RT[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }
}


void recode_poses(const geometry_msgs::PoseStamped::ConstPtr& pose1, const geometry_msgs::PoseStamped::ConstPtr& pose2)
{

    ROS_INFO("recieved");

    // 两个相机的外参矩阵
    double RT1[16];
    double RT2[16];
    create_RT(pose1, RT1);
    create_RT(pose2, RT2);

    // 打印两个相机的外参矩阵
    std::cout << "RT1: " << std::endl;
    print_RT(RT1);

    std::cout << "RT2: " << std::endl;
    print_RT(RT2);

    // 记录二者同步后的数据，保存到txt文件中
    std::ofstream file(Traj_save_path, std::ios_base::app);

    if (!file) {
        std::cerr << "Unable to open file " << Traj_save_path << std::endl;
        return;
    }

    file << "TimeStamp: " << pose1->header.stamp.toSec() << "  " << pose2->header.stamp.toSec() << "\n";

    file << "Real RT Matrix:\n";
    file << "Position: " << pose1->pose.position.x << ", " << pose1->pose.position.y << ", " << pose1->pose.position.z << ")\n";
    file << "Orientation: " << pose1->pose.orientation.w << ", " << pose1->pose.orientation.x << ", " << pose1->pose.orientation.y << ", " << pose1->pose.orientation.z << ")\n";

    // Write the calculated RT matrix
    file << "Calculated RT Matrix:\n";
    file << "Position: " << pose2->pose.position.x << ", " << pose2->pose.position.y << ", " << pose2->pose.position.z << ")\n";
    file << "Orientation: " << pose2->pose.orientation.w << ", " << pose2->pose.orientation.x << ", " << pose2->pose.orientation.y << ", " << pose2->pose.orientation.z << ")\n";

    file.close();
}


void campare_poses_RT(const geometry_msgs::PoseStamped::ConstPtr& pose1, const geometry_msgs::PoseStamped::ConstPtr& pose2)
{

    ROS_INFO("recieved");

    // 两个相机的外参矩阵
    double RT1[16];
    double RT2[16];
    create_RT(pose1, RT1);
    create_RT(pose2, RT2);

    // 打印两个相机的外参矩阵
    std::cout << "RT1: " << std::endl;
    print_RT(RT1);

    std::cout << "RT2: " << std::endl;
    print_RT(RT2);

    // 计算两个相机之间的相对位姿
    double RT1_inv[16];
    double RT2_to_1[16];
    double RT2_to_1_inv[16];
    double RT1_to_2[16];
    double RT1_to_2_inv[16];
    double RT_relative[16];

    // 计算RT1的逆矩阵
    double det = RT1[0] * RT1[5] * RT1[10] + RT1[1] * RT1[6] * RT1[8] + RT1[2] * RT1[4] * RT1[9] - RT1[2] * RT1[5] * RT1[8] - RT1[1] * RT1[4] * RT1[10] - RT1[0] * RT1[6] * RT1[9];
    RT1_inv[0] = (RT1[5] * RT1[10] - RT1[6] * RT1[9]) / det;
    RT1_inv[1] = (RT1[2] * RT1[9] - RT1[1] * RT1[10]) / det;
    RT1_inv[2] = (RT1[1] * RT1[6] - RT1[2] * RT1[5]) / det;
    RT1_inv[3] = -(RT1[3] * RT1_inv[0] + RT1[7] * RT1_inv[1] + RT1[11] * RT1_inv[2]);
    RT1_inv[4] = (RT1[6] * RT1[8] - RT1[4] * RT1[10]) / det;
    RT1_inv[5] = (RT1[0] * RT1[10] - RT1[2] * RT1[8]) / det;
    RT1_inv[6] = (RT1[2] * RT1[4] - RT1[0] * RT1[6]) / det;
    RT1_inv[7] = -(RT1[3] * RT1_inv[4] + RT1[7] * RT1_inv[5] + RT1[11] * RT1_inv[6]);
    RT1_inv[8] = (RT1[4] * RT1[9] - RT1[5] * RT1[8]) / det;
    RT1_inv[9] = (RT1[1] * RT1[8] - RT1[0] * RT1[9]) / det;
    RT1_inv[10] = (RT1[0] * RT1[5] - RT1[1] * RT1[4]) / det;
    RT1_inv[11] = -(RT1[3] * RT1_inv[8] + RT1[7] * RT1_inv[9] + RT1[11] * RT1_inv[10]);
    RT1_inv[12] = 0;
    RT1_inv[13] = 0;
    RT1_inv[14] = 0;
    RT1_inv[15] = 1;

    // 计算RT2相对于RT1的位姿
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            RT2_to_1[i * 4 + j] = 0;
            for (int k = 0; k < 4; k++)
            {
                RT2_to_1[i * 4 + j] += RT1_inv[i * 4 + k] * RT2[k * 4 + j];
            }
        }
    }

    // 计算RT2相对于RT1的逆矩阵

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "get_camera_pose");

    ros::NodeHandle n;

    ROS_INFO("Start");
    
    /*No Sync*/
    // ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC1, 1000, deal_msg);
    // ros::Subscriber sub2 = n.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC2, 1000, deal_msg);


    /*Sync*/
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub1(n, POSE_TOPIC1, 1000);
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub2(n, POSE_TOPIC2, 1000);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), sub1, sub2);
    sync.registerCallback(boost::bind(&recode_poses, _1, _2));

    ros::spin();            

    return 0;
}