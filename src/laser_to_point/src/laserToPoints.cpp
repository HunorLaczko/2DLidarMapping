#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf/message_filter.h"
 #include "tf/transform_datatypes.h"
#include "message_filters/subscriber.h"
#include "/home/luni/catkin_ws/src/laser_geometry/include/laser_geometry/laser_geometry.h"

ros::Publisher pub1;
ros::Publisher pub2;

class LaserScanToPointCloud
{

public:

    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_sub_2;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;
    geometry_msgs::Pose currentPose;

    LaserScanToPointCloud(ros::NodeHandle n) :
        n_(n),
        laser_sub_(n_, "/scan", 10),

        laser_notifier_(laser_sub_,listener_, "laser", 10)
    {

        laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        sensor_msgs::PointCloud cloud;
        try
        {
            projector_.transformLaserScanToPointCloud(
                "laser",*scan_in, cloud,listener_);
        }
        catch (tf::TransformException& e)
        {
            std::cout << e.what();
            return;
        }

        // Do something with cloud.
        //std::cout<<currentPose.orientation.x <<std::endl;
        /*for(size_t i = 0; i < cloud.points.size(); i++ )
        {
            cloud.points[i].x += currentPose.position.x;
            cloud.points[i].y += currentPose.position.y;
            cloud.points[i].z += currentPose.position.z;
            //cloud.points[i] += currentPose.position;
        }*/
        scan_pub_.publish(cloud);

    }



};

void poseCallbackCam (const geometry_msgs::PoseStamped::ConstPtr& pose_in, LaserScanToPointCloud* l)
{
/// Translation
    //std::cout<<pose_in->pose.position.x <<std::endl;
    geometry_msgs::PoseStamped orig = *pose_in;
    float tmp = orig.pose.position.y;
    orig.pose.position.y = -orig.pose.position.z;
    orig.pose.position.z = tmp;
    //orig.pose.position.x = orig.pose.position.x;

/// Rotation (translation)
    float tmp_ori = orig.pose.orientation.y;
    orig.pose.orientation.y = -orig.pose.orientation.z;
    orig.pose.orientation.z = tmp_ori;

    //orig.pose.orientation.x = -orig.pose.orientation.x;
    //orig.pose.orientation.y = -orig.pose.orientation.y;
    //orig.pose.orientation.z = -orig.pose.orientation.z;

/// Rotation (rotation)
    l->currentPose = pose_in->pose;
    tf::Quaternion q_rot, q_orig, q_new;
    double r=0, p=0, y=3.14159;
    q_rot = tf::createQuaternionFromRPY(r, p, y);
    quaternionMsgToTF(orig.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    quaternionTFToMsg(q_new, l->currentPose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type
    quaternionTFToMsg(q_new, orig.pose.orientation);

    //l->currentPose.orientation.z = -l->currentPose.orientation.z;
    //l->currentPose.orientation.w = -l->currentPose.orientation.w;

/// Laser transformations
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "laser";
    static_transformStamped.child_frame_id = "world";
    static_transformStamped.transform.translation.x = l->currentPose.position.x;
    static_transformStamped.transform.translation.y = l->currentPose.position.y;
    static_transformStamped.transform.translation.z = l->currentPose.position.z;
    //tf2::Quaternion quat;
    //quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    static_transformStamped.transform.rotation.x = l->currentPose.orientation.x;
    static_transformStamped.transform.rotation.y = l->currentPose.orientation.y;
    static_transformStamped.transform.rotation.z = l->currentPose.orientation.z;
    static_transformStamped.transform.rotation.w = l->currentPose.orientation.w;
    //static_broadcaster.sendTransform(static_transformStamped);

/// Publish
    pub1.publish(orig);
}

void poseCallbackObj (const geometry_msgs::PoseStamped::ConstPtr& pose_in, LaserScanToPointCloud* l)
{
/// Translation
    //std::cout<<pose_in->pose.position.x <<std::endl;
    geometry_msgs::PoseStamped orig = *pose_in;
    float tmp = orig.pose.position.y;
    orig.pose.position.y = -orig.pose.position.z;
    orig.pose.position.z = tmp;
    //orig.pose.position.x = orig.pose.position.x;

/// Rotation (translation)
    float tmp_ori = orig.pose.orientation.y;
    orig.pose.orientation.y = -orig.pose.orientation.z;
    orig.pose.orientation.z = tmp_ori;

    //orig.pose.orientation.x = -orig.pose.orientation.x;
    //orig.pose.orientation.y = -orig.pose.orientation.y;
    //orig.pose.orientation.z = -orig.pose.orientation.z;

/// Rotation (rotation)
    l->currentPose = pose_in->pose;
    tf::Quaternion q_rot, q_orig, q_new;
    double r=0, p=0, y=3.14159;
    q_rot = tf::createQuaternionFromRPY(r, p, y);
    quaternionMsgToTF(orig.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    quaternionTFToMsg(q_new, l->currentPose.orientation);  // Stuff the new rotation back into the pose. This requires conversion into a msg type
    quaternionTFToMsg(q_new, orig.pose.orientation);


    //l->currentPose.orientation.z = -l->currentPose.orientation.z;
    //l->currentPose.orientation.w = -l->currentPose.orientation.w;

/// Publish
    pub2.publish(orig);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "my_scan_to_cloud");
    ros::NodeHandle n;
    LaserScanToPointCloud lstopc(n);

/// Camera vector
    lstopc.pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/drone_1/pose", 100, boost::bind(poseCallbackCam, _1, &lstopc));
    pub1 = n.advertise<geometry_msgs::PoseStamped>("repose", 1000);

/// Objecct vector
    lstopc.pose_sub_2 = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/drone_2/pose", 100, boost::bind(poseCallbackObj, _1, &lstopc));
    pub2 = n.advertise<geometry_msgs::PoseStamped>("repose2", 1000);


    ros::spin();
    return 0;
}
