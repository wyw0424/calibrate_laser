/*
function：matches laser data to the image
time：2020-03
author：z
*/


#include <iostream>
#include <fstream>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
 
using namespace std;

#define __app_name__ "calibrate_laser_node"

struct Point2PointErrorCostFunction
{
  const Eigen::Vector3d point_fix;
  const Eigen::Vector3d point_move;

  Point2PointErrorCostFunction(const Eigen::Vector3d &fix, const Eigen::Vector3d &move)
  : point_fix(fix)
  , point_move(move)
  {

  }
  template <typename T>
  bool operator()(const T* const pred_pose, T* residuals) const
  {
    T m_point[3] = {T(point_move[0]), T(point_move[1]), T(point_move[2])};
    ceres::AngleAxisRotatePoint(pred_pose, m_point, m_point);

    m_point[0] += pred_pose[3];
    m_point[1] += pred_pose[4];
    m_point[2] += pred_pose[5];
    // predict - measured
    residuals[0] = m_point[0] - T(point_fix[0]);
    residuals[1] = m_point[1] - T(point_fix[1]);
    residuals[2] = m_point[2] - T(point_fix[2]);

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d &last, const Eigen::Vector3d &curr)
  {
    return (new ceres::AutoDiffCostFunction<Point2PointErrorCostFunction, 3, 6>(new Point2PointErrorCostFunction(last, curr)));
  }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

class CalibrateLaser
{
private:
	ros::NodeHandle node_handle_;

	ros::Subscriber subscriber_image_raw_;
	ros::Subscriber subscriber_points_raw_;
	ros::Subscriber subscriber_clicked_point_;
	ros::Publisher  publisher_cloudpoints_result_;
	std::string calibration_file;
	cv::Mat img;
	
	std::vector<cv::Point3f> clicked_laser_points_;
	std::vector<cv::Point3f> first_laser_points_;
	std::vector<cv::Point3f> second_laser_points_;
	Eigen::Matrix4d final_tf_;
	sensor_msgs::PointCloud2ConstPtr curr_msg;

	Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();

	/*void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4d& pose, double score) 
	{
		// broadcast the transform over tf
		geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, "map", sensor_frame_id_topic);
		pose_broadcaster.sendTransform(odom_trans);

		// publish the transform
		nav_msgs::Odometry odom;
		odom.header.stamp = stamp;
		odom.header.frame_id = "map";

		odom.pose.pose.position.x = pose(0, 3);
		odom.pose.pose.position.y = pose(1, 3);
		odom.pose.pose.position.z = pose(2, 3);
		odom.pose.pose.orientation = odom_trans.transform.rotation;

		//double match_score = pose_estimator->score();
		odom.pose.covariance[0] = score;
		odom.child_frame_id = "lidar_pose";
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = 0.0;

		pose_pub.publish(odom);
	}*/
	
	void CalibrateLasers()
	{
		std::cout << "clicked_laser_points_ Number of points: " << clicked_laser_points_.size() << std::endl;

		if (clicked_laser_points_.size() > 11)
		{
			for(int i = 0; i < clicked_laser_points_.size(); i++)
			{
				if(i % 2 == 0)
				{
					first_laser_points_.push_back(clicked_laser_points_[i]);
				}
				else
				{
					second_laser_points_.push_back(clicked_laser_points_[i]);
				}
			}
			std::cout << "first_laser_points_ Number of points: " << first_laser_points_.size() << std::endl;
			std::cout << "second_laser_points_ Number of points: " << second_laser_points_.size() << std::endl;
			
		    //std::cout << "icp problem.." << std::endl;
		    ceres::Problem icp_problem;
		    double pose[6] = {0,0,0,0,0,0};

			for(int i = 0; i< first_laser_points_.size(); i++)
		 	{
		 		Eigen::Vector3d last = Eigen::Vector3d(first_laser_points_[i].x, first_laser_points_[i].y, first_laser_points_[i].z);
		 		Eigen::Vector3d curr = Eigen::Vector3d(second_laser_points_[i].x, second_laser_points_[i].y, second_laser_points_[i].z);
		 		ceres::CostFunction* cost_function = Point2PointErrorCostFunction::Create(last, curr);
		 		icp_problem.AddResidualBlock(cost_function, NULL, pose);
		 	} 
		 	//std::cout << "solve problem.. " << std::endl;
		 	ceres::Solver::Options options;
		 	//options.linear_solver_type = ceres::DENSE_SCHUR;
		 	options.linear_solver_type = ceres::DENSE_QR;
		 	options.minimizer_progress_to_stdout = false;
		 	options.max_num_iterations = 100;

		 	ceres::Solver::Summary summary;
		 	ceres::Solve(options, &icp_problem, &summary);
		 	//std::cout << summary.BriefReport() << "\n";

		 	Eigen::Matrix3d R;
		 	ceres::AngleAxisToRotationMatrix(pose,R.data());
		 	Eigen::Vector3d translation = Eigen::Vector3d(pose[3], pose[4], pose[5]);

		 	tf.block<3, 3>(0, 0) = R;
		 	tf.block<3, 1>(0, 3) = translation;

			std::cout << "R： " << R << std::endl;
			std::cout << "translation: " << translation << std::endl;
			/*
			R：     0.305766    -0.952098  -0.00400505
    				0.952052     0.305791  -0.00942561
   					0.0101988   -0.000930984  0.999948
			translation: -0.0766164
  					0.116545
					-0.0048021
			*/

		 	// final pose
  			//final_tf_ = Eigen::Matrix4d::Identity();
		 	//final_tf_ = final_tf_* tf;
		 	// publish odom
		 	//publish_odometry(curr_msg->header.stamp, final_tf_, 0);
     	}
	}
	
	void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
	{
		clicked_laser_points_.push_back(cv::Point3f(in_clicked_point.point.x,
		                                               in_clicked_point.point.y,
		                                               in_clicked_point.point.z));
		std::cout << cv::Point3f(in_clicked_point.point.x,
		                         in_clicked_point.point.y,
		                         in_clicked_point.point.z) << std::endl << std::endl;
		CalibrateLasers();
	}

	void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//std::cout << "In pointcloud callback !" << std::endl;
		pcl::PCLPointCloud2 pcl_pc2;
    	pcl_conversions::toPCL(*msg,pcl_pc2);
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    	pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
		pcl::transformPointCloud (*cloud, *transformed_cloud, tf);

		sensor_msgs::PointCloud2 output;
		//Convert the cloud to ROS message
		pcl::toROSMsg(*transformed_cloud, output);
		output.header.frame_id = "velodyne_h";
		publisher_cloudpoints_result_.publish(output);
	}

public:
	void run()
	{
		ros::NodeHandle private_node_handle("~");  //to receive args
		std::string  imageraw_topic_str, imageresult_topic_str, laser_topic_str, calibrate_points_topic_str;
		
		private_node_handle.param<std::string>("calibration_file", calibration_file, "/home/autoware_lidar_camera_calibration.yaml");
		private_node_handle.param<std::string>("image_src", imageraw_topic_str, "/output/image");
		private_node_handle.param<std::string>("laser_points_src", laser_topic_str, "/velodyne_points");
		private_node_handle.param<std::string>("calibrate_points_topic", calibrate_points_topic_str, "/clicked_point");
		//subscriber_image_raw_ = node_handle_.subscribe(imageraw_topic_str, 1, &CalibrateLaser::ImageCallback, this);
		subscriber_points_raw_ = node_handle_.subscribe(laser_topic_str, 1, &CalibrateLaser::PointsCallback, this);
		subscriber_clicked_point_ = node_handle_.subscribe(calibrate_points_topic_str, 1, &CalibrateLaser::RvizClickedPointCallback, this);

		publisher_cloudpoints_result_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/result/cloudpoints", 1, false);

		ros::spin();
	}
};
 
int main(int argc, char* argv[])
{
	ros::init(argc, argv, __app_name__);
	
	CalibrateLaser cti;
	cti.run();
	
	return 0;
}

