// Copyright (C) 2018  Zhi Yan and Li Sun

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.
#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "av_toolkit_custom_msgs/msg/cluster_array.hpp"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>

using std::placeholders::_1;

const int region_max_ = 10; // Change this value to match how far you want to detect.


class AdaptiveClustering : public rclcpp::Node
{
    public:
      AdaptiveClustering() : Node("adaptive_clustring")
      {
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points", 1, std::bind(&AdaptiveClustering::pointCloudCallback, this, _1));
        mode_sub = this->create_subscription<std_msgs::msg::String>("yolo_mode", 10, std::bind(&AdaptiveClustering::ModeCallback, this, _1));
        cluster_array_pub_ = this->create_publisher<av_toolkit_custom_msgs::msg::ClusterArray>("clusters", 100);
        cloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 100);
        pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("poses", 100);
        marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 100);
      }

    private:
    bool print_fps_ = true;

    int cluster_size_min_ = 10;
    int cluster_size_max_ = 2200000;

    int regions_[14] = {2,3,3,3,3,3,3,2,3,3,3,3,3,3}; // VLP-16
    int frames; clock_t start_time; bool reset = true;//fps

    std::string mode;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub;
    rclcpp::Publisher<av_toolkit_custom_msgs::msg::ClusterArray>::SharedPtr cluster_array_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

    void ModeCallback(const std_msgs::msg::String::SharedPtr msg) {
        mode = msg->data;
    }

    //#define LOG
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr ros_pc2_in) {
      if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps

      /*** Convert ROS message to PCL ***/
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);

        /*** Remove ground and ceiling ***/
        pcl::IndicesPtr pc_indices(new std::vector<int>);
        pcl::PassThrough<pcl::PointXYZI> pt;


        if (mode == "delivery") {
            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("z");
            pt.setFilterLimits(0.5, 5.0);
            pt.filter(*pcl_pc_in);

            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("y");
            pt.setFilterLimits(-2.5, -1.0);
            pt.filter(*pcl_pc_in);

            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("x");
            pt.setFilterLimits(1.0, 15.0);
            pt.filter(*pc_indices);

        }
        else{
            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("z");
            pt.setFilterLimits(-0.5, 4.0);
            pt.filter(*pcl_pc_in);

            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("y");
            pt.setFilterLimits(-5.0, 5.0);
            pt.filter(*pcl_pc_in);

            pt.setInputCloud(pcl_pc_in);
            pt.setFilterFieldName("x");
            pt.setFilterLimits(0.5, 15.0);
            pt.filter(*pc_indices);
        }

        /*** Divide the point cloud into nested circular regions ***/
        boost::array<std::vector<int>, region_max_> indices_array;
        for(int i = 0; i < int(pc_indices->size()); i++) {
          float range = 0.0;
          for(int j = 0; j < region_max_; j++) { //regions_[j]의 간격만큼 나누어서 생각
            float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
      	pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
      	pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
            if(d2 > range * range && d2 <= (range+regions_[j]) * (range+regions_[j])) {
            	indices_array[j].push_back((*pc_indices)[i]);
            	break;
            }
            range += regions_[j];
          }
        }

        /*** Scan Matching ***/
        pcl::PointCloud<pcl::PointXYZI>::Ptr previous_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // Combine previous_cloud and pcl_pc_in into a single point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        *combined_cloud = *previous_cloud + *pcl_pc_in;

        if (!previous_cloud->empty()) {
          // Perform Scan Matching to estimate motion and align the combined point clouds
          pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
          icp.setInputSource(pcl_pc_in);
          icp.setInputTarget(combined_cloud);

          // Set the maximum number of iterations
          icp.setMaximumIterations(100);
          // Set the transformation epsilon (minimum difference between the two consecutive transformations)
          icp.setTransformationEpsilon(1e-6);
          // Set the maximum distance threshold between two correspondences (correspondences with higher distances will be ignored)
          icp.setMaxCorrespondenceDistance(0.05);

          icp.align(*transformed_cloud);

          if (icp.hasConverged()) {
            // Get the estimated transformation matrix
            Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
            // Apply the transformation matrix to the current point cloud
            pcl::transformPointCloud(*pcl_pc_in, *transformed_cloud, transformation_matrix);
          } else {
            // If the ICP did not converge, use the original point cloud
            transformed_cloud = pcl_pc_in;
          }
        } else {
          // If this is the first frame, use the original point cloud
          transformed_cloud = pcl_pc_in;
        }

        // Split the combined cloud back into previous_cloud and pcl_pc_in
        int combined_size = combined_cloud->size();
        int previous_size = previous_cloud->size();
        int current_size = pcl_pc_in->size();

        previous_cloud->clear();
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (int i = 0; i < combined_size; i++) {
          if (i < previous_size) {
            previous_cloud->push_back(combined_cloud->points[i]);
          } else if (i < combined_size - current_size) {
            temp_cloud->push_back(combined_cloud->points[i]);
          } else {
            pcl_pc_in->push_back(combined_cloud->points[i]);
          }
        }

        /*** Euclidean clustering ***/
        float tolerance = 0.0;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;

        for(int i = 0; i < region_max_; i++) {
          tolerance += 0.15; //라이더보다 멀리 있을 수록 tolerance가 커져 점들이 떨어져있더라도 한물체로 잡힘.
          if(int(indices_array[i].size()) > cluster_size_min_) {
            boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
            tree->setInputCloud(pcl_pc_in, indices_array_ptr);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(tolerance);
            ec.setMinClusterSize(cluster_size_min_);
            ec.setMaxClusterSize(cluster_size_max_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(pcl_pc_in);
            ec.setIndices(indices_array_ptr);
            ec.extract(cluster_indices);

            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
            	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            	  cluster->points.push_back(pcl_pc_in->points[*pit]);
        	}
            	cluster->width = cluster->size();
            	cluster->height = 1;
            	cluster->is_dense = true;
      	clusters.push_back(cluster);
            }
          }
        }

        /*** Output ***/
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
          sensor_msgs::msg::PointCloud2 ros_pc2_out;
          pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
          pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
          cloud_filtered_pub_->publish(ros_pc2_out);


        av_toolkit_custom_msgs::msg::ClusterArray cluster_array;
        geometry_msgs::msg::PoseArray pose_array;
        visualization_msgs::msg::MarkerArray marker_array;
        // if(cluster_array_pub_.getNumSubscribers() > 0){}
        for(int i = 0; i < int(clusters.size()); i++) {
            // sensor_msgs::msg::PointCloud2 ros_pc2_out;
            // pcl::toROSMsg(*clusters[i], ros_pc2_out);
            // cluster_array.clusters.push_back(ros_pc2_out);

            // if(pose_array_pub_.getNumSubscribers() > 0){}
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusters[i], centroid);

            geometry_msgs::msg::Pose pose;
            pose.position.x = centroid[0];
            pose.position.y = centroid[1];
            pose.position.z = centroid[2];
            pose.orientation.w = 1;
            pose_array.poses.push_back(pose);

      #ifdef LOG
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*clusters[i], min, max);
            std::cerr << ros_pc2_in->header.seq << " "
      		<< ros_pc2_in->header.stamp << " "
      		<< min[0] << " "
      		<< min[1] << " "
      		<< min[2] << " "
      		<< max[0] << " "
      		<< max[1] << " "
      		<< max[2] << " "
      		<< std::endl;
      #endif

            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*clusters[i], min, max);

            visualization_msgs::msg::Marker marker;
            marker.header = ros_pc2_in->header;
            marker.ns = "adaptive_clustering_v1";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;

            geometry_msgs::msg::Point p[24];
            p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
            p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
            p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
            p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
            p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
            p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
            p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
            p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
            p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
            p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
            p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
            p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
            p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
            p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
            p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
            p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
            p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
            p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
            p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
            p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
            p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
            p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
            p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
            p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
            for(int i = 0; i < 24; i++) {
        	marker.points.push_back(p[i]);
            }

            marker.scale.x = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.5;
            marker.lifetime = rclcpp::Duration(0,100000000); // 0.1초
            marker_array.markers.push_back(marker);

        }

        if(cluster_array.clusters.size()) {
          cluster_array.header = ros_pc2_in->header;
          cluster_array_pub_->publish(cluster_array);
        }

        if(pose_array.poses.size()) {
          pose_array.header = ros_pc2_in->header;
          pose_array_pub_->publish(pose_array);
        }

        if(marker_array.markers.size()) {
          marker_array_pub_->publish(marker_array);
        }

        if(print_fps_)if(++frames>10){std::cerr<<"[adaptive_clustering_v1] fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<std::endl;reset = true;}//fps
      }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AdaptiveClustering>());
    rclcpp::shutdown();
    return 0;
}
