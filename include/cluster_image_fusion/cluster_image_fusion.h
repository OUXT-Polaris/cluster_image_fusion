#ifndef CLUSTER_IMAGE_FUSION_CLUSTER_IMAGE_FUSION_H_INCLUDED
#define CLUSTER_IMAGE_FUSION_CLUSTER_IMAGE_FUSION_H_INCLUDED

// Headers in this package
#include <cluster_image_fusion/ClusterImageFusionConfig.h>

// Headers in ROS
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <hungarian_solver/hungarian_solver.h>
#include <sensor_msgs/CameraInfo.h>
#include <vision_info_server/vision_info_parser.h>

// Headers in STL
#include <memory>

// Headers in Boost
#include <boost/optional.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

// Headers in Eigen
#include <Eigen/Core>

namespace cluster_image_fusion
{
    typedef message_filters::sync_policies::ApproximateTime
        <vision_msgs::Detection2DArray, vision_msgs::Detection2DArray, vision_msgs::Detection3DArray> SyncPolicy;

    class ClusterImageFusion : public nodelet::Nodelet
    {
    protected:
        void onInit();
    private:
        std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection2DArray> > image_detection_sub_ptr_;
        std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection2DArray> > cluster_rect_sub_ptr_;
        std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection3DArray> > cluster_bbox_sub_ptr_;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_ptr_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        void callback(const vision_msgs::Detection2DArray::ConstPtr image_detection,
            const vision_msgs::Detection2DArray::ConstPtr cluster_rect,
            const vision_msgs::Detection3DArray::ConstPtr cluster_bbox);
        void paramCllback(cluster_image_fusion::ClusterImageFusionConfig &config, uint32_t level);
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg);
        boost::optional<sensor_msgs::CameraInfo> camera_info_;
        ros::Subscriber camera_info_sub_;
        dynamic_reconfigure::Server<cluster_image_fusion::ClusterImageFusionConfig> server_;
        dynamic_reconfigure::Server<cluster_image_fusion::ClusterImageFusionConfig>::CallbackType param_func_;
        cluster_image_fusion::ClusterImageFusionConfig config_;
        vision_msgs::Detection2DArray filterDetection(const vision_msgs::Detection2DArray::ConstPtr detection);
        std::pair<vision_msgs::Detection2DArray,vision_msgs::Detection3DArray> filterClusterDetection
            (const vision_msgs::Detection2DArray::ConstPtr detection, const vision_msgs::Detection3DArray::ConstPtr detection_3d);
        vision_info_parser::VisionInfoParser parser_;
        ros::Subscriber vision_info_sub_;
        void visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg);
        double getIOU(vision_msgs::BoundingBox2D rect0,vision_msgs::BoundingBox2D rect1);
        hungarian_solver::Solver solver_;
        boost::optional<Eigen::MatrixXd> getCostMatrix(vision_msgs::Detection2DArray image_detection_filtered,vision_msgs::Detection2DArray cluster_rect_filtered);
        ros::Publisher fusion_result_pub_;
    };
}

#endif  //CLUSTER_IMAGE_FUSION_CLUSTER_IMAGE_FUSION_H_INCLUDED