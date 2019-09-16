#include <cluster_image_fusion/cluster_image_fusion.h>

namespace cluster_image_fusion
{
    void ClusterImageFusion::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        image_detection_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection2DArray> >(pnh_, "input/image_detection", 10);
        cluster_rect_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection2DArray> >(pnh_, "input/cluster_rect", 10);
        sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10),*image_detection_sub_ptr_,*cluster_rect_sub_ptr_);
        sync_ptr_->registerCallback(boost::bind(&ClusterImageFusion::callback, this, _1, _2));
        camera_info_sub_ = pnh_.subscribe("input/camera_info",1,&ClusterImageFusion::cameraInfoCallback,this);
    };

    void ClusterImageFusion::callback(const vision_msgs::Detection2DArray::ConstPtr image_detection,const vision_msgs::Detection2DArray::ConstPtr cluster_rect)
    {
        if(!camera_info_)
        {
            return;
        }
        return;
    }

    void ClusterImageFusion::paramCllback(cluster_image_fusion::ClusterImageFusionConfig &config, uint32_t level)
    {
        return;
    }

    void ClusterImageFusion::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr msg)
    {
        camera_info_ = *msg;
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cluster_image_fusion::ClusterImageFusion,nodelet::Nodelet);