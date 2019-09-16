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
        vision_info_sub_ = pnh_.subscribe("input/vision_info",1,&ClusterImageFusion::visionInfoCallback,this);
        param_func_ = boost::bind(&ClusterImageFusion::paramCllback, this, _1, _2);
        server_.setCallback(param_func_);
    }

    void ClusterImageFusion::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg)
    {
        parser_.parseFromRosMessage(*msg);
        return;
    }

    void ClusterImageFusion::callback(const vision_msgs::Detection2DArray::ConstPtr image_detection,const vision_msgs::Detection2DArray::ConstPtr cluster_rect)
    {
        if(camera_info_ && parser_.getClasses())
        {
            vision_msgs::Detection2DArray image_detection_filtered = filterDetection(image_detection);
            vision_msgs::Detection2DArray cluster_rect_filtered = filterDetection(cluster_rect);
            return;
        }
        return;
    }

    vision_msgs::Detection2DArray ClusterImageFusion::filterDetection(const vision_msgs::Detection2DArray::ConstPtr detection)
    {
        vision_msgs::Detection2DArray ret;
        ret.header = detection->header;
        for(auto itr = detection->detections.begin(); itr != detection->detections.end(); itr++)
        {
            if(itr->bbox.size_x > config_.min_width && itr->bbox.size_y > config_.min_height)
            {
                if((itr->bbox.center.x+itr->bbox.size_x*0.5)>=0 || (itr->bbox.center.x-itr->bbox.size_x*0.5)<=camera_info_->width)
                {
                    if((itr->bbox.center.y+itr->bbox.size_y*0.5)>=0 || (itr->bbox.center.y-itr->bbox.size_y*0.5)<=camera_info_->width)
                    {
                        ret.detections.push_back(*itr);
                    }
                }
            }
        }
        return ret;
    }

    void ClusterImageFusion::paramCllback(cluster_image_fusion::ClusterImageFusionConfig &config, uint32_t level)
    {
        config_ = config;
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