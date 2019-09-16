#include <cluster_image_fusion/cluster_image_fusion.h>

namespace cluster_image_fusion
{
    void ClusterImageFusion::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        image_detection_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection2DArray> >(pnh_, "input/image_detection", 10);
        cluster_rect_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection2DArray> >(pnh_, "input/cluster_rect", 10);
        cluster_bbox_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection3DArray> >(pnh_, "input/cluster_bbox", 10);
        fusion_result_pub_ = pnh_.advertise<vision_msgs::Detection3DArray>("output/fusion_result",1);
        sync_ptr_ = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(10),*image_detection_sub_ptr_,*cluster_rect_sub_ptr_,*cluster_bbox_sub_ptr_);
        sync_ptr_->registerCallback(boost::bind(&ClusterImageFusion::callback, this, _1, _2, _3));
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

    std::pair<vision_msgs::Detection2DArray,vision_msgs::Detection3DArray> ClusterImageFusion::filterClusterDetection
        (const vision_msgs::Detection2DArray::ConstPtr detection, const vision_msgs::Detection3DArray::ConstPtr detection_3d)
    {
        ROS_ASSERT(detection->detections.size() == detection_3d->detections.size());
        std::pair<vision_msgs::Detection2DArray,vision_msgs::Detection3DArray> ret;
        ret.first.header = detection->header;
        ret.second.header = detection_3d->header;
        int num_detections = detection->detections.size();
        for(int i=0; i<num_detections; i++)
        {
            if(detection->detections[i].bbox.size_x > config_.min_width && detection->detections[i].bbox.size_y > config_.min_height)
            {
                if((detection->detections[i].bbox.center.x+detection->detections[i].bbox.size_x*0.5)>=0 || 
                    (detection->detections[i].bbox.center.x+detection->detections[i].bbox.size_x*0.5)<=camera_info_->width)
                    {
                        if((detection->detections[i].bbox.center.y+detection->detections[i].bbox.size_y*0.5)>=0 || 
                            (detection->detections[i].bbox.center.y+detection->detections[i].bbox.size_y*0.5)<=camera_info_->height)
                            {
                                ret.first.detections.push_back(detection->detections[i]);
                                ret.second.detections.push_back(detection_3d->detections[i]);
                            }
                    }
            }
        }
        return ret;
    }

    void ClusterImageFusion::callback(const vision_msgs::Detection2DArray::ConstPtr image_detection,
        const vision_msgs::Detection2DArray::ConstPtr cluster_rect,const vision_msgs::Detection3DArray::ConstPtr cluster_bbox)
    {
        vision_msgs::Detection3DArray fusion_result;
        fusion_result.header = cluster_bbox->header;
        if(camera_info_ && parser_.getClasses())
        {
            vision_msgs::Detection2DArray image_detection_filtered = filterDetection(image_detection);
            int num_image_detection_filtered = image_detection_filtered.detections.size();
            std::pair<vision_msgs::Detection2DArray,vision_msgs::Detection3DArray> cluster_detection_filtered 
                = filterClusterDetection(cluster_rect,cluster_bbox);
            int num_cluster_rect_detection_filtered = cluster_detection_filtered.first.detections.size();
            if(num_image_detection_filtered==0 || num_cluster_rect_detection_filtered==0)
            {
                fusion_result_pub_.publish(fusion_result);
                return;
            }
            Eigen::MatrixXd mat = getCostMatrix(image_detection_filtered,cluster_detection_filtered.first);
            try
            {
                boost::optional<std::vector<std::pair<int,int> > > match = solver_.solve(mat,10);
                if(match)
                {
                    for(auto itr=match->begin(); itr!=match->end(); itr++)
                    {
                        vision_msgs::Detection3D cluster_bbox_detection = cluster_detection_filtered.second.detections[itr->first];
                        vision_msgs::Detection2D cluster_detection = cluster_detection_filtered.first.detections[itr->first];
                        vision_msgs::Detection2D image_detection = image_detection_filtered.detections[itr->second];
                        cluster_bbox_detection.results = image_detection.results;
                        fusion_result.detections.push_back(cluster_bbox_detection);
                    }
                }
                else
                {
                    fusion_result_pub_.publish(fusion_result);
                    ROS_WARN_STREAM("Failed to find match result.");
                }
            }
            catch(...)
            {
                fusion_result_pub_.publish(fusion_result);
                ROS_WARN_STREAM("Failed to find match result.");
                return;
            }
            fusion_result_pub_.publish(fusion_result);
            return;
        }
        fusion_result_pub_.publish(fusion_result);
        return;
    }

    Eigen::MatrixXd ClusterImageFusion::getCostMatrix(vision_msgs::Detection2DArray image_detection_filtered,vision_msgs::Detection2DArray cluster_rect_filtered)
    {
        Eigen::MatrixXd mat(cluster_rect_filtered.detections.size(),image_detection_filtered.detections.size());
        for(int r=0; r<cluster_rect_filtered.detections.size(); r++)
        {
            for(int c=0; c<image_detection_filtered.detections.size(); c++)
            {
                mat(r,c) = 1.0-getIOU(cluster_rect_filtered.detections[r].bbox,image_detection_filtered.detections[c].bbox);
            }
        }
        return mat;
    }

    double ClusterImageFusion::getIOU(vision_msgs::BoundingBox2D rect0,vision_msgs::BoundingBox2D rect1)
    {
        std::array<double,4> box_a,box_b;
        box_a[0] = rect0.center.x-rect0.size_x*0.5;
        box_a[1] = rect0.center.y-rect0.size_y*0.5;
        box_a[2] = rect0.size_x;
        box_a[3] = rect0.size_y;
        box_b[0] = rect1.center.x-rect1.size_x*0.5;
        box_b[1] = rect1.center.y-rect1.size_y*0.5;
        box_b[2] = rect1.size_x;
        box_b[3] = rect1.size_y;
        double x_a = std::max(box_a[0], box_b[0]);
        double y_a = std::max(box_a[1], box_b[1]);
        double x_b = std::min(box_a[2], box_b[2]);
        double y_b = std::min(box_a[3], box_b[3]);
        double inter_area = std::max(0.0,x_b-x_a+1.0)*std::max(0.0,y_b-y_a+1.0);
        double box_a_area = (box_a[2] - box_a[0] + 1.0) * (box_a[3] - box_a[1] + 1.0);
        double box_b_area = (box_b[2] - box_b[0] + 1.0) * (box_b[3] - box_b[1] + 1.0);
        return inter_area / float(box_a_area + box_b_area - inter_area);
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
                    if((itr->bbox.center.y+itr->bbox.size_y*0.5)>=0 || (itr->bbox.center.y-itr->bbox.size_y*0.5)<=camera_info_->height)
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