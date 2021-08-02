# MaskRCNN progress
```bash
image_segmentation_sync_policy_ =
    new message_filters::Synchronizer<ImageSegmentationSyncPolicy>(
        ImageSegmentationSyncPolicy(kQueueSize), *depth_image_sub_,
        *rgb_image_sub_, *instance_segmentation_sub_);
->
image_segmentation_sync_policy_->registerCallback(boost::bind(
          &DepthSegmentationNode::imageSegmentationCallback, this, _1, _2, _3));
->
void imageSegmentationCallback(
    const sensor_msgs::Image::ConstPtr& depth_msg,
    const sensor_msgs::Image::ConstPtr& rgb_msg,
    const mask_rcnn_ros::Result::ConstPtr& segmentation_msg)
->
semanticInstanceSegmentationFromRosMsg(segmentation_msg,
    &instance_segmentation);

build a new instance_segmentation from segmentation_msg
save all masks as cv::Mat to semantic_instance_segmentation->masks
save all class_ids to semantic_instance_segmentation->labels
```
->
```bash
void imageSegmentationCallback(
    const sensor_msgs::Image::ConstPtr& depth_msg,
    const sensor_msgs::Image::ConstPtr& rgb_msg,
    const mask_rcnn_ros::Result::ConstPtr& segmentation_msg)
->
void preprocess(const sensor_msgs::Image::ConstPtr& depth_msg,
                const sensor_msgs::Image::ConstPtr& rgb_msg,
                cv::Mat* rescaled_depth,
                cv::Mat* dilated_rescaled_depth,
                cv_bridge::CvImagePtr cv_rgb_image,
                cv_bridge::CvImagePtr cv_depth_image,
                cv::Mat* bw_image,
                cv::Mat* mask)

change mask_rcnn result to cv::Mat data format
->
computeEdgeMap(depth_msg, rgb_msg, dilated_rescaled_depth, cv_rgb_image,
               cv_depth_image, bw_image, mask, &depth_map, &normal_map,
               &edge_map);

compute depth, normal and edge map
->
depth_segmenter_.labelMap(cv_rgb_image->image, rescaled_depth,
                          instance_segmentation, depth_map, edge_map,
                          normal_map, &label_map, &segment_masks,
                          &segments);

create the labels for depth, normal and label
->

```
# label_tsdf_integrator.cc
```bash
void Controller::processSegment(
    const sensor_msgs::PointCloud2::Ptr& segment_point_cloud_msg)
->
integrator_->computeSegmentLabelCandidates(
    segment, &segment_label_candidates, &segment_merge_candidates_)
increaseLabelCountForSegment(segment, label, segment_points_count,
                                     candidates, &merge_candidate_labels)

set std::map<Label, std::map<Segment*, size_t>> segment_label_candidates
which is called candidates in function increaseLabelCountForSegment
->
void Controller::integrateFrame(ros::Time msg_timestamp)

the max_segment bellow needs its segment_label_candidates
->
void LabelTsdfIntegrator::decideLabelPointClouds(
    std::vector<voxblox::Segment*>* segments_to_integrate,
    std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
    std::map<Segment*, std::vector<Label>>* segment_merge_candidates)

std::set<Segment*> labelled_segments comes from
std::pair<Segment*, Label> pair;
we need pair.first.instance_label_ != 0u
->
bool LabelTsdfIntegrator::getNextSegmentLabelPair(
    const std::set<Segment*>& labelled_segments,
    std::set<Label>* assigned_labels,
    std::map<voxblox::Label, std::map<voxblox::Segment*, size_t>>* candidates,
    std::map<Segment*, std::vector<Label>>* segment_merge_candidates,
    std::pair<Segment*, Label>* segment_label_pair)

std::pair<Segment*, Label> pair comes from
Segment* max_segment = candidates[*].second[*].first := segment_it
which satisfied
(count_greater_than_max && count_greater_than_min && is_unlabelled)
where
bool count_greater_than_max = segment_it->second > max_count;
bool count_greater_than_min =
    segment_it->second > label_tsdf_config_.min_label_voxel_count;
bool is_unlabelled =
    labelled_segments.find(segment_it->first) == labelled_segments.end();
->

```
