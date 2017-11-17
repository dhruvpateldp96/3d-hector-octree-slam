//=================================================================================================
// Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>

#include <filters/filter_chain.h>

class LaserscanToPointcloud
{
public:

  LaserscanToPointcloud()
    : filter_chain_("sensor_msgs::LaserScan")
  {
    ros::NodeHandle nh_;

    scan_sub_ = nh_.subscribe("scan", 10, &LaserscanToPointcloud::scanCallback, this);
    point_cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud",10,false);


    ros::NodeHandle pnh_("~");
    pnh_.param("max_range", p_max_range_, 29.0);
    pnh_.param("min_range", p_min_range_, 0.0);

    filter_chain_.configure("scan_filter_chain", pnh_);

    pnh_.param("use_high_fidelity_projection", p_use_high_fidelity_projection_, false);

    if (p_use_high_fidelity_projection_){
      pnh_.param("target_frame", p_target_frame_, std::string("NO_TARGET_FRAME_SPECIFIED"));

      if (p_target_frame_ == "NO_TARGET_FRAME_SPECIFIED"){
        ROS_ERROR("No target frame specified! Needs to be set for high fidelity projection to work");
        p_use_high_fidelity_projection_ = false;
        return;
      }

      tfl_.reset(new tf::TransformListener());
      wait_duration_ = ros::Duration(0.5);

    }
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    filter_chain_.update(*scan_in, scan_filtered_);

    cloud2_.data.clear();

    const sensor_msgs::LaserScan* scan_to_convert = &scan_filtered_;


    if (p_min_range_ > 0.0){
      scan_min_range_ = scan_filtered_;

      size_t num_scans = scan_min_range_.ranges.size();

      std::vector<float>& ranges_vec = scan_min_range_.ranges;

      float min_range = static_cast<float>(p_min_range_);

      for (size_t i = 0; i < num_scans; ++i){
        if (ranges_vec[i] < min_range){
          ranges_vec[i] = -INFINITY;
        }
      }

      scan_to_convert = &scan_min_range_;
    }

    if (p_use_high_fidelity_projection_){

      ros::Time end_time   = scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment) ;

      if(tfl_->waitForTransform(p_target_frame_, scan_in->header.frame_id, scan_in->header.stamp, wait_duration_) &&
         tfl_->waitForTransform(p_target_frame_, scan_in->header.frame_id, end_time, wait_duration_)){
        projector_.transformLaserScanToPointCloud(p_target_frame_, *scan_to_convert, cloud2_, *tfl_, p_max_range_, laser_geometry::channel_option::Intensity);
      }else{
        ROS_WARN("Timed out waiting for transform between %s and %s for %f seconds. Unable to transform laser scan.",p_target_frame_.c_str(), scan_in->header.frame_id.c_str(), wait_duration_.toSec());
        return;
      }

    }else{
      projector_.projectLaser(*scan_to_convert, cloud2_, p_max_range_, laser_geometry::channel_option::Intensity);
    }

    if (cloud2_.data.size() > 0){
      point_cloud2_pub_.publish(cloud2_);
    }
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud2_pub_;

  boost::shared_ptr<tf::TransformListener> tfl_;
  ros::Duration wait_duration_;

  double p_max_range_;
  double p_min_range_;
  bool p_use_high_fidelity_projection_;
  std::string p_target_frame_;

  laser_geometry::LaserProjection projector_;

  sensor_msgs::PointCloud2 cloud2_;
  sensor_msgs::LaserScan scan_min_range_;

  sensor_msgs::LaserScan scan_filtered_;
  
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_laserscan_to_pointcloud_node");

  LaserscanToPointcloud ls;

  ros::spin();
}
