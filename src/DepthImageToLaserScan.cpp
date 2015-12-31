/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

namespace depthimage_to_laserscan
{
/**
 * Specialization for this bgr8 image. The original depth image saved
 * depth in uint16 but encoded in rgb565, so conversion from bgr8 to
 * rgb565 is needed.
 */
template<>
void DepthImageToLaserScan::convert<uint8_t>(const sensor_msgs::ImageConstPtr& depth_msg,
  const image_geometry::PinholeCameraModel& cam_model,
  const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) const
{
  // Use correct principal point from calibration
  float center_x = cam_model.cx();
  float center_y = cam_model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(1);
  float constant_x = unit_scaling / cam_model.fx();
  float constant_y = unit_scaling / cam_model.fy();

  const uint8_t* depth_row = reinterpret_cast<const uint8_t*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(uint8_t);

  int offset = (int)(cam_model.cy() - scan_height / 2);
  depth_row += offset * row_step; // Offset to center of image

  for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step)
  {
    for(int u = 0; u < (int)depth_msg->width; ++u) // Loop over each pixel in row
    {
      int byte_idx = 3 * u;
      // bgr8 to rgb565 to uint16.
      uint16_t blue = depth_row[byte_idx] >> 3;
      uint16_t green = depth_row[byte_idx + 1] >> 2;
      uint16_t red = depth_row[byte_idx + 2] >> 3;
      uint16_t depth = (red << 11) | (green << 5) | blue;

      double r = depth; // Assign to pass through NaNs and Infs
      double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
      int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

      if(depthimage_to_laserscan::DepthTraits<uint16_t>::valid(depth)) // Not NaN or Inf
      {
        // Calculate in XYZ
        double x = (u - center_x) * depth * constant_x;
        double z = depthimage_to_laserscan::DepthTraits<uint16_t>::toMeters(depth);

        // Calculate actual distance
        r = sqrt(pow(x, 2.0) + pow(z, 2.0));
      }

      // Determine if this point should be used.
      if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max))
      {
        scan_msg->ranges[index] = r;
      }
    }
  }
}

DepthImageToLaserScan::DepthImageToLaserScan(){
}

DepthImageToLaserScan::~DepthImageToLaserScan(){
}

double DepthImageToLaserScan::magnitude_of_ray(const cv::Point3d& ray) const{
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}

double DepthImageToLaserScan::angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const{
  double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
}

bool DepthImageToLaserScan::use_point(const float new_value, const float old_value, const float range_min, const float range_max) const{  
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);
  
  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }
  
  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return false;
  }
  
  if(!old_finite){ // New value is in range and finite, use it.
    return true;
  }
  
  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  return shorter_check;
}

sensor_msgs::LaserScanPtr DepthImageToLaserScan::convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
	      const sensor_msgs::CameraInfoConstPtr& info_msg){
  // Set camera model
  cam_model_.fromCameraInfo(info_msg);
  
  // Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
  cv::Point2d raw_pixel_left(0, cam_model_.cy());
  cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);
  
  cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_.cy());
  cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);
  
  cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
  cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);
  
  double angle_max = angle_between_rays(left_ray, center_ray);
  double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image
  
  // Fill in laserscan message
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
  scan_msg->header = depth_msg->header;
  if(output_frame_id_.length() > 0){
    scan_msg->header.frame_id = output_frame_id_;
  }
  scan_msg->angle_min = angle_min;
  scan_msg->angle_max = angle_max;
  scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;
  
  // Check scan_height vs image_height
  if(scan_height_/2 > cam_model_.cy() || scan_height_/2 > depth_msg->height - cam_model_.cy()){
    std::stringstream ss;
    ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
    throw std::runtime_error(ss.str());
  }

  // Calculate and fill the ranges
  uint32_t ranges_size = depth_msg->width;
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());
  
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    convert<uint16_t>(depth_msg, cam_model_, scan_msg, scan_height_);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    convert<float>(depth_msg, cam_model_, scan_msg, scan_height_);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::BGR8)
  {
    convert<uint8_t>(depth_msg, cam_model_, scan_msg, scan_height_);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
  
  return scan_msg;
}

void DepthImageToLaserScan::set_scan_time(const float scan_time){
  scan_time_ = scan_time;
}

void DepthImageToLaserScan::set_range_limits(const float range_min, const float range_max){
  range_min_ = range_min;
  range_max_ = range_max;
}

void DepthImageToLaserScan::set_scan_height(const int scan_height){
  scan_height_ = scan_height;
}

void DepthImageToLaserScan::set_output_frame(const std::string output_frame_id){
  output_frame_id_ = output_frame_id;
}

} // namespace depthimage_to_laserscan
