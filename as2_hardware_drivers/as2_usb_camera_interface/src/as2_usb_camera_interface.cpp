// Copyright 2024 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file as2_usb_camera_interface.hpp
*
* @brief Main for the USB camera interface node
*
* @authors David Perez Saura, Miguel Fernandez Cortizas
*/

#include "as2_usb_camera_interface.hpp"

namespace usb_camera_interface
{

UsbCameraInterface::UsbCameraInterface(const rclcpp::NodeOptions & options)
: as2::Node("usb_camera_interface", options)
{
  // Initialize camera
  camera_ = std::make_shared<as2::sensors::Camera>(this);

  // Setup camera
  setupCamera();

  // create timer for image capture
  static auto image_capture_timer_ = this->create_timer(
    std::chrono::milliseconds(10), std::bind(&UsbCameraInterface::captureImage, this));
}

void UsbCameraInterface::setupCamera()
{
  bool arducam = false;
  std::string device_port;
  double framerate;
  int image_width;
  int image_height;

  if (!this->has_parameter("arducam")) {
    this->declare_parameter<bool>("arducam");
  }
  if (!this->has_parameter("device")) {
    this->declare_parameter<std::string>("device");
  }
  if (!this->has_parameter("framerate")) {
    this->declare_parameter<double>("framerate");
  }
  if (!this->has_parameter("image_height")) {
    this->declare_parameter<int>("image_height");
  }
  if (!this->has_parameter("image_width")) {
    this->declare_parameter<int>("image_width");
  }

  this->get_parameter("arducam", arducam);
  this->get_parameter("device", device_port);
  this->get_parameter("framerate", framerate);
  this->get_parameter("image_height", image_height);
  this->get_parameter("image_width", image_width);

  RCLCPP_INFO(this->get_logger(), "Video device: %s", device_port.c_str());

  if (arducam) {
    RCLCPP_INFO(this->get_logger(), "Using RPi5 CSI camera via V4L2");

    cap_.open(device_port, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Cannot open device");
      return;
    }
    
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('B', 'G', 'R', '3'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    cap_.set(cv::CAP_PROP_FPS, framerate);

  } else {
    cap_.open(device_port);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Cannot open device");
      return;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height);
    cap_.set(cv::CAP_PROP_FPS, framerate);
  }

  RCLCPP_INFO(get_logger(), "Camera capture setup complete");
}

void UsbCameraInterface::captureImage()
{
  // Capture image in device with opencv2
  cv::Mat frame;
  if (!cap_.read(frame)) {
    RCLCPP_ERROR(get_logger(), "Cannot read image");
    return;
  }

  camera_->updateData(frame);
}

}  // namespace usb_camera_interface
