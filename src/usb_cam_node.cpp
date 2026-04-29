// Copyright 2014 Robert Bosch, LLC
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
//    * Neither the name of the Robert Bosch, LLC nor the names of its
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

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <chrono>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{
namespace
{

rmw_qos_profile_t sensor_data_qos_profile()
{
  return rclcpp::SensorDataQoS().get_rmw_qos_profile();
}

rclcpp::SensorDataQoS sensor_data_qos()
{
  return rclcpp::SensorDataQoS();
}

}  // namespace

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  m_camera(new usb_cam::UsbCam()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_compressed_img_msg(nullptr),
  m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
      sensor_data_qos_profile()))),
  m_compressed_image_publisher(nullptr),
  m_compressed_cam_info_publisher(nullptr),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
  m_capture_thread_running(false),
  m_service_capture(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("low_latency_mode", false);
  this->declare_parameter("buffer_count", 4);
  this->declare_parameter("startup_frame_drop_count", 0);
  this->declare_parameter("skip_device_check", false);  // allow bypassing V4L2 device list check

  get_params();
  init();
  m_parameters_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &UsbCamNode::parameters_callback, this,
      std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  stop_capture_worker();
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_publish_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    try {
      m_camera->start_capturing();
      warmup_capture();
      if (m_parameters.low_latency_mode) {
        start_capture_worker();
      } else {
        if (m_timer) {
          m_timer->reset();
        }
        if (m_publish_timer) {
          m_publish_timer->reset();
        }
      }
      response->success = true;
      response->message = "Start Capturing";
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start capture: " << exception.what());
    }
  } else {
    stop_capture_worker();
    try {
      m_camera->stop_capturing();
      response->success = true;
      response->message = "Stop Capturing";
    } catch (const std::exception & exception) {
      response->success = false;
      response->message = exception.what();
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to stop capture: " << exception.what());
    }
  }
}

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);

    // if the target path is relative, resolve it
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }

    return target_path.string();
  }
  return path;
}

void UsbCamNode::init()
{
  while (m_parameters.frame_id == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  m_camera_info.reset(
    new camera_info_manager::CameraInfoManager(
      this, m_parameters.camera_name, m_parameters.camera_info_url));
  // check for default camera info
  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  // Check if given device name is an available v4l2 device (unless skipped)
  if (!m_parameters.skip_device_check) {
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Device specified is not available or is not a vaild V4L2 device: `" <<
          m_parameters.device_name << "`"
      );
      RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
      for (const auto & device : available_devices) {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
        RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
      }
      rclcpp::shutdown();
      return;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Skipping V4L2 device availability check by request.");
  }

  // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
  // and publisher
  if (m_parameters.pixel_format_name == "mjpeg") {
    m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
    m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
    m_compressed_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
      std::string(BASE_TOPIC_NAME) + "/compressed", sensor_data_qos());
    m_compressed_cam_info_publisher =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", sensor_data_qos());
  }

  m_image_msg->header.frame_id = m_parameters.frame_id;

  try {
    RCLCPP_INFO(
      this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
      m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
      m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
      m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
    io_method_t io_method =
      usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
    if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
      throw std::invalid_argument("Unknown IO method `" + m_parameters.io_method_name + "`");
    }

    m_camera->configure(m_parameters, io_method);
    set_v4l2_params();

    m_camera->start();
    warmup_capture();

    auto frame_rate = std::max<size_t>(1, m_camera->get_frame_rate());
    if (static_cast<size_t>(m_parameters.framerate) > frame_rate) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Desired framerate " << m_parameters.framerate <<
          " is higher than the camera's capability " << frame_rate << " fps");
      m_parameters.framerate = frame_rate;
    }

    if (m_parameters.low_latency_mode) {
      RCLCPP_INFO(this->get_logger(), "Low latency mode enabled: using dedicated capture thread");
      start_capture_worker();
    } else {
      const int period_ms = std::max<int>(1, static_cast<int>(1000.0 / frame_rate));
      m_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
        std::bind(&UsbCamNode::update, this));
      RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");

      const int publish_period_ms = std::max<int>(1, static_cast<int>(1000.0 / m_parameters.framerate));
      m_publish_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(publish_period_ms)),
        std::bind(&UsbCamNode::publish, this));
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Camera initialization failed: " << exception.what());
    rclcpp::shutdown();
  }
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus", "low_latency_mode", "buffer_count",
      "startup_frame_drop_count", "skip_device_check"
    }
  );

  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      m_parameters.pixel_format_name = parameter.value_to_string();
    } else if (parameter.get_name() == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    } else if (parameter.get_name() == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (parameter.get_name() == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (parameter.get_name() == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (parameter.get_name() == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (parameter.get_name() == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (parameter.get_name() == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (parameter.get_name() == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (parameter.get_name() == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (parameter.get_name() == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (parameter.get_name() == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (parameter.get_name() == "focus") {
      m_parameters.focus = parameter.as_int();
    } else if (parameter.get_name() == "low_latency_mode") {
      m_parameters.low_latency_mode = parameter.as_bool();
    } else if (parameter.get_name() == "buffer_count") {
      m_parameters.buffer_count = parameter.as_int();
    } else if (parameter.get_name() == "startup_frame_drop_count") {
      m_parameters.startup_frame_drop_count = parameter.as_int();
    } else if (parameter.get_name() == "skip_device_check") {
      m_parameters.skip_device_check = parameter.as_bool();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void UsbCamNode::set_v4l2_params()
{
  // set camera parameters
  if (m_parameters.brightness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
  }

  if (m_parameters.contrast >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
  }

  if (m_parameters.saturation >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
  }

  if (m_parameters.sharpness >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
  }

  if (m_parameters.gain >= 0) {
    RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
  }

  // check auto white balance
  if (m_parameters.auto_white_balance) {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("exposure_auto", 1);
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("exposure_auto", 3);
  }

  // check auto focus
  if (m_parameters.autofocus) {
    m_camera->set_auto_focus(1);
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_auto", 1);
  } else {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_auto", 0);
    if (m_parameters.focus >= 0) {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
    }
  }
}

bool UsbCamNode::take_and_send_image()
{
  // Only resize if required
  if (m_image_msg->data.size() != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      // Some formats don't have a linesize specified by v4l2
      // Fall back to manually calculating it step = size / height
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  if (!m_camera->get_image(
      reinterpret_cast<char *>(&m_image_msg->data[0]),
      m_parameters.low_latency_mode))
  {
    return false;
  }

  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool UsbCamNode::take_and_send_image_mjpeg()
{
  // Only resize if required
  if (m_compressed_img_msg->data.size() != m_camera->get_image_size_in_bytes()) {
    m_compressed_img_msg->format = "jpeg";
    m_compressed_img_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  if (!m_camera->get_image(
      reinterpret_cast<char *>(&m_compressed_img_msg->data[0]),
      m_parameters.low_latency_mode))
  {
    return false;
  }

  auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;

  return true;
}

rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
  if (m_timer) {
    m_timer->reset();
  }
  if (m_publish_timer) {
    m_publish_timer->reset();
  }
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::warmup_capture()
{
  const int frames_to_drop = std::max(0, m_parameters.startup_frame_drop_count);

  for (int frame_index = 0; frame_index < frames_to_drop; ++frame_index) {
    const bool captured =
      (m_parameters.pixel_format_name == "mjpeg") ? take_and_send_image_mjpeg() : take_and_send_image();
    if (!captured) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Warmup frame capture did not complete in time.");
      return;
    }
  }
}

void UsbCamNode::update()
{
  if (!m_camera->is_capturing()) {
    return;
  }

  try {
    const bool is_successful = capture_frame();
    if (!is_successful) {
      RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Capture failed: " << exception.what());
    restart_capture_stream(exception.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Capture failed with an unknown exception");
    restart_capture_stream("unknown capture error");
  }
}

void UsbCamNode::publish()
{
  if (m_parameters.pixel_format_name == "mjpeg") {
    m_compressed_image_publisher->publish(*m_compressed_img_msg);
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  } else {
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  }
}

bool UsbCamNode::capture_frame()
{
  const bool is_successful =
    (m_parameters.pixel_format_name == "mjpeg") ? take_and_send_image_mjpeg() : take_and_send_image();

  if (is_successful && m_parameters.low_latency_mode) {
    publish();
  }

  return is_successful;
}

void UsbCamNode::start_capture_worker()
{
  if (!m_parameters.low_latency_mode || m_capture_thread_running.exchange(true)) {
    return;
  }

  m_capture_thread = std::thread(&UsbCamNode::capture_loop, this);
}

void UsbCamNode::stop_capture_worker()
{
  if (!m_capture_thread_running.exchange(false)) {
    return;
  }

  if (m_capture_thread.joinable()) {
    m_capture_thread.join();
  }
}

void UsbCamNode::capture_loop()
{
  while (m_capture_thread_running.load() && rclcpp::ok()) {
    if (!m_camera->is_capturing()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    try {
      const bool is_successful = capture_frame();
      if (!is_successful) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Low-latency capture loop failed: " << exception.what());
      restart_capture_stream(exception.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Low-latency capture loop failed with an unknown exception");
      restart_capture_stream("unknown low-latency capture error");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void UsbCamNode::restart_capture_stream(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(m_capture_mutex);
  RCLCPP_WARN_STREAM(this->get_logger(), "Restarting capture stream after error: " << reason);

  try {
    if (m_camera->is_capturing()) {
      m_camera->stop_capturing();
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to stop capture during recovery: " << exception.what());
  }

  try {
    m_camera->start_capturing();
    warmup_capture();
  } catch (const std::exception & exception) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to restart capture stream: " << exception.what());
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
