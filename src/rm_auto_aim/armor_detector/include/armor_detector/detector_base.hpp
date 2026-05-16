#ifndef ARMOR_DETECTOR_DETECTOR_BASE_HPP_
#define ARMOR_DETECTOR_DETECTOR_BASE_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/parameter.hpp>
#include <vector>

#include "armor_detector/types.hpp"
#include "rm_interfaces/msg/debug_armors.hpp"
#include "rm_interfaces/msg/debug_lights.hpp"

namespace fyt::auto_aim {

class DetectorBase {
public:
  virtual ~DetectorBase() = default;

  virtual std::vector<Armor> detect(const cv::Mat &input) noexcept = 0;
  virtual cv::Mat getAllNumbersImage() const noexcept = 0;
  virtual void drawResults(cv::Mat &img) const noexcept = 0;
  virtual const cv::Mat &binaryImage() const noexcept = 0;
  virtual rm_interfaces::msg::DebugLights &debugLights() noexcept = 0;
  virtual rm_interfaces::msg::DebugArmors &debugArmors() noexcept = 0;
  virtual void applyParameter(const rclcpp::Parameter &param) noexcept = 0;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_DETECTOR_DETECTOR_BASE_HPP_
