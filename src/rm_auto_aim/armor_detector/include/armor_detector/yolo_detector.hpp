#ifndef ARMOR_DETECTOR_YOLO_DETECTOR_HPP_
#define ARMOR_DETECTOR_YOLO_DETECTOR_HPP_

#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "armor_detector/detector_base.hpp"
#include "rm_utils/common.hpp"

namespace fyt::auto_aim {

class YoloDetector : public DetectorBase {
public:
  struct ClassInfo {
    EnemyColor color;
    std::string number;
    ArmorType type;
  };

  YoloDetector(const std::string &model_path,
               const std::string &device,
               float score_threshold,
               float nms_threshold,
               bool use_traditional_refine,
               EnemyColor detect_color,
               bool debug = false);

  std::vector<Armor> detect(const cv::Mat &input) noexcept override;
  cv::Mat getAllNumbersImage() const noexcept override;
  void drawResults(cv::Mat &img) const noexcept override;
  const cv::Mat &binaryImage() const noexcept override { return binary_img_; }
  rm_interfaces::msg::DebugLights &debugLights() noexcept override { return debug_lights_; }
  rm_interfaces::msg::DebugArmors &debugArmors() noexcept override { return debug_armors_; }
  void applyParameter(const rclcpp::Parameter &param) noexcept override;

private:
  void sortCorners(std::array<cv::Point2f, 4> &corners) const noexcept;
  bool decodeClass(int class_id, ClassInfo &info) const noexcept;

private:
  ov::Core core_;
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;

  std::string model_path_;
  std::string device_;
  float score_threshold_;
  float nms_threshold_;
  bool use_traditional_refine_;
  EnemyColor detect_color_;
  bool debug_;

  cv::Mat input_canvas_;
  cv::Mat binary_img_;
  rm_interfaces::msg::DebugLights debug_lights_;
  rm_interfaces::msg::DebugArmors debug_armors_;
  std::vector<Armor> armors_;
};

}  // namespace fyt::auto_aim

#endif
