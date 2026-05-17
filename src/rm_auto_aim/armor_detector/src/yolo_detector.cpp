#include "armor_detector/yolo_detector.hpp"

#include <algorithm>
#include <array>
#include <filesystem>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace fyt::auto_aim {

namespace {
constexpr int kInputSize = 640;
constexpr int kClassCount = 38;

const std::array<YoloDetector::ClassInfo, kClassCount> kClassMap = {{
    {EnemyColor::BLUE, "sentry", ArmorType::SMALL},
    {EnemyColor::RED, "sentry", ArmorType::SMALL},
    {EnemyColor::WHITE, "sentry", ArmorType::SMALL},
    {EnemyColor::BLUE, "1", ArmorType::LARGE},
    {EnemyColor::RED, "1", ArmorType::LARGE},
    {EnemyColor::WHITE, "1", ArmorType::LARGE},
    {EnemyColor::BLUE, "2", ArmorType::SMALL},
    {EnemyColor::RED, "2", ArmorType::SMALL},
    {EnemyColor::WHITE, "2", ArmorType::SMALL},
    {EnemyColor::BLUE, "3", ArmorType::SMALL},
    {EnemyColor::RED, "3", ArmorType::SMALL},
    {EnemyColor::WHITE, "3", ArmorType::SMALL},
    {EnemyColor::BLUE, "4", ArmorType::SMALL},
    {EnemyColor::RED, "4", ArmorType::SMALL},
    {EnemyColor::WHITE, "4", ArmorType::SMALL},
    {EnemyColor::BLUE, "5", ArmorType::SMALL},
    {EnemyColor::RED, "5", ArmorType::SMALL},
    {EnemyColor::WHITE, "5", ArmorType::SMALL},
    {EnemyColor::BLUE, "outpost", ArmorType::SMALL},
    {EnemyColor::RED, "outpost", ArmorType::SMALL},
    {EnemyColor::WHITE, "outpost", ArmorType::SMALL},
    {EnemyColor::BLUE, "base", ArmorType::LARGE},
    {EnemyColor::RED, "base", ArmorType::LARGE},
    {EnemyColor::WHITE, "base", ArmorType::LARGE},
    {EnemyColor::WHITE, "base", ArmorType::LARGE},
    {EnemyColor::BLUE, "base", ArmorType::SMALL},
    {EnemyColor::RED, "base", ArmorType::SMALL},
    {EnemyColor::WHITE, "base", ArmorType::SMALL},
    {EnemyColor::WHITE, "base", ArmorType::SMALL},
    {EnemyColor::BLUE, "3", ArmorType::LARGE},
    {EnemyColor::RED, "3", ArmorType::LARGE},
    {EnemyColor::WHITE, "3", ArmorType::LARGE},
    {EnemyColor::BLUE, "4", ArmorType::LARGE},
    {EnemyColor::RED, "4", ArmorType::LARGE},
    {EnemyColor::WHITE, "4", ArmorType::LARGE},
    {EnemyColor::BLUE, "5", ArmorType::LARGE},
    {EnemyColor::RED, "5", ArmorType::LARGE},
    {EnemyColor::WHITE, "5", ArmorType::LARGE},
}};

float computeRectangularError(const std::array<cv::Point2f, 4> &corners) {
  const cv::Point2f left_center = (corners[0] + corners[1]) * 0.5f;
  const cv::Point2f right_center = (corners[2] + corners[3]) * 0.5f;
  const cv::Point2f left_to_right = right_center - left_center;
  const float roll = std::atan2(left_to_right.y, left_to_right.x);

  const float left_error = std::abs(
      std::atan2((corners[0] - corners[1]).y, (corners[0] - corners[1]).x) -
      roll - static_cast<float>(CV_PI / 2.0));
  const float right_error = std::abs(
      std::atan2((corners[3] - corners[2]).y, (corners[3] - corners[2]).x) -
      roll - static_cast<float>(CV_PI / 2.0));
  return std::max(left_error, right_error);
}

bool isGeometryValid(const Armor &armor) {
  const float left_edge = cv::norm(armor.corners[0] - armor.corners[1]);
  const float right_edge = cv::norm(armor.corners[2] - armor.corners[3]);
  const float top_edge = cv::norm(armor.corners[1] - armor.corners[2]);
  const float bottom_edge = cv::norm(armor.corners[0] - armor.corners[3]);

  const float max_width = std::max(left_edge, right_edge);
  const float max_length = std::max(top_edge, bottom_edge);
  if (max_width < 4.0f || max_length < 6.0f) {
    return false;
  }

  const float ratio = max_length / std::max(1.0f, max_width);
  if (ratio < 0.8f || ratio > 8.0f) {
    return false;
  }

  return armor.rectangular_error < 1.0f;
}
}  // namespace

YoloDetector::YoloDetector(const std::string &model_path,
                           const std::string &device,
                           float score_threshold,
                           float nms_threshold,
                           bool use_traditional_refine,
                           EnemyColor detect_color,
                           bool debug)
: model_path_(model_path),
  device_(device),
  score_threshold_(score_threshold),
  nms_threshold_(nms_threshold),
  use_traditional_refine_(use_traditional_refine),
  detect_color_(detect_color),
  debug_(debug) {
  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error("YOLO model not found: " + model_path_);
  }

  auto model = core_.read_model(model_path_);
  ov::preprocess::PrePostProcessor ppp(model);
  auto &input = ppp.input();

  input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, kInputSize, kInputSize, 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::RGB);

  input.model().set_layout("NCHW");
  input.preprocess().convert_element_type(ov::element::f32).scale(255.0f);

  model = ppp.build();
  compiled_model_ =
      core_.compile_model(model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  infer_request_ = compiled_model_.create_infer_request();
  input_canvas_.create(kInputSize, kInputSize, CV_8UC3);
}

bool YoloDetector::decodeClass(int class_id, ClassInfo &info) const noexcept {
  if (class_id < 0 || class_id >= static_cast<int>(kClassMap.size())) {
    return false;
  }
  info = kClassMap[static_cast<size_t>(class_id)];
  return info.color != EnemyColor::WHITE && info.color == detect_color_;
}

void YoloDetector::sortCorners(std::array<cv::Point2f, 4> &corners) const noexcept {
  std::array<cv::Point2f, 2> top_points{};
  std::array<cv::Point2f, 2> bottom_points{};
  std::sort(corners.begin(), corners.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
  top_points[0] = corners[0];
  top_points[1] = corners[1];
  bottom_points[0] = corners[2];
  bottom_points[1] = corners[3];
  std::sort(top_points.begin(), top_points.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return a.x < b.x; });
  std::sort(bottom_points.begin(), bottom_points.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return a.x < b.x; });

  // landmarks() expects bottom-left, top-left, top-right, bottom-right
  corners[0] = bottom_points[0];
  corners[1] = top_points[0];
  corners[2] = top_points[1];
  corners[3] = bottom_points[1];
}

std::vector<Armor> YoloDetector::detect(const cv::Mat &input) noexcept {
  armors_.clear();
  debug_lights_.data.clear();
  debug_armors_.data.clear();
  binary_img_.release();

  if (input.empty()) {
    return {};
  }

  const double x_scale = static_cast<double>(kInputSize) / input.rows;
  const double y_scale = static_cast<double>(kInputSize) / input.cols;
  const double scale = std::min(x_scale, y_scale);
  const int h = static_cast<int>(input.rows * scale);
  const int w = static_cast<int>(input.cols * scale);

  input_canvas_.setTo(cv::Scalar(0, 0, 0));
  cv::resize(input, input_canvas_(cv::Rect(0, 0, w, h)), {w, h});
  ov::Tensor input_tensor(
      ov::element::u8, {1, kInputSize, kInputSize, 3}, input_canvas_.data);

  infer_request_.set_input_tensor(input_tensor);
  infer_request_.infer();

  auto output_tensor = infer_request_.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  cv::transpose(output, output);

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::array<cv::Point2f, 4>> corners_list;
  class_ids.reserve(output.rows);
  confidences.reserve(output.rows);
  boxes.reserve(output.rows);
  corners_list.reserve(output.rows);

  for (int r = 0; r < output.rows; ++r) {
    auto xywh = output.row(r).colRange(0, 4);
    auto scores = output.row(r).colRange(4, 4 + kClassCount);
    auto one_keypoints = output.row(r).colRange(4 + kClassCount, 50);

    double score = 0.0;
    cv::Point max_point;
    cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);
    if (score < score_threshold_) {
      continue;
    }

    ClassInfo class_info{};
    if (!decodeClass(max_point.x, class_info)) {
      continue;
    }

    float cx = xywh.at<float>(0);
    float cy = xywh.at<float>(1);
    float bw = xywh.at<float>(2);
    float bh = xywh.at<float>(3);
    int left = static_cast<int>((cx - 0.5f * bw) / scale);
    int top = static_cast<int>((cy - 0.5f * bh) / scale);
    int width = static_cast<int>(bw / scale);
    int height = static_cast<int>(bh / scale);

    std::array<cv::Point2f, 4> corners{};
    for (int i = 0; i < 4; ++i) {
      corners[static_cast<size_t>(i)] = {
          one_keypoints.at<float>(0, i * 2) / static_cast<float>(scale),
          one_keypoints.at<float>(0, i * 2 + 1) / static_cast<float>(scale)};
    }
    sortCorners(corners);

    class_ids.push_back(max_point.x);
    confidences.push_back(static_cast<float>(score));
    boxes.emplace_back(left, top, width, height);
    corners_list.push_back(corners);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  for (int idx : indices) {
    ClassInfo class_info{};
    if (!decodeClass(class_ids[idx], class_info)) {
      continue;
    }

    Armor armor;
    armor.type = class_info.type;
    armor.number = class_info.number;
    armor.color = class_info.color;
    armor.confidence = confidences[idx];
    armor.bbox = boxes[idx];
    armor.has_corners = true;
    armor.corners = corners_list[idx];
    armor.center = (armor.corners[0] + armor.corners[1] + armor.corners[2] + armor.corners[3]) * 0.25f;
    armor.classfication_result = class_info.number;
    armor.rank = armorRankFromString(armor.number);
    armor.center_norm = {armor.center.x / input.cols, armor.center.y / input.rows};
    armor.rectangular_error = computeRectangularError(armor.corners);
    armor.is_valid = armor.confidence >= score_threshold_ && isGeometryValid(armor);

    if (!armor.is_valid) {
      continue;
    }

    rm_interfaces::msg::DebugArmor debug_armor;
    debug_armor.type = armorTypeToString(armor.type);
    debug_armor.center_x = armor.center.x;
    debug_armor.light_ratio = 1.0;
    debug_armor.center_distance = 1.0;
    debug_armor.angle = armor.rectangular_error;
    debug_armors_.data.emplace_back(debug_armor);
    armors_.emplace_back(std::move(armor));
  }

  std::sort(armors_.begin(), armors_.end(), [](const Armor &a, const Armor &b) {
    if (a.rank != b.rank) {
      return a.rank < b.rank;
    }
    return a.center_norm.y < b.center_norm.y;
  });

  return armors_;
}

cv::Mat YoloDetector::getAllNumbersImage() const noexcept {
  return cv::Mat(cv::Size(20, 28), CV_8UC1);
}

void YoloDetector::drawResults(cv::Mat &img) const noexcept {
  for (const auto &armor : armors_) {
    if (!armor.has_corners) {
      continue;
    }
    const auto &c = armor.corners;
    cv::line(img, c[0], c[1], cv::Scalar(0, 255, 0), 2);
    cv::line(img, c[1], c[2], cv::Scalar(0, 255, 0), 2);
    cv::line(img, c[2], c[3], cv::Scalar(0, 255, 0), 2);
    cv::line(img, c[3], c[0], cv::Scalar(0, 255, 0), 2);

    const std::string text =
      armorTypeToString(armor.type) + " " + armor.number + " " +
      cv::format("%.2f", armor.confidence);

    const int text_x = std::clamp(armor.bbox.x, 0, std::max(0, img.cols - 180));
    const int text_y = std::clamp(armor.bbox.y - 8, 20, std::max(20, img.rows - 5));
    const cv::Point text_org(text_x, text_y);

    cv::putText(img, text, text_org, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
  }
}

void YoloDetector::applyParameter(const rclcpp::Parameter &param) noexcept {
  if (param.get_name() == "yolo.score_threshold") {
    score_threshold_ = static_cast<float>(param.as_double());
  } else if (param.get_name() == "yolo.nms_threshold") {
    nms_threshold_ = static_cast<float>(param.as_double());
  }
}

}  // namespace fyt::auto_aim
