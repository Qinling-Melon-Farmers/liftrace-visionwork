#pragma once
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>

namespace uav_vision {
// Color is an optional legacy segmentation aid, never proof of H identity.
// Every returned mask still requires the caller's independent H shape checks.
inline cv::Mat landingHMask(const cv::Mat& bgr, const std::string& method,
                           int saturation_max, int value_max,
                           double minimum_contrast) {
  cv::Mat mask;
  if (method == "legacy_hsv") {
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0,0,0),
                cv::Scalar(180,saturation_max,value_max), mask);
  } else if (method == "grayscale_otsu") {
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    const int dark = cv::countNonZero(mask);
    if (dark == 0 || dark == static_cast<int>(mask.total()) ||
        cv::mean(gray, ~mask)[0] - cv::mean(gray, mask)[0] < minimum_contrast)
      mask.setTo(0);
  } else {
    throw std::invalid_argument("Unknown landing_h_segmentation: " + method);
  }
  return mask;
}
}  // namespace uav_vision
