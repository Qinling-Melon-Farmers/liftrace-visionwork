#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>

namespace uav_vision {

struct HStrokeObservation {
  cv::Point2f center;
  cv::Rect bbox;
  double area = 0.0;
};

inline std::vector<cv::Range> strokeRuns(const std::vector<bool>& values,
                                        int minimum_length) {
  std::vector<cv::Range> result;
  int begin = -1;
  for (int i = 0; i <= static_cast<int>(values.size()); ++i) {
    const bool active = i < static_cast<int>(values.size()) && values[i];
    if (active && begin < 0) begin = i;
    if (!active && begin >= 0) {
      if (i - begin >= minimum_length) result.emplace_back(begin, i);
      begin = -1;
    }
  }
  return result;
}

// Recognize the two parallel strokes and one connecting stroke of H.
// The outside ring and the outer ends of the long strokes may be cropped;
// both inner notches and the transverse stroke must still be visible.
// Center comes from INNER stroke edges, not the center of a cropped bbox.
inline bool detectHStrokes(const cv::Mat& dark_mask, double min_size_px,
                           HStrokeObservation& best) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(dark_mask.clone(), contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  best.area = 0.0;
  for (const auto& contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < min_size_px * min_size_px * 0.5) continue;
    const auto box = cv::minAreaRect(contour);
    const double small = std::min(box.size.width, box.size.height);
    const double large = std::max(box.size.width, box.size.height);
    if (small < min_size_px || small / large < 0.35 ||
        area / (small * large) > 0.90) continue;

    cv::Point2f corners[4]; box.points(corners);
    cv::Mat component = cv::Mat::zeros(dark_mask.size(), CV_8UC1);
    cv::drawContours(component, std::vector<std::vector<cv::Point>>{contour},
                     0, cv::Scalar(255), cv::FILLED);
    for (int orientation = 0; orientation < 2; ++orientation) {
      cv::Point2f source[4], destination[4] = {{0,0},{127,0},{127,127},{0,127}};
      for (int i = 0; i < 4; ++i) source[i] = corners[(i - orientation + 4) % 4];
      const cv::Mat transform = cv::getPerspectiveTransform(source, destination);
      cv::Mat normalized;
      cv::warpPerspective(component, normalized, transform, cv::Size(128,128), cv::INTER_NEAREST);
      std::vector<bool> long_strokes(128);
      for (int col = 0; col < 128; ++col)
        long_strokes[col] = cv::countNonZero(normalized.col(col)) >= 90;
      const auto sides = strokeRuns(long_strokes, 5);
      if (sides.size() != 2 || sides[0].start > 15 || sides[1].end < 113) continue;
      const int gap = sides[1].start - sides[0].end;
      if (gap < 25 || gap > 100) continue;
      const int lo = sides[0].end + 3, hi = sides[1].start - 3;
      std::vector<double> row_fill(128);
      std::vector<bool> connector(128);
      for (int row = 0; row < 128; ++row) {
        row_fill[row] = cv::countNonZero(normalized.row(row).colRange(lo,hi)) /
                        static_cast<double>(hi - lo);
        connector[row] = row_fill[row] > 0.85;
      }
      const auto bars = strokeRuns(connector, 5);
      if (bars.size() != 1) continue;
      const int top = bars[0].start, bottom = bars[0].end;
      if (top < 8 || bottom > 120 || bottom-top < 8 || bottom-top > 72) continue;
      double upper = 0.0, lower = 0.0;
      for (int row = 0; row < top-2; ++row) upper += row_fill[row];
      for (int row = bottom+2; row < 128; ++row) lower += row_fill[row];
      if (upper / (top-2) > 0.20 || lower / (126-bottom) > 0.20) continue;
      if (area <= best.area) continue;
      std::vector<cv::Point2f> center{{
          0.5f * (sides[0].end + sides[1].start - 1),
          0.5f * (top + bottom - 1)}};
      cv::perspectiveTransform(center, center, transform.inv());
      best.center = center.front();
      best.bbox = cv::boundingRect(contour);
      best.area = area;
    }
  }
  return best.area > 0.0;
}

}  // namespace uav_vision
