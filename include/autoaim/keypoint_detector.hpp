// YOLO-pose keypoint armor detector (B-02, inference side).
//
// Implements the integration design of docs/yolo_keypoints_plan.md: a
// YOLOv8/v11-pose ONNX model (4 keypoints + class) run through OpenCV DNN,
// producing the same ArmorDetection the traditional light-bar pipeline
// produces, so pose/tracker/aimer call sites do not change.  Training is
// out of scope here (needs a GPU machine and the dataset, see the plan).
//
// Model contract: input 640x640 letterboxed BGR->RGB, output tensor
// [1, 4 + C + 12, N] or [1, N, 4 + C + 12] with per-anchor
// (cx, cy, w, h, class scores..., (x, y, conf) x 4 keypoints).
#pragma once

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

struct KeypointCandidate {
    cv::Rect2f box;  // image coordinates
    float score = 0;
    int cls = -1;
    std::array<cv::Point2f, 4> kpts{};  // image coordinates, model order
    std::array<float, 4> kpt_conf{};
};

struct Letterbox {
    float scale = 1, pad_x = 0, pad_y = 0;
};
Letterbox letterboxFor(int img_w, int img_h, int input_size);

// Parse a raw pose-model output tensor into NMS-filtered candidates in
// image coordinates.  Free function so it can be unit-tested with synthetic
// tensors, without a trained model.
std::vector<KeypointCandidate> parseKeypointOutput(const cv::Mat& output, int num_classes,
                                                   float conf_thr, float nms_thr, int input_size,
                                                   int img_w, int img_h);

// Order 4 keypoints as the ArmorDetection contract requires:
// 0 = left bottom, 1 = left top, 2 = right top, 3 = right bottom.
// Geometric split (two smallest-x points form the left edge); this assumes
// the plate is not rolled past ~45 deg, which the gimbal geometry guarantees
// in practice.
std::array<cv::Point2f, 4> orderArmorCorners(const std::array<cv::Point2f, 4>& kpts);

// Build a full ArmorDetection from a candidate: corners ordered, lights
// synthesised from the left/right keypoint edges, plate size class from the
// quad aspect ratio (same large_plate_ratio config as the traditional path).
ArmorDetection candidateToArmor(const KeypointCandidate& c, const DetectorConfig& cfg);

class KeypointDetector {
public:
    explicit KeypointDetector(const DetectorConfig& cfg);
    bool ok() const { return !net_.empty(); }
    std::vector<ArmorDetection> detect(const cv::Mat& bgr);

private:
    DetectorConfig cfg_;
    cv::dnn::Net net_;
};

}  // namespace autoaim
