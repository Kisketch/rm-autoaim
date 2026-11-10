// Armor detection in the image: light bars -> light pairs -> corners -> number.
//
// Lineage:
//  - channel-difference binarisation and bar/pair filters: our assessment
//    solution (template/), tuned on these videos (centre of the bar is
//    over-exposed white, only the rim is red, so R-B beats HSV and gray).
//  - "no other light inside the pair" check, IPPE-friendly corner order and the
//    MLP number classifier: chenjunnn/rm_auto_aim (MIT).
//  - PCA symmetry-axis corner refinement: CSU-FYT-Vision/FYT2024_vision (Apache-2.0),
//    re-implemented.
#pragma once

#include <opencv2/dnn.hpp>

#include <memory>
#include <string>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/keypoint_detector.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

class NumberClassifier {
public:
    NumberClassifier(const std::string& model_path, const std::string& label_path,
                     double threshold, std::vector<std::string> ignore_classes);
    // Fills number_img / number / confidence and removes rejected armors.
    void classify(const cv::Mat& bgr, std::vector<ArmorDetection>& armors);
    bool ok() const { return !net_.empty(); }

private:
    cv::Mat extractNumber(const cv::Mat& bgr, const ArmorDetection& armor) const;
    cv::dnn::Net net_;
    std::vector<std::string> labels_;
    double threshold_;
    std::vector<std::string> ignore_;
};

struct DetectorDebug {
    cv::Mat mask;
    std::vector<Light> lights;
    int pairs_before_classify = 0;
};

class Detector {
public:
    explicit Detector(const DetectorConfig& cfg);
    std::vector<ArmorDetection> detect(const cv::Mat& bgr, DetectorDebug* debug = nullptr);
    // Windowed detection: run the light search only inside the given
    // windows (full-frame coordinates), then pair/classify as usual.
    // Callers pass windows around predicted target positions (tracking-
    // guided ROI); an empty window list is exactly full-frame detection.
    // Lights cut by a window border can be shortened or lost, so windows
    // need a margin around the prediction. With the ONNX keypoint backend
    // this falls back to full-frame detection.
    std::vector<ArmorDetection> detect(const cv::Mat& bgr, const std::vector<cv::Rect>& windows,
                                       DetectorDebug* debug = nullptr);

    const DetectorConfig& config() const { return cfg_; }
    // True when the ONNX keypoint backend is configured AND its model
    // loaded; false means detect() runs the traditional pipeline (either by
    // configuration, or as the fallback for an unloadable model).
    bool usingOnnx() const { return keypoint_ && keypoint_->ok(); }

private:
    // Returns false for frames detect() must treat as "no armors"
    // (empty/degenerate/unsupported type); converts grayscale to BGR.
    bool normalizeFrame(const cv::Mat& bgr, cv::Mat& frame) const;
    std::vector<Light> findLights(const cv::Mat& bgr, cv::Mat& mask) const;
    // findLights on a sub-image whose (0,0) sits at `offset` in the full
    // frame; returned lights are in full-frame coordinates.
    std::vector<Light> findLightsIn(const cv::Mat& bgr, cv::Point offset, cv::Mat& mask) const;
    std::vector<ArmorDetection> finishDetect(const cv::Mat& frame, std::vector<Light> lights,
                                             const cv::Mat& mask, DetectorDebug* debug);
    std::vector<ArmorDetection> matchLights(const std::vector<Light>& lights) const;
    bool containsOtherLight(const Light& a, const Light& b, const std::vector<Light>& lights) const;
    void refineCorners(const cv::Mat& gray, Light& light) const;

    DetectorConfig cfg_;
    NumberClassifier classifier_;
    std::unique_ptr<KeypointDetector> keypoint_;  // only when backend == "onnx"
};

Light makeLight(const cv::RotatedRect& rect, Color color);

}  // namespace autoaim
