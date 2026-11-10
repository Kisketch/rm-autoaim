#include "autoaim/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace autoaim {
namespace {

float longAxisTiltDeg(const cv::Point2f& top, const cv::Point2f& bottom) {
    return static_cast<float>(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) *
                              180.0 / kPi);
}

uchar sampleGray(const cv::Mat& gray, float x, float y) {
    const int xi = cvRound(x), yi = cvRound(y);
    if (xi < 0 || yi < 0 || xi >= gray.cols || yi >= gray.rows) return 0;
    return gray.at<uchar>(yi, xi);
}

}  // namespace

Light makeLight(const cv::RotatedRect& rect, Color color) {
    Light l;
    l.rect = rect;
    l.color = color;
    cv::Point2f p[4];
    rect.points(p);
    std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    l.top = (p[0] + p[1]) * 0.5f;
    l.bottom = (p[2] + p[3]) * 0.5f;
    l.center = (l.top + l.bottom) * 0.5f;
    l.length = static_cast<float>(cv::norm(l.top - l.bottom));
    l.width = static_cast<float>(cv::norm(p[0] - p[1]));
    l.tilt_deg = longAxisTiltDeg(l.top, l.bottom);
    return l;
}

// ---------------------------------------------------------------- classifier

NumberClassifier::NumberClassifier(const std::string& model_path, const std::string& label_path,
                                   double threshold, std::vector<std::string> ignore_classes)
    : threshold_(threshold), ignore_(std::move(ignore_classes)) {
    if (!model_path.empty()) net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream in(label_path);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) labels_.push_back(line);
    }
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat& bgr, const ArmorDetection& armor) const {
    // Same warp as the reference implementation the model was trained with:
    // bars of 12 px inside a 28 px tall strip, 32 px (small) / 52 px (large) wide.
    constexpr int kLightLen = 12, kWarpH = 28, kSmallW = 32, kLargeW = 52;
    const int top_y = (kWarpH - kLightLen) / 2 - 1;
    const int bottom_y = top_y + kLightLen;
    const int warp_w = armor.type == ArmorType::SMALL ? kSmallW : kLargeW;
    const cv::Point2f src[4] = {armor.corners[0], armor.corners[1], armor.corners[2],
                                armor.corners[3]};
    const cv::Point2f dst[4] = {{0.f, float(bottom_y)}, {0.f, float(top_y)},
                                {float(warp_w - 1), float(top_y)},
                                {float(warp_w - 1), float(bottom_y)}};
    cv::Mat warped;
    cv::warpPerspective(bgr, warped, cv::getPerspectiveTransform(src, dst), {warp_w, kWarpH});
    cv::Mat roi = warped(cv::Rect((warp_w - 28) / 2, 0, 28, 28)).clone();
    // NOTE: the template (and the model's training pipeline) used RGB2GRAY on
    // BGR frames. Kept identical on purpose; swapping it changes the input
    // distribution the MLP was trained on.
    cv::cvtColor(roi, roi, cv::COLOR_RGB2GRAY);
    cv::adaptiveThreshold(roi, roi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 19, -3);
    return roi;
}

void NumberClassifier::classify(const cv::Mat& bgr, std::vector<ArmorDetection>& armors) {
    if (net_.empty()) {
        for (auto& a : armors) a.number = "unknown";
        return;
    }
    for (auto& armor : armors) {
        armor.number_img = extractNumber(bgr, armor);
        cv::Mat input = armor.number_img / 255.0;
        cv::Mat blob;
        cv::dnn::blobFromImage(input, blob);
        net_.setInput(blob);
        cv::Mat out = net_.forward().clone();
        const float max_v = *std::max_element(out.begin<float>(), out.end<float>());
        cv::Mat prob;
        cv::exp(out - max_v, prob);
        prob /= static_cast<float>(cv::sum(prob)[0]);
        double conf;
        cv::Point id;
        cv::minMaxLoc(prob.reshape(1, 1), nullptr, &conf, nullptr, &id);
        armor.confidence = static_cast<float>(conf);
        armor.number = id.x < static_cast<int>(labels_.size()) ? labels_[id.x] : "negative";
    }
    armors.erase(std::remove_if(armors.begin(), armors.end(),
                                [this](const ArmorDetection& a) {
                                    if (a.confidence < threshold_) return true;
                                    for (const auto& ig : ignore_)
                                        if (a.number == ig) return true;
                                    // rm_auto_aim type/number consistency rule
                                    if (a.type == ArmorType::LARGE)
                                        return a.number == "outpost" || a.number == "2" ||
                                               a.number == "guard";
                                    return a.number == "1" || a.number == "base";
                                }),
                 armors.end());
}

// ---------------------------------------------------------------- detector

Detector::Detector(const DetectorConfig& cfg)
    : cfg_(cfg),
      classifier_(cfg.model_path, cfg.label_path, cfg.classify_threshold, cfg.ignore_classes) {
    if (cfg_.backend == "onnx") keypoint_ = std::make_unique<KeypointDetector>(cfg_);
}

std::vector<Light> Detector::findLights(const cv::Mat& bgr, cv::Mat& mask) const {
    std::vector<cv::Mat> ch;
    cv::split(bgr, ch);
    cv::Mat diff;
    if (cfg_.enemy_color == Color::RED)
        cv::subtract(ch[2], ch[0], diff);
    else
        cv::subtract(ch[0], ch[2], diff);
    double thr = cfg_.channel_diff_threshold;
    if (cfg_.channel_diff_adaptive) {
        cv::Scalar mean, stddev;
        cv::meanStdDev(diff, mean, stddev);
        // Only useful in dim venues: measured +21.7 pp under 0.6x brightness,
        // -1.1 pp on the nominal clips, so it stays off by default.
        thr = std::clamp(mean[0] + cfg_.channel_diff_k * stddev[0],
                         cfg_.channel_diff_min, cfg_.channel_diff_max);
    }
    cv::threshold(diff, mask, thr, 255, cv::THRESH_BINARY);
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k, {-1, -1}, 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<Light> lights;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < 12.0) continue;
        const cv::RotatedRect rect = cv::minAreaRect(c);
        const float w = rect.size.width, h = rect.size.height;
        if (w < 1.5f || h < 1.5f) continue;
        const float len = std::max(w, h), thick = std::min(w, h);
        if (len < cfg_.min_light_length) continue;
        const float aspect = len / thick;
        if (aspect < cfg_.min_light_aspect || aspect > cfg_.max_light_aspect) continue;
        if (area / (w * h) < cfg_.min_fill_ratio) continue;
        Light l = makeLight(rect, cfg_.enemy_color);
        if (l.tilt_deg > cfg_.max_light_tilt_deg) continue;
        lights.push_back(l);
    }
    if (lights.size() > 40) {
        std::partial_sort(lights.begin(), lights.begin() + 40, lights.end(),
                          [](const Light& a, const Light& b) { return a.length > b.length; });
        lights.resize(40);
    }
    return lights;
}

bool Detector::containsOtherLight(const Light& a, const Light& b,
                                  const std::vector<Light>& lights) const {
    const std::vector<cv::Point2f> pts{a.top, a.bottom, b.top, b.bottom};
    const cv::Rect box = cv::boundingRect(pts);
    for (const auto& l : lights) {
        if (l.center == a.center || l.center == b.center) continue;
        if (box.contains(l.top) || box.contains(l.bottom) || box.contains(l.center)) return true;
    }
    return false;
}

std::vector<ArmorDetection> Detector::matchLights(const std::vector<Light>& lights) const {
    struct Cand {
        ArmorDetection det;
        float score, width;
    };
    std::vector<Cand> cands;
    for (std::size_t i = 0; i < lights.size(); ++i) {
        for (std::size_t j = i + 1; j < lights.size(); ++j) {
            const Light* l = &lights[i];
            const Light* r = &lights[j];
            if (l->center.x > r->center.x) std::swap(l, r);

            const float angle_diff = std::abs(l->tilt_deg * (l->top.x < l->bottom.x ? 1 : -1) -
                                              r->tilt_deg * (r->top.x < r->bottom.x ? 1 : -1));
            if (angle_diff > cfg_.max_pair_angle_diff_deg) continue;
            const float lmax = std::max(l->length, r->length), lmin = std::min(l->length, r->length);
            if (lmin < 1.f || lmax / lmin > cfg_.max_length_ratio) continue;
            const float dx = r->center.x - l->center.x;
            const float dy = std::abs(r->center.y - l->center.y);
            const float avg = 0.5f * (l->length + r->length);
            if (dx <= 0.f || dy > cfg_.max_center_y_diff * avg) continue;
            const float thick = 0.5f * (l->width + r->width);
            const float gap = dx - thick;
            if (gap < cfg_.min_gap_ratio * avg || gap > cfg_.max_gap_ratio * avg) continue;
            const float plate_ratio = (dx + thick) / avg;
            if (plate_ratio < cfg_.min_plate_ratio || plate_ratio > cfg_.max_plate_ratio) continue;
            if (cfg_.reject_contained_lights && containsOtherLight(*l, *r, lights)) continue;

            ArmorDetection d;
            d.left = *l;
            d.right = *r;
            d.color = l->color;
            const float center_ratio = static_cast<float>(cv::norm(r->center - l->center)) / avg;
            d.type = center_ratio > cfg_.large_plate_ratio ? ArmorType::LARGE : ArmorType::SMALL;
            const float score = angle_diff / cfg_.max_pair_angle_diff_deg +
                                (lmax / lmin - 1.f) / (cfg_.max_length_ratio - 1.f) +
                                dy / (cfg_.max_center_y_diff * avg);
            cands.push_back({d, score, dx + thick});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.score < b.score; });
    std::vector<ArmorDetection> out;
    for (const auto& c : cands) {
        const cv::Point2f center = (c.det.left.center + c.det.right.center) * 0.5f;
        bool dup = false;
        for (const auto& k : out)
            if (cv::norm(k.center - center) < 0.5f * c.width) dup = true;
        // a light may belong to one armor only
        for (const auto& k : out)
            if (k.left.center == c.det.left.center || k.right.center == c.det.right.center ||
                k.left.center == c.det.right.center || k.right.center == c.det.left.center)
                dup = true;
        if (dup) continue;
        ArmorDetection d = c.det;
        d.center = center;
        out.push_back(d);
    }
    return out;
}

// PCA symmetry axis + brightness-edge search along it (idea from FYT2024).
void Detector::refineCorners(const cv::Mat& gray, Light& light) const {
    if (light.width < 2.5f || light.length < 8.f) return;
    cv::Rect box = light.rect.boundingRect();
    const int pad = std::max(2, cvRound(0.15f * light.length));
    box = cv::Rect(box.x - pad, box.y - pad, box.width + 2 * pad, box.height + 2 * pad) &
          cv::Rect(0, 0, gray.cols, gray.rows);
    if (box.area() < 9) return;
    const cv::Mat roi = gray(box);
    const double mean = cv::mean(roi)[0];

    // intensity-weighted centroid and covariance
    double sw = 0, sx = 0, sy = 0;
    for (int y = 0; y < roi.rows; ++y) {
        const uchar* p = roi.ptr<uchar>(y);
        for (int x = 0; x < roi.cols; ++x) {
            const double w = std::max(0.0, p[x] - mean);
            sw += w, sx += w * x, sy += w * y;
        }
    }
    if (sw <= 0) return;
    const double mx = sx / sw, my = sy / sw;
    double cxx = 0, cyy = 0, cxy = 0;
    for (int y = 0; y < roi.rows; ++y) {
        const uchar* p = roi.ptr<uchar>(y);
        for (int x = 0; x < roi.cols; ++x) {
            const double w = std::max(0.0, p[x] - mean);
            cxx += w * (x - mx) * (x - mx);
            cyy += w * (y - my) * (y - my);
            cxy += w * (x - mx) * (y - my);
        }
    }
    // principal eigenvector of [[cxx cxy][cxy cyy]]
    const double theta = 0.5 * std::atan2(2 * cxy, cxx - cyy);
    cv::Point2f axis(static_cast<float>(std::cos(theta)), static_cast<float>(std::sin(theta)));
    if (std::abs(axis.y) < 0.5f) return;  // not a vertical-ish bar; keep the rect corners
    if (axis.y > 0) axis = -axis;         // point upward
    const cv::Point2f centroid(static_cast<float>(mx + box.x), static_cast<float>(my + box.y));

    auto findEdge = [&](int dir) -> cv::Point2f {
        const float L = light.length;
        const cv::Point2f d = axis * static_cast<float>(dir);
        const cv::Point2f n(-axis.y, axis.x);
        const int half = std::max(0, cvRound((light.width - 2) / 2));
        cv::Point2f acc(0, 0);
        int count = 0;
        for (int i = -half; i <= half; ++i) {
            const cv::Point2f start = centroid + d * (0.35f * L) + n * static_cast<float>(i);
            float best = 0;
            cv::Point2f best_pt;
            bool found = false;
            cv::Point2f prev = start;
            for (float s = 1; s < 0.35f * L; s += 1) {
                const cv::Point2f cur = start + d * s;
                const float drop = float(sampleGray(gray, prev.x, prev.y)) - sampleGray(gray, cur.x, cur.y);
                if (drop > best) best = drop, best_pt = prev, found = true;
                prev = cur;
            }
            if (found && best > 8) acc += best_pt, ++count;
        }
        if (count == 0) return {-1, -1};
        cv::Point2f p = acc / count;
        // project onto the axis so both endpoints lie on the symmetry line
        const float t = (p - centroid).dot(axis);
        return centroid + axis * t;
    };

    const cv::Point2f top = findEdge(+1), bottom = findEdge(-1);
    if (top.x < 0 || bottom.x < 0) return;
    const float new_len = static_cast<float>(cv::norm(top - bottom));
    // reject refinements that change the bar length implausibly
    if (new_len < 0.7f * light.length || new_len > 1.4f * light.length) return;
    light.top = top;
    light.bottom = bottom;
    light.center = (top + bottom) * 0.5f;
    light.length = new_len;
}

std::vector<ArmorDetection> Detector::detect(const cv::Mat& bgr, DetectorDebug* debug) {
    if (usingOnnx()) {
        auto armors = keypoint_->detect(bgr);
        if (debug) debug->pairs_before_classify = static_cast<int>(armors.size());
        return armors;
    }
    cv::Mat mask;
    std::vector<Light> lights = findLights(bgr, mask);
    std::vector<ArmorDetection> armors = matchLights(lights);

    if (cfg_.refine_corners && !armors.empty()) {
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        for (auto& a : armors) {
            refineCorners(gray, a.left);
            refineCorners(gray, a.right);
        }
    }
    for (auto& a : armors) {
        a.corners = {a.left.bottom, a.left.top, a.right.top, a.right.bottom};
        a.center = (a.corners[0] + a.corners[1] + a.corners[2] + a.corners[3]) * 0.25f;
    }
    if (debug) {
        debug->mask = mask;
        debug->lights = lights;
        debug->pairs_before_classify = static_cast<int>(armors.size());
    }
    classifier_.classify(bgr, armors);
    return armors;
}

}  // namespace autoaim
