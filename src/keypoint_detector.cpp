#include "autoaim/keypoint_detector.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace autoaim {

Letterbox letterboxFor(int img_w, int img_h, int input_size) {
    Letterbox lb;
    lb.scale = std::min(static_cast<float>(input_size) / img_w, static_cast<float>(input_size) / img_h);
    lb.pad_x = (input_size - img_w * lb.scale) / 2;
    lb.pad_y = (input_size - img_h * lb.scale) / 2;
    return lb;
}

namespace {

float iou(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x0 = std::max(a.x, b.x), y0 = std::max(a.y, b.y);
    const float x1 = std::min(a.x + a.width, b.x + b.width), y1 = std::min(a.y + a.height, b.y + b.height);
    const float inter = std::max(0.f, x1 - x0) * std::max(0.f, y1 - y0);
    const float uni = a.width * a.height + b.width * b.height - inter;
    return uni > 0 ? inter / uni : 0;
}

Light edgeLight(const cv::Point2f& bottom, const cv::Point2f& top, Color color) {
    Light l;
    l.bottom = bottom;
    l.top = top;
    l.center = (bottom + top) * 0.5f;
    l.length = static_cast<float>(cv::norm(top - bottom));
    l.width = l.length * 0.2f;  // bars are ~5:1; pose solving uses corners, not this
    const cv::Point2f v = top - bottom;
    l.tilt_deg = static_cast<float>(std::atan2(v.x, -v.y) * 180.0 / 3.14159265358979323846);
    l.color = color;
    l.rect = cv::RotatedRect(l.center, cv::Size2f(l.width, l.length), l.tilt_deg);
    return l;
}

}  // namespace

std::vector<KeypointCandidate> parseKeypointOutput(const cv::Mat& output, int num_classes,
                                                   float conf_thr, float nms_thr, int input_size,
                                                   int img_w, int img_h) {
    std::vector<KeypointCandidate> out;
    if (output.dims != 3 || num_classes <= 0) return out;
    cv::Mat m;
    output.convertTo(m, CV_32F);
    const int channels = 4 + num_classes + 12;
    const int d1 = m.size[1], d2 = m.size[2];
    bool channels_first;
    int n;
    if (d1 == channels) {
        channels_first = true;
        n = d2;
    } else if (d2 == channels) {
        channels_first = false;
        n = d1;
    } else {
        return out;  // not the tensor layout this parser implements
    }
    const float* data = m.ptr<float>();
    auto at = [&](int c, int i) { return channels_first ? data[c * n + i] : data[i * channels + c]; };
    const Letterbox lb = letterboxFor(img_w, img_h, input_size);
    auto unmap = [&](float x, float y) {
        return cv::Point2f((x - lb.pad_x) / lb.scale, (y - lb.pad_y) / lb.scale);
    };

    std::vector<KeypointCandidate> cands;
    for (int i = 0; i < n; ++i) {
        int best_cls = -1;
        float best = 0;
        for (int c = 0; c < num_classes; ++c) {
            const float s = at(4 + c, i);
            if (s > best) best = s, best_cls = c;
        }
        if (best_cls < 0 || best < conf_thr) continue;
        const float cx = at(0, i), cy = at(1, i), w = at(2, i), h = at(3, i);
        KeypointCandidate kc;
        kc.score = best;
        kc.cls = best_cls;
        const cv::Point2f tl = unmap(cx - w / 2, cy - h / 2);
        const cv::Point2f br = unmap(cx + w / 2, cy + h / 2);
        kc.box = cv::Rect2f(tl, br);
        const int kbase = 4 + num_classes;
        for (int k = 0; k < 4; ++k) {
            kc.kpts[k] = unmap(at(kbase + 3 * k, i), at(kbase + 3 * k + 1, i));
            kc.kpt_conf[k] = at(kbase + 3 * k + 2, i);
        }
        cands.push_back(kc);
    }
    std::sort(cands.begin(), cands.end(),
              [](const KeypointCandidate& a, const KeypointCandidate& b) { return a.score > b.score; });
    std::vector<bool> suppressed(cands.size(), false);
    for (std::size_t i = 0; i < cands.size(); ++i) {
        if (suppressed[i]) continue;
        out.push_back(cands[i]);
        for (std::size_t j = i + 1; j < cands.size(); ++j)
            if (!suppressed[j] && iou(cands[i].box, cands[j].box) > nms_thr) suppressed[j] = true;
    }
    return out;
}

std::array<cv::Point2f, 4> orderArmorCorners(const std::array<cv::Point2f, 4>& kpts) {
    std::array<int, 4> idx{0, 1, 2, 3};
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return kpts[a].x < kpts[b].x; });
    const int l0 = idx[0], l1 = idx[1], r0 = idx[2], r1 = idx[3];
    // Within each edge, bottom is the larger image y.
    const int lb = kpts[l0].y > kpts[l1].y ? l0 : l1;
    const int lt = lb == l0 ? l1 : l0;
    const int rb = kpts[r0].y > kpts[r1].y ? r0 : r1;
    const int rt = rb == r0 ? r1 : r0;
    return {kpts[lb], kpts[lt], kpts[rt], kpts[rb]};
}

ArmorDetection candidateToArmor(const KeypointCandidate& c, const DetectorConfig& cfg) {
    ArmorDetection d;
    d.corners = orderArmorCorners(c.kpts);
    d.center = (d.corners[0] + d.corners[1] + d.corners[2] + d.corners[3]) * 0.25f;
    d.left = edgeLight(d.corners[0], d.corners[1], cfg.enemy_color);
    d.right = edgeLight(d.corners[3], d.corners[2], cfg.enemy_color);
    d.color = cfg.enemy_color;
    d.confidence = c.score;
    d.number = c.cls >= 0 && c.cls < static_cast<int>(cfg.onnx_classes.size())
                   ? cfg.onnx_classes[c.cls]
                   : "unknown";
    const float width = 0.5f * (static_cast<float>(cv::norm(d.corners[2] - d.corners[1])) +
                                static_cast<float>(cv::norm(d.corners[3] - d.corners[0])));
    const float height = 0.5f * (d.left.length + d.right.length);
    d.type = height > 0 && width / height > cfg.large_plate_ratio ? ArmorType::LARGE : ArmorType::SMALL;
    return d;
}

KeypointDetector::KeypointDetector(const DetectorConfig& cfg) : cfg_(cfg) {
    if (cfg_.onnx_path.empty()) return;
    try {
        net_ = cv::dnn::readNetFromONNX(cfg_.onnx_path);
    } catch (const cv::Exception& e) {
        std::cerr << "keypoint detector: cannot load " << cfg_.onnx_path << ": " << e.what() << "\n";
        net_ = cv::dnn::Net();
    }
}

std::vector<ArmorDetection> KeypointDetector::detect(const cv::Mat& bgr) {
    std::vector<ArmorDetection> out;
    if (net_.empty() || bgr.empty()) return out;
    const int in = cfg_.keypoint_input;
    const Letterbox lb = letterboxFor(bgr.cols, bgr.rows, in);
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(static_cast<int>(std::lround(bgr.cols * lb.scale)),
                                      static_cast<int>(std::lround(bgr.rows * lb.scale))));
    cv::Mat canvas(in, in, CV_8UC3, cv::Scalar(114, 114, 114));
    const int ox = static_cast<int>(std::lround(lb.pad_x)), oy = static_cast<int>(std::lround(lb.pad_y));
    resized.copyTo(canvas(cv::Rect(ox, oy, resized.cols, resized.rows)));
    const cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(in, in), cv::Scalar(),
                                                /*swapRB=*/true, /*crop=*/false);
    net_.setInput(blob);
    const cv::Mat output = net_.forward();
    const auto cands = parseKeypointOutput(output, static_cast<int>(cfg_.onnx_classes.size()),
                                           static_cast<float>(cfg_.keypoint_conf),
                                           static_cast<float>(cfg_.keypoint_nms), in, bgr.cols, bgr.rows);
    out.reserve(cands.size());
    for (const auto& c : cands) out.push_back(candidateToArmor(c, cfg_));
    return out;
}

}  // namespace autoaim
