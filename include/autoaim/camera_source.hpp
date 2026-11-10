// Camera acquisition with exposure timestamps.
//
// Implements step 1 of docs/timestamp_design.md (G-02): every frame leaves
// this interface carrying the host-clock time of its *exposure centre*, so
// the pose side can interpolate the gimbal pose at that instant (see
// pose_buffer.hpp) instead of guessing with "now".
//
// Sources without a hardware timestamp (video files, plain UVC devices)
// still produce stamps, but flag them with hw_stamp = false: they are fine
// for offline evaluation and must not be treated as meeting the G-02 budget
// (exposure jitter sigma <= 0.3 ms) on a real robot.
#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace autoaim {

struct SourceFrame {
    cv::Mat image;
    double exposure_stamp = 0.0;  // seconds, host steady clock, exposure centre
    std::uint64_t index = 0;
    bool hw_stamp = false;  // stamp came from the camera's own clock (aligned)
};

class CameraSource {
public:
    virtual ~CameraSource() = default;
    // Blocking read of the next frame. False on end-of-stream / fatal error.
    virtual bool grab(SourceFrame& out) = 0;
    virtual double fps() const = 0;
    virtual std::string name() const = 0;
};

// Offline evaluation: stamps are synthetic (index / fps), exactly like the
// existing apps compute them, so offline numbers stay comparable.
class VideoFileSource : public CameraSource {
public:
    explicit VideoFileSource(const std::string& path);
    ~VideoFileSource() override;
    bool grab(SourceFrame& out) override;
    double fps() const override { return fps_; }
    std::string name() const override { return "video:" + path_; }

private:
    std::string path_;
    double fps_ = 0.0;
    std::uint64_t index_ = 0;
    void* cap_ = nullptr;  // cv::VideoCapture, hidden to keep videoio out of this header
};

// Live UVC / V4L2 device through cv::VideoCapture. The stamp is the host
// receipt time minus half the nominal frame period - an approximation whose
// jitter does NOT meet the G-02 budget; it exists so the robot app can run
// before the MVS source is available, with the limitation explicit in data
// (hw_stamp stays false).
class DeviceSource : public CameraSource {
public:
    explicit DeviceSource(int index, double nominal_fps = 0.0);
    ~DeviceSource() override;
    bool grab(SourceFrame& out) override;
    double fps() const override { return fps_; }
    std::string name() const override { return "device"; }

private:
    double fps_ = 0.0;
    std::uint64_t index_ = 0;
    void* cap_ = nullptr;  // cv::VideoCapture
};

#ifdef AUTOAIM_WITH_MVS
// Hikvision industrial camera (the MV-CS016-10UC of G-02) through the MVS
// SDK. The frame's device timestamp is converted to the host clock with a
// ClockAligner fed on every frame, and the stamp is moved to the exposure
// centre (+ exposure/2). Requires the MVS SDK at build time:
//   cmake -DAUTOAIM_WITH_MVS=ON -DMVS_ROOT=<sdk dir> ...
struct MvsConfig {
    int device_index = 0;
    double exposure_us = 1500.0;
    double frame_rate = 165.0;  // matches the reality params used by SMBU pb2025
    bool hardware_trigger = false;  // true: external trigger starts exposure
    // Subtracted from every stamp.  The ClockAligner offset is the MEDIAN
    // receipt delay (exposure start -> host receipt), so aligned stamps are
    // late by readout + transfer + host latency.  That is not harmless: the
    // attitude is looked up at this stamp, so while the gimbal turns at w the
    // measurement is rotated by w * delay.  Measure it with DelayCalibrator
    // (delay_calibration.hpp: static plate, gimbal sweep) and put it here.
    double stamp_offset_us = 0.0;
};

class MvsSource : public CameraSource {
public:
    explicit MvsSource(const MvsConfig& cfg);
    ~MvsSource() override;
    MvsSource(const MvsSource&) = delete;
    MvsSource& operator=(const MvsSource&) = delete;
    bool grab(SourceFrame& out) override;
    double fps() const override { return cfg_.frame_rate; }
    std::string name() const override { return "mvs"; }

private:
    MvsConfig cfg_;
    std::uint64_t index_ = 0;
    class Impl;
    std::unique_ptr<Impl> impl_;
};
#endif  // AUTOAIM_WITH_MVS

}  // namespace autoaim
