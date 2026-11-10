// Core data types shared by every stage of the pipeline.
//
// Frames used throughout:
//   camera : OpenCV optical frame. X right, Y down, Z forward (metres).
//   world  : gimbal-centred, gravity aligned. x forward, y left, z up (metres).
//            The camera is assumed to sit at the world origin (fixed camera in
//            the test videos); R_world_cam carries the camera pitch.
//
// Armor yaw follows rm_auto_aim: the armor x-axis points INTO the robot, so a
// plate seen head-on has yaw == atan2(y, x) of its own position, and the robot
// centre is  armor_position + r * (cos(yaw), sin(yaw)).
#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <string>
#include <vector>

namespace autoaim {

constexpr double kPi = 3.14159265358979323846;

enum class Color { RED = 0, BLUE = 1 };
enum class ArmorType { SMALL = 0, LARGE = 1 };

inline const char* toString(ArmorType t) { return t == ArmorType::SMALL ? "small" : "large"; }

// Physical armor dimensions (metres). Light bars: distance between the two bar
// centre lines x bar length. Panel: the hittable plate.
struct ArmorGeometry {
    double light_width;   // distance between bar centre lines
    double light_height;  // bar length
    double panel_width;
    double panel_height;
};
constexpr ArmorGeometry kSmallArmor{0.135, 0.055, 0.135, 0.125};
constexpr ArmorGeometry kLargeArmor{0.225, 0.055, 0.230, 0.125};
inline const ArmorGeometry& geometryOf(ArmorType t) {
    return t == ArmorType::SMALL ? kSmallArmor : kLargeArmor;
}

// A light bar in the image.
struct Light {
    cv::RotatedRect rect;
    cv::Point2f top, bottom, center;
    float length = 0.f;    // |top - bottom|
    float width = 0.f;     // bar thickness
    float tilt_deg = 0.f;  // angle of the long axis from vertical
    Color color = Color::RED;
};

// A matched light pair in the image (before pose estimation).
struct ArmorDetection {
    Light left, right;
    ArmorType type = ArmorType::SMALL;
    // Corner order (the contract the number classifier depends on):
    //   0 = left bottom, 1 = left top, 2 = right top, 3 = right bottom
    std::array<cv::Point2f, 4> corners;
    cv::Point2f center;
    std::string number;       // "1".."4", "outpost", "guard", "base"
    float confidence = 0.f;
    cv::Mat number_img;       // 28x28 binarised digit (debug)
    Color color = Color::RED;  // team colour the plate was detected as
};

// Identity of the physical robot a detection belongs to.  The number alone
// is NOT an identity: both teams field a "3", and a hero's large "1" is a
// different vehicle from anything else wearing "1".  Trackers key on this
// (colour + number + plate size) so a same-number detection of the wrong
// colour or size can never be fused into a track.
inline std::string deviceId(const ArmorDetection& det) {
    return std::string(det.color == Color::RED ? "R" : "B") + det.number +
           (det.type == ArmorType::LARGE ? ":large" : ":small");
}

// An armor with a 3-D pose in the world frame.
struct ArmorObservation {
    ArmorDetection det;
    cv::Vec3d position;      // plate centre, world frame
    double yaw = 0.0;        // world yaw of the armor x-axis (into the robot)
    double distance = 0.0;   // |position|
    double reproj_error = 0.0;  // mean corner reprojection error (px) of the final pose
    double image_center_dist = 0.0;  // px from the image centre (for target choice)
};

// Everything the pipeline produced for one frame.
struct FrameObservations {
    int frame = 0;
    double stamp = 0.0;  // seconds
    std::vector<ArmorObservation> armors;
};

inline double wrapAngle(double a) {
    while (a > kPi) a -= 2 * kPi;
    while (a <= -kPi) a += 2 * kPi;
    return a;
}

}  // namespace autoaim
