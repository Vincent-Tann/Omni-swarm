#include "ImageDescriptor_t.hpp"
#include <swarm_msgs/ImageDescriptor.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <opencv2/opencv.hpp>

inline swarm_msgs::ImageDescriptor toROSMsg(const ImageDescriptor_t & _img) {
    swarm_msgs::ImageDescriptor img_des;
    return img_des;
}


inline Pose_t fromROSPose(const geometry_msgs::Pose & pose) {
    Pose_t t;
    t.orientation[0] = pose.orientation.x;
    t.orientation[1] = pose.orientation.y;
    t.orientation[2] = pose.orientation.z;
    t.orientation[3] = pose.orientation.w;

    t.position[0] = pose.position.x;
    t.position[1] = pose.position.y;
    t.position[2] = pose.position.z;
    return t;
}


inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point2d_t> & dst) {
    for (auto pt : src) {
        Point2d_t pt2;
        pt2.x = pt.x;
        pt2.y = pt.y;
        dst.push_back(pt2);
    }
}

inline void ROSPoints2LCM(const std::vector<geometry_msgs::Point32> & src, std::vector<Point3d_t> & dst) {
    for (auto pt : src) {
        Point3d_t pt3;
        pt3.x = pt.x;
        pt3.y = pt.y;
        pt3.z = pt.z;
        dst.push_back(pt3);
    }
}


inline cv::Mat cvfeatureFromByte(uint8_t*data, int feature_num=1000, int feature_len = 32) {
    cv::Mat mat(feature_num, feature_len, CV_8UC1);
    memcpy(mat.data, data, feature_num*feature_len*sizeof(uint8_t));
    return mat;
}


inline cv::Point2f toCV(Point2d_t a) {
    cv::Point2f pt;
    pt.x = a.x;
    pt.y = a.y;
    return pt;
}

inline cv::Point3f toCV(Point3d_t a) {
    cv::Point3f pt;
    pt.x = a.x;
    pt.y = a.y;
    pt.z = a.z;
    return pt;
}


inline std::vector<cv::Point2f> toCV(std::vector<Point2d_t> arr) {
    std::vector<cv::Point2f> _arr;
    for (auto a : arr) {
        _arr.push_back(toCV(a));
    }
    return _arr;
}


inline std::vector<cv::Point2f> toCV(Point2d_t * arr, int len) {
    std::vector<cv::Point2f> _arr;
    for (int i = 0; i < len; i++) {
        auto a = arr[i];
        _arr.push_back(toCV(a));
    }
    return _arr;
}
