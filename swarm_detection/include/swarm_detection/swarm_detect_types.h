#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <assert.h>
#include <aruco/aruco.h>
#include <camodocal/camera_models/CataCamera.h>

using namespace Eigen;
using namespace ceres;
// namespace SwarmDetection {}
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond attitude = Eigen::Quaterniond(1, 0, 0, 0);

    void to_vector(double ret[]) {
        ret[0] = attitude.w();
        ret[1] = attitude.x();
        ret[2] = attitude.y();
        ret[3] = attitude.z();

        ret[4] = position.x();
        ret[5] = position.y();
        ret[6] = position.z();
    }
    Eigen::Vector3d apply_pose_to(Eigen::Vector3d point) const {
        return attitude * point + position;
    }
};



//Output q, t equal to T_{camtoimu}^-1*T_{imu}^-1
//That's (T_{imu} T_{camtoimu})^-1
// Thatis T_{cam}^-1
template<typename T>
void
worldToCameraTransform(const T* const q_cam_odo, const T* const t_cam_odo,
                       const T* const p_odo, const T* const att_odo,
                       T* q, T* t, bool optimize_cam_odo_z = true)
{
    Eigen::Quaternion<T> q_z_inv(cos(att_odo[0] / T(2)), T(0), T(0), -sin(att_odo[0] / T(2)));
    Eigen::Quaternion<T> q_y_inv(cos(att_odo[1] / T(2)), T(0), -sin(att_odo[1] / T(2)), T(0));
    Eigen::Quaternion<T> q_x_inv(cos(att_odo[2] / T(2)), -sin(att_odo[2] / T(2)), T(0), T(0));

    Eigen::Quaternion<T> q_zyx_inv = q_x_inv * q_y_inv * q_z_inv;

    T q_odo[4] = {q_zyx_inv.w(), q_zyx_inv.x(), q_zyx_inv.y(), q_zyx_inv.z()};

    T q_odo_cam[4] = {q_cam_odo[3], -q_cam_odo[0], -q_cam_odo[1], -q_cam_odo[2]};

    T q0[4];
    ceres::QuaternionProduct(q_odo_cam, q_odo, q0);

    T t0[3];
    T t_odo[3] = {p_odo[0], p_odo[1], p_odo[2]};

    ceres::QuaternionRotatePoint(q_odo, t_odo, t0);

    t0[0] += t_cam_odo[0];
    t0[1] += t_cam_odo[1];

    if (optimize_cam_odo_z)
    {
        t0[2] += t_cam_odo[2];
    }

    ceres::QuaternionRotatePoint(q_odo_cam, t0, t);
    t[0] = -t[0];
    t[1] = -t[1];
    t[2] = -t[2];

    // Convert quaternion from Ceres convention (w, x, y, z)
    // to Eigen convention (x, y, z, w)
    q[0] = q0[1];
    q[1] = q0[2];
    q[2] = q0[3];
    q[3] = q0[0];
}



struct Camera {
    int size_w;
    int size_h;
    camodocal::CataCamera * camera_model;
    std::vector<double> m_intrinsic_params;

    template <typename T> inline
    void project_to_camera(
        const T* const p_odo, const T* const att_odo,
        const Eigen::Matrix<T, 3, 1>& P,
        Eigen::Matrix<T, 2, 1>& p)  const {


        std::vector<T> intrinsic_params(m_intrinsic_params.begin(), m_intrinsic_params.end());
        
        T t_cam_odo[3] = {T(pos()(0)), T(pos()(1)), T(pos()(2))};
        T q_cam_odo[4] = {T(att().coeffs()(0)), T(att().coeffs()(1)), T(att().coeffs()(2)), T(att().coeffs()(3))};

        T q[4];
        T t[3];

        worldToCameraTransform(q_cam_odo, t_cam_odo, p_odo, att_odo, q, t, true);
        camera_model->spaceToPlane(intrinsic_params.data(), q, t, P,p);
    }

    template <typename T> inline
    void project_to_camera(
        const Eigen::Matrix<T, 3, 1>& P,
        Eigen::Matrix<T, 2, 1>& p)  const {
            T p_odo[3];
            p_odo[0] = T(0);
            p_odo[1] = T(0);
            p_odo[2] = T(0);

            T att_odo[3];
            att_odo[0] = T(0);
            att_odo[1] = T(0);
            att_odo[2] = T(0);


            project_to_camera(p_odo, att_odo, P, p);
    }

    Camera(const std::string& filename, Pose _pose):
        pose(_pose)
    {
        camera_model = new camodocal::CataCamera;
        camodocal::CataCamera::Parameters params = camera_model->getParameters();
        
        params.readFromYamlFile(filename);
        camera_model->setParameters(params);

        camera_model->writeParameters(m_intrinsic_params);
    }





    Pose pose;

    Quaterniond att() const {
        return pose.attitude;
    }

    Vector3d pos() const {
        return pose.position;
    }
};


struct DroneMarker {
    //Trans is relative to own drone
    int id;
    int drone_id;
    double size;
    Pose pose;

    //Corner position on the drone
    Eigen::Vector3d rel_corner_pos(int corner_no) const {
    switch (corner_no) {
        case 0:
            return pose.apply_pose_to(Eigen::Vector3d(0, - size/2, size/2));
            break;
        case 1:
            return pose.apply_pose_to(Eigen::Vector3d(0, size/2, size/2));
            break;
        case 2:
            return pose.apply_pose_to(Eigen::Vector3d(0, size/2, -size/2));
            break;
        case 3:
            return pose.apply_pose_to(Eigen::Vector3d(0, - size/2, -size/2));
            break;
    }
    return Eigen::Vector3d(0, 0, 0);
}

    DroneMarker(int _id, int _drone_id, double _size):
        size(_size), id(_id), drone_id(_drone_id)
    {}

};

typedef std::map<int, DroneMarker*> marker_dict;

class MarkerCornerObservsed {
public:
    int corner_no;
    Eigen::Vector2d observed_point;
    DroneMarker * marker = nullptr;
    Eigen::Vector3d rel_corner_pos() const {
        assert(marker!=nullptr && "Must like corner to a marker before use relative corner position");
        return marker->rel_corner_pos(this->corner_no);
    }

    MarkerCornerObservsed(int _corner_no, DroneMarker * _marker):
        corner_no(_corner_no), marker(_marker)
    {

    }
};

struct SwarmDroneDefs {
    int drone_id;
    std::map<int, DroneMarker> markers;
};



typedef std::vector<Camera*> camera_array;
typedef std::vector<MarkerCornerObservsed> corner_array;
typedef std::vector<aruco::Marker> aruco_marker_array;
