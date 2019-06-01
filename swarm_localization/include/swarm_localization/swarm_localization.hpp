#include <iostream>
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <thread>
#include <unistd.h>
#include "localiztion_costfunction.hpp"
#include <functional>
#include <swarm_localization/swarm_types.hpp>


typedef std::map<int, Eigen::Vector3d> ID2Vector3d;
typedef std::map<int, Eigen::Quaterniond> ID2Quat;

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::SizedCostFunction;
using ceres::Covariance;

using namespace swarm;
using namespace Eigen;

float rand_FloatRange(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, 7>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, 7> HorizonCost;

class SwarmLocalizationSolver {

    CostFunction *
    _setup_cost_function_by_sf(SwarmFrame &sf, std::vector<double*> &swarm_est_poses, bool is_lastest_frame) {
        SwarmFrameError * sferror = new SwarmFrameError(sf, all_nodes, id_stamp_pose[sf.ts], is_lastest_frame);
        int res_num = sferror->residual_count();
        auto cost_function  = new SFErrorCost(sferror);
        int poses_num = swarm_est_poses.size();
        ROS_INFO("Poses num %d res num %d", poses_num, res_num);
        for (int i =0;i < poses_num; i ++){
            cost_function->AddParameterBlock(7);
        }
        cost_function->SetNumResiduals(res_num);
        return cost_function;
    }

    CostFunction *
    _setup_cost_function_by_sf_win(std::vector<SwarmFrame> &sf_win, std::vector<double*> &swarm_est_poses) {
        auto she = new SwarmHorizonError(sf_win, all_nodes, id_stamp_pose);

        HorizonCost *cost_function = new HorizonCost(she);
        int res_num = she->residual_count();

        int poses_num = swarm_est_poses.size();
        for (int i =0;i < poses_num; i ++){
            cost_function->AddParameterBlock(7);
        }
        ROS_INFO("SFHorizon res %d", res_num);
        cost_function->SetNumResiduals(res_num);
        return cost_function;
    }

    std::vector<SwarmFrame> sf_sld_win;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

    std::vector<double*> _swarm_est_poses;

public:
    std::vector<int> all_nodes;
    unsigned int max_frame_number = 20;
    unsigned int min_frame_number = 10;
    unsigned int last_drone_num = 0;
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;

    int last_problem_ptr = 0;
    bool finish_init = false;

    std::map<int, Vector3d> est_pos;
    std::map<int, Vector3d> est_vel;

    IDStampPose id_stamp_pose;


    ros::Time last_est_time_tick = ros::Time::now();

    std::map<unsigned int, unsigned int> node_kf_count;


    bool has_new_keyframe = false;


    SwarmLocalizationSolver(int _max_frame_number, int _min_frame_number, double _acpt_cost = 0.4,
               int _thread_num = 4) :
            max_frame_number(_max_frame_number), min_frame_number(_min_frame_number),
            thread_num(_thread_num), acpt_cost(_acpt_cost) {
    }

    void init_pose(int _id, int tick, double * _p) {
        if (_id == self_id) {
            sf_sld_win[tick].id2nodeframe[_id].pose().to_vector(_p);
            return;
        }
        _p[0] = rand_FloatRange(-3, 3);
        _p[1] = rand_FloatRange(-3, 3);
        _p[2] = rand_FloatRange(0, 3);

        _p[3] = 1;
        _p[4] = 0;
        _p[5] = 0;
        _p[6] = 0;
    }

    void random_init_pose(std::vector<double*> &_est_poses, int start = 0, int end = 100) {
        _est_poses.clear();
        for(int i = 0;i < sf_sld_win.size(); i++) {
            if (i!=sf_sld_win.size()-1) {
                for (auto it : sf_sld_win[i].id2nodeframe) {
                    auto sf = sf_sld_win[i];
                    int _id = it.first;
                    double * _p = new double[7];
                    init_pose(it.first, i, _p);
                    _est_poses.push_back(_p);
                    id_stamp_pose[sf.ts][_id] =  _est_poses.size() - 1;

                }
            } else {
                ROS_INFO("Is last frame, avoid self id %d", self_id);
                for (auto it : sf_sld_win[i].id2nodeframe) {
                    if (it.first != self_id) {
                        auto sf = sf_sld_win[i];
                        int _id = it.first;
                        double * _p = new double[7];
                        init_pose(it.first, i, _p);
                        _est_poses.push_back(_p);
                        id_stamp_pose[sf.ts][_id] = _est_poses.size() - 1;
                    }

                }
            }

        }
        ROS_INFO("EST %d poses", _est_poses.size());
    }

    bool detect_outlier(const SwarmFrame &sf) {
        //Detect if it's outlier

        // for (int i = 0; i  < s)
        return false;
    }

    std::vector<int> judge_is_key_frame(const SwarmFrame &sf) {

        auto _ids = sf.node_id_list;
        std::vector<int> ret(0);
        if (_ids.size() < 2)
            return ret;

//        //Temp code
//        if (_ids.size() < drone_num) {
//            return ret;
//        }

        if (sf_sld_win.empty()) {
            for (auto _id : _ids) {
                node_kf_count[_id] = 1;
            }
            return _ids;
        }


        for (auto _id : _ids) {
            if (sf_sld_win.back().HasID(_id)) {
                Eigen::Vector3d _diff = sf.position(_id) - sf_sld_win.back().position(_id);
                if (_diff.norm() > min_accept_keyframe_movement || sf.HasDetect(_id)) {
                    ret.push_back(_id);
                    node_kf_count[_id] += 1;
                }
            } else {
                ret.push_back(_id);
                node_kf_count[_id] += 1;
            }
        }
        return ret;
    }

    void delete_frame_i(int i) {
        sf_sld_win.erase(sf_sld_win.begin() + i);
    }

    bool is_frame_useful(unsigned int i) {
        for (unsigned int id : sf_sld_win[i].node_id_list) {
            if (node_kf_count[id] < min_frame_number) {
                return true;
            }
        }
        return false;
    }

    void process_frame_clear() {
        unsigned int i = 0;
        //Delete non keyframe first
        while (i < sf_sld_win.size() && sf_sld_win.size() > max_frame_number) {
            if (!is_frame_useful(i)) {
                delete_frame_i(i);
            } else {
                i++;
            }
        }
    }


    void add_new_swarm_frame(const SwarmFrame &sf) {
        process_frame_clear();
        if (detect_outlier(sf)) {
            ROS_INFO("Outlier detected!");
            return;
        }

        auto _ids = sf.node_id_list;

        std::vector<int> is_kf_list = judge_is_key_frame(sf);
        if (!is_kf_list.empty()) {

            has_new_keyframe = true;

            sf_sld_win.push_back(sf);
            id_stamp_pose[sf.ts] = std::map<int, int>();


        }

        if (_ids.size() > drone_num) {
            //For here the drone num increase
            drone_num = _ids.size();
        }
    }

    Eigen::Vector3d get_estimate_pos(int _id) {
        return est_pos[_id];
    }

    // Eigen::Vector3d predict_pos(Eigen::Vector3d pos, Eigen::e)

    bool PredictSwarm(const SwarmFrame &sf, SwarmFrameState & _s_state) {

        auto _ids = sf.node_id_list;
        unsigned int drone_num_now = _ids.size();

        int self_ptr = 0;
        for (unsigned int i = 0; i < _ids.size(); i++) {
            if (_ids[i] == self_id) {
                self_ptr = i;
                break;
            }
        }

        return true;

        /*
        for (unsigned int i = 0; i < _ids.size(); i++) {
            int _id = _ids[i];
            Eigen::Vector3d pos = swarmRes.est_id_pose_in_k(i, self_ptr, Zxyzth);
            Eigen::Vector3d vel = swarmRes.est_id_vel_in_k(i, self_ptr, Zxyzth);
            Eigen::Quaterniond quat = swarmRes.est_id_quat_in_k(i, self_ptr, Zxyzth);

            est_pos[_id] = pos;
            est_vel[_id] = vel;
            id2vec[_id] = pos;
            id2vel[_id] = vel;
            id2quat[_id] = quat;

            printf("Id %d pos %3.2f %3.2f %3.2f dis27 %3.2f\n", _id, pos.x(), pos.y(), pos.z(),
                   (pos - est_pos[7]).norm());
        }

        printf("\nDistance Matrix\n\t\t");
        for (int i = 0; i < drone_num_now; i++) {
            int _id_i = _ids[i];
            printf("%d:\t", _id_i);
        }

        for (auto idi : _ids) {
            printf("\nE/M/B%d:\t", idi);
            for (auto idj : _ids) {
                double est_d = swarmRes.distance_idj_idi(idj, idi, Zxyzth).norm();
                double bias = swarmRes.bias_ij(idi, idj, Zxyzth);
                printf("%3.2f:%3.2f:%3.2f\t", est_d, (sf.distance(idj, idi) + sf.distance(idi, idj)) / 2, bias);
            }

        }

        printf("\n\n");
        */


    }


    bool solve_with_multiple_init(int start_drone_num, int min_number = 5, int max_number = 10) {

        double cost = acpt_cost;
        bool cost_updated = false;

        // ROS_INFO("Try to use multiple init to solve expect cost %f", cost);

        std::vector<double*> _est_poses;
        for (int i = 0; i < max_number; i++) {
            random_init_pose(_est_poses, start_drone_num, drone_num);
            double c = solve_once(_est_poses, false);
            ROS_INFO("Got better cost %f", c);

            if (c < cost) {
                ROS_INFO("Got better cost %f", c);
                cost_updated = true;
                cost_now = cost = c;
                _swarm_est_poses = _est_poses;
                if (i > min_number) {
                    return true;
                }
            }
        }

        return cost_updated;
    }

    double solve() {
        if (self_id < 0
            || node_kf_count.find(self_id) == node_kf_count.end()
            || node_kf_count[self_id] < min_frame_number)
            return -1;

        if (!has_new_keyframe)
            return cost_now;

        if (!finish_init || drone_num > last_drone_num) {
            ROS_INFO("No init before, try to init");
            finish_init = solve_with_multiple_init(last_drone_num);
            if (finish_init) {
                last_drone_num = drone_num;
                ROS_INFO("Finish init\n");
            }

        } else if (has_new_keyframe) {
            cost_now = solve_once(this->_swarm_est_poses, true);
        }

        if (cost_now > acpt_cost)
            finish_init = false;
        return cost_now;
    }

    inline unsigned int sliding_window_size() const {
        return sf_sld_win.size();
    }

    double solve_once(std::vector<double*> &swarm_est_poses, bool report = false) {

        Problem problem;

//        if (solve_count % 10 == 0)
        printf("TICK %d Trying to solve size %d, poses %d\n", solve_count, sliding_window_size(), swarm_est_poses.size());

        has_new_keyframe = false;

        for (unsigned int i = 0; i < sf_sld_win.size(); i++ ) {
            SwarmFrame &sf = sf_sld_win[i];
            bool is_lastest_frame = (i==sf_sld_win.size()-1);
            problem.AddResidualBlock(
                    _setup_cost_function_by_sf(sf, swarm_est_poses, is_lastest_frame),
                    nullptr,
                    swarm_est_poses
            );
        }

        problem.AddResidualBlock(_setup_cost_function_by_sf_win(sf_sld_win, swarm_est_poses), nullptr, swarm_est_poses);

        last_problem_ptr = sliding_window_size();

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
        options.max_num_iterations = 200;
        options.num_threads = thread_num;
        Solver::Summary summary;
        // options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type = ceres::DOGLEG;

        // std::cout << "Start solving problem" << std::endl;
        ceres::Solve(options, &problem, &summary);

        double equv_cost = 2 * summary.final_cost / sliding_window_size();

        equv_cost = equv_cost / (double) (drone_num * (drone_num - 1) / 2);
        equv_cost = sqrt(equv_cost);
        if (!report) {
            return equv_cost;
        }

        // if (solve_count % 10 == 0)
        std::cout << "\n\nSize:" << sliding_window_size() << "\n" << summary.BriefReport() << " Equv cost : "
                  << equv_cost << " Time : " << summary.total_time_in_seconds * 1000 << "ms\n";
        // std::cout << summary.FullReport()<< "\n";


        if (solve_count % 10 == 0)
            auto sf = sf_sld_win.back();

        /*
        for (int _id : sf.node_id_list) {
            int i = id_to_index[_id];
            ROS_INFO(
                    "i %d id %d x %5.4f y %5.4f z %5.4f :dyaw %5.4f estpos %5.4f %5.4f %5.4f self %3.2f %3.2f %3.2f\n",
                    i,
                    _id,
                    Zxyzth[i * 4],
                    Zxyzth[i * 4 + 1],
                    Zxyzth[i * 4 + 2],
                    Zxyzth[i * 4 + 3],
                    est_pos[_id].x(),
                    est_pos[_id].y(),
                    est_pos[_id].z(),
                    sf.position(_id).x(),
                    sf.position(_id).y(),
                    sf.position(_id).z()
            );
        }*/

        solve_count++;

        return equv_cost;
    }
};
