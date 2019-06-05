#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include <vector>
#include <algorithm>
#include <map>
#include <time.h>
#include <thread>
#include <unistd.h>
#include <functional>
#include <swarm_localization/swarm_types.hpp>
#include <set>

typedef std::map<int, Eigen::Vector3d> ID2Vector3d;
typedef std::map<int, Eigen::Quaterniond> ID2Quat;

using namespace swarm;
using namespace Eigen;

struct SwarmFrameError;
struct SwarmHorizonError;

inline float rand_FloatRange(float a, float b) {
    return ((b - a) * ((float) rand() / RAND_MAX)) + a;
}

typedef ceres::DynamicAutoDiffCostFunction<SwarmFrameError, 7>  SFErrorCost;
typedef ceres::DynamicAutoDiffCostFunction<SwarmHorizonError, 7> HorizonCost;

//Poses is dict of timestamp and then id;
//state<ts,id>
typedef std::map<int64_t, std::map<int,double*>> EstimatePoses;
typedef std::map<int, std::map<int64_t,double*>> EstimatePosesIDTS;

class SwarmLocalizationSolver {


    std::vector<SwarmFrame> sf_sld_win;
    std::map<int64_t, SwarmFrame> all_sf;
    int64_t last_kf_ts = 0;
    int64_t last_saved_est_kf_ts = 0;
    unsigned int drone_num = 0;

    unsigned int solve_count = 0;
    const double min_accept_keyframe_movement = 0.2;

//    std::vector<double*> _swarm_est_poses;
//

    EstimatePoses est_poses_tsid, est_poses_tsid_saved;
    EstimatePosesIDTS est_poses_idts, est_poses_idts_saved;

    bool detect_outlier(const SwarmFrame &sf) const;
    void delete_frame_i(int i);

    bool is_frame_useful(unsigned int i) const;

    void process_frame_clear();

    void random_init_pose(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts) const {

    }

    void sync_est_poses(const EstimatePoses &_est_poses_tsid);

    CostFunction *
    _setup_cost_function_by_sf(const SwarmFrame &sf, std::map<int, int> id2poseindex, bool is_lastest_frame) const;


    void
    setup_problem_with_sferror(const EstimatePoses &swarm_est_poses, Problem &problem, const SwarmFrame &sf, bool is_lastest_frame) const;

    CostFunction *
    _setup_cost_function_by_nf_win(const std::vector<NodeFrame> &nf_win, const std::map<int64_t, int> & ts2poseindex, bool is_self) const;

    void setup_problem_with_sfherror(const EstimatePosesIDTS & est_poses_idts, Problem &problem, int _id) const;

    double solve_once(EstimatePoses &swarm_est_poses, EstimatePosesIDTS &est_poses_idts, bool report = false);
    
    std::vector<int> judge_is_key_frame(const SwarmFrame &sf);

    void add_as_keyframe(const SwarmFrame &sf);
    
    bool solve_with_multiple_init(int start_drone_num, int min_number = 5, int max_number = 10);
    
    unsigned int max_frame_number = 20;
    unsigned int min_frame_number = 10;

    std::set<int> all_nodes;

    unsigned int last_drone_num = 0;

    std::map<unsigned int, unsigned int> node_kf_count;

    bool has_new_keyframe = false;

public:
    int self_id = -1;
    unsigned int thread_num;
    double cost_now = 0;
    double acpt_cost = 0.4;

    double initial_random_noise = 1.0;

    bool finish_init = false;

    ros::Time last_est_time_tick = ros::Time::now();

    SwarmLocalizationSolver(int _max_frame_number, int _min_frame_number, double _acpt_cost = 0.4,
                            int _thread_num = 4) :
            max_frame_number(_max_frame_number), min_frame_number(_min_frame_number),
            thread_num(_thread_num), acpt_cost(_acpt_cost) {
    }

   
    void add_new_swarm_frame(const SwarmFrame &sf);

    SwarmFrameState PredictSwarm(const SwarmFrame &sf) const;

    Pose PredictNode(const NodeFrame & nf, bool attitude_yaw_only=false) const;

    bool CanPredictSwarm() {
        return finish_init;
    }


    double solve_time_count = 0;


    double solve();

    inline unsigned int sliding_window_size() const;

    
};