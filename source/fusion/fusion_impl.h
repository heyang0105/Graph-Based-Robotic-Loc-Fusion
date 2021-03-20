// interface of the multiple sensor fusion algorithm
// Yang He
// 2021/01/28

// the main interface of the multiple sensor fusion model
#ifndef FUSION_IMPL_H
#define FUSION_IMPL_H

#include<string>
#include<vector>

using namespace std;

namespace fusion{

// the input data type
#ifdef _WINDOWS
enum class __declspec(dllexport) DataType {
#else
enum class DataType {
#endif
    
    NONE, 

    /* GNSS, Binary Code, Laser loc*/
    LOC_TYPE,   // gives rise to the jump

    /* VO, VIO, Laser Odom, wheel odom, IMU*/
    ODOM_TYPE}; // gives rise to the accumulated drifts

// seting the fusion
#ifdef _WINDOWS
struct __declspec(dllexport) Options{
#else
struct Options{
#endif
    /* INIT: num of LOC_TYPE sensor */
    int loc_sensor_num = 1;// NOT USED

    /* INIT: num of LOC_TYPE sensor */
    int odom_sensor_num = 2; // NOT USED

    /* STRUCT DIM: data buffer*/
    int sensor_data_buffer_size = 200;

    /* STRUCT DIM: minimum window size */
    int min_window_size_to_fuse = 5;

    /* STRUCT DIM: maitained window size */
    int window_size = 10;

    /* STRUCT DIM: num to process switch calc */
    int switch_size = 3; // set switch form the newest
    
    /* CTRL: if using last remove */
    bool use_last_remove = false;

    /* CTRL: if adaptively adjust each sensors' cov based on the calculation result */
    bool use_adptive_sensor_cov = true;

    /* CTRL: anchor setting strategy */
    bool use_opitmal_estimate = true; 

    /* CTRL: pose prior */
    bool use_loc_pose_prior = true;

    /* CTRL: the weight of last calculation */
    double loc_pose_prior_scale = 5.0;

    /* CTRL: init switch value */
    double init_switch_value = 0.8;

    /* CTRL: optim*/
    int iteration_num = 20;
    
    /* CTRL: time threshold to add (unit: second) */
    double time_interval = 0.1; // the min time 

    /* CTRL: to maintain window (unit: m) */
    double min_translation_to_add = 0.02;
    
    /* CTRL: to maintain window (unit: deg)*/
    double min_angle_to_add = 5.0; //angle-axis 

    /* CTRL: to dynamicly adapt info mat */
    double varience_penetration_coeff = 2.0;

    /* CTRL: to set the switch */
    double switch_off_low_bound = 0.25;

    /* CTRL: to set the switch */
    double switch_off_up_bound = 0.9;
    
    /* CTRL: to get newest state (REMOVE) */
    string propagate_odom_sensor_name = "ODO_ODOM"; // use the highest frequency/ accuracy odom to propagate
    
    /* CTRL: to set the anchor to the map frame */
    /* all values in the sliding window are aligned into this Frame of World*/
    string main_loc_sensor_name = "LASER_LOC"; 
    
    bool check();
};

// output info
#ifdef _WINDOWS
struct __declspec(dllexport) Summary{
#else
struct Summary{
#endif
    bool is_inited = true;// if the system is 
    bool is_fusion_good = true;
    bool is_one_sensor_failed = false;
    bool is_fatal_error = false;
    vector<int> failed_sensor_indexs;
    vector<string> failed_sensor_names;
    void printSummary();
};

// the main interface
#ifdef _WINDOWS
class __declspec(dllexport) FusionImpl{	
#else
class FusionImpl{	
#endif
public:
    FusionImpl() = delete;
    FusionImpl(Options& opt);
    ~FusionImpl();

    // ### interface to use the fusion module ###
    
    // 1. Start: Set your sensor here
    int registerSensor(const string &sensor_name, const DataType &type,
        const double &sigma_trans, const double &sigma_rot);
    
    //   also the corelation between the states
    int registerExternalParams(const string &sensor_name_0, const string &sensor_name_1,
      const double& x, const double& y, const double& z, 
      const double& qw, const double& qx, const double& qy, const double& qz,
      const vector<double> &sigma_trans_vec, const vector<double> &sigma_rot_vec,
      const double &switch_prior_info);

    // 2. Set Data: Add the Data
    bool addSensorData(const string &sensor_name, 
      const double& timestamp, 
      const double& x, const double& y, const double& z, 
      const double& qw, const double& qx, const double& qy, const double& qz);

    /* the sensor's variance may be dynamicly estiamted during the process */
    bool setSensorSigma(const string &sensor_name,
        const vector<double> &sigma_trans_vec, 
        const vector<double> &sigma_rot_vec);

    // 3. get fused result
    bool fuse(Summary &sum, 
        double& x, double& y, double& z, 
        double& qw, double& qx, double& qy, double& qz);

    // 4. gebug usage, get the value of the whole window
    bool getWinPoses(vector<vector<double>>& poses);

private:
    void* impl_;
};

} //namespace
#endif
