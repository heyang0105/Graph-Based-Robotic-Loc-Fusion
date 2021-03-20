#include"fusion_impl.h"

#include<assert.h>
#include<string>
#include<iostream>

#include"fusion/FusionManager.h"

const char* LIB_INFO = "VERSION: 0.0.1 , 2021-03-19";

using namespace std;
namespace fusion{

bool Options::check(){
    bool res = true;
    if(loc_sensor_num < 1){
        cerr<<"Must have more than one loc data \n";
        res = res && false;
    }
    
    if(odom_sensor_num < 2){
        cerr<<"Must have more than two odom data \n";
        res = res && false;
    }

    if(window_size < 0){
        cerr<<"Wrong window size \n";
        res = res && false;
    }

    return res;
}

void Summary::printSummary(){
    // TODO
}


FusionImpl::FusionImpl(Options& opt) :
    impl_(nullptr){
    assert(opt.check());
    impl_ = new FusionManager(opt);
}

FusionImpl::~FusionImpl(){
    if(impl_ != nullptr){
        delete impl_;
    }
    impl_ = nullptr;
}

//----------------------IMPLEMENTATION------------------------------
int FusionImpl::registerSensor(const string &sensor_name, const DataType &type,
    const double &sigma_trans, const double &sigma_rot){

    int res_val = -1;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->registerSensor(sensor_name, type, sigma_trans, sigma_rot);
    return res_val;
}

int FusionImpl::registerExternalParams(const string &sensor_name_0, const string &sensor_name_1,
    const double& x, const double& y, const double& z, 
    const double& qw, const double& qx, const double& qy, const double& qz,
    const vector<double> &sigma_trans_vec, const vector<double> &sigma_rot_vec,
    const double &switch_prior_info){
        
    int res_val = -1;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->registerExternalParams(sensor_name_0, sensor_name_1,
       x,  y,  z, 
        qw, qx, qy, qz,
        sigma_trans_vec, sigma_rot_vec,
        switch_prior_info);
    return res_val;
}

// 2. Set Data: Add the Data
bool FusionImpl::addSensorData(const string &sensor_name, 
    const double& timestamp, 
    const double& x, const double& y, const double& z, 
    const double& qw, const double& qx, const double& qy, const double& qz){

    bool res_val = false;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->addSensorData(sensor_name, timestamp, 
        x, y, z, qw, qx, qy, qz);
    return res_val;
}

/* the sensor's variance may be dynamicly estiamted during the process */
bool  FusionImpl::setSensorSigma(const string &sensor_name,
    const vector<double> &sigma_trans_vec, 
    const vector<double> &sigma_rot_vec){

    bool res_val = false;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->setSensorSigma(sensor_name, sigma_trans_vec, sigma_rot_vec);
    return res_val;
}

// 3. get fused result
bool FusionImpl::fuse(Summary &sum, 
    double& x, double& y, double& z, 
    double& qw, double& qx, double& qy, double& qz){

    bool res_val = false;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->fuse(sum,
       x, y, z, qw, qx, qy, qz);
    return res_val;
}

// 4. debug get the window
bool FusionImpl::getWinPoses(vector<vector<double>>& poses){
    bool res_val = false;
    FusionManager* impl = (FusionManager*)(impl_);
    res_val = impl->getWinPoses(poses);
    return res_val;
} 

}//namespace