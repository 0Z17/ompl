//
// Created by 91520 on 24-12-16.
//

#include "CollisionChecker.h"
#include "RemoteAPIClient.h"


namespace cc = collision_checker_coppeliaSim;

collision_checker_coppeliaSim::CollisionChecker::CollisionChecker(const std::string &host, std::string name, std::string jointName)
    : host_(host), sim_(RemoteAPIClient(host).getObject().sim()){
    baseHandle_ = sim_.getObject(name);
    jointHandle_ = sim_.getObject(jointName);
    coll_ = sim_.createCollection();
    sim_.addItemToCollection(coll_, sim_.handle_tree, baseHandle_, 0B00);
}

std::string collision_checker_coppeliaSim::CollisionChecker::getHost() const {
    return host_;
}

void collision_checker_coppeliaSim::CollisionChecker::getConfig(cc::Vector5d &config){
    std::vector<double> ori = sim_.getObjectOrientation(baseHandle_);
    double yaw = ori[2];
    std::vector<double> pos = sim_.getObjectPosition(baseHandle_);
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    double jointPos = sim_.getJointPosition(jointHandle_);
    cc::Vector5d conf;
    conf << x, y, z, yaw, jointPos;
    config = conf;
}

void collision_checker_coppeliaSim::CollisionChecker::getConfig(std::vector<double> &config){
    std::vector<double> ori = sim_.getObjectOrientation(baseHandle_);
    double yaw = ori[2];
    std::vector<double> pos = sim_.getObjectPosition(baseHandle_);
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    double jointPos = sim_.getJointPosition(jointHandle_);
    config = {x, y, z, yaw, jointPos};
}

void collision_checker_coppeliaSim::CollisionChecker::setConfig(const cc::Vector5d &config){
    std::vector<double> pos = {config(0), config(1), config(2)};
    std::vector<double> ori = {0.0, 0.0, config(3)};
    const double jointPos = config(4);
    sim_.setObjectPosition(baseHandle_, pos);
    sim_.setObjectOrientation(baseHandle_, ori);
    sim_.setJointPosition(jointHandle_, jointPos);
}

void collision_checker_coppeliaSim::CollisionChecker::setConfig(const std::vector<double> &config){
    std::vector<double> pos = {config[0], config[1], config[2]};
    std::vector<double> ori = {0.0, 0.0, config[3]};
    const double jointPos = config[4];
    sim_.setObjectPosition(baseHandle_, pos);
    sim_.setObjectOrientation(baseHandle_, ori);
    sim_.setJointPosition(jointHandle_, jointPos);
}

bool collision_checker_coppeliaSim::CollisionChecker::checkCollision() {
    auto [result, collision] = sim_.checkCollision(coll_, sim_.handle_all);
    return !collision.empty();
}



