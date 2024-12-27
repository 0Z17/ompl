//
// Created by 91520 on 24-12-16.
//

#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include "RemoteAPIClient.h"
#include <string>
#include <Eigen/Dense>

namespace collision_checker_coppeliaSim {

using Vector5d = Eigen::Matrix<double, 5, 1>;

class CollisionChecker {
public:
    explicit CollisionChecker(const std::string &host = "172.31.16.1", std::string name = "/skyvortex",
                                std::string jointName = "/operator_1_joint");
    ~CollisionChecker() = default;

    /** @brief Get the host IP address of the CoppeliaSim application. */
    std::string getHost() const;

    /** @brief Set the configuration of the UAM */
    void setConfig(const Vector5d &config);

    void setConfig(const std::vector<double> &config);

    /** @brief Get the current position of the UAM */
    void getConfig(Vector5d &config);
    void getConfig(std::vector<double> &config);

    /** @brief Check if the UAM is colliding with any object in the simulation.
     *  @return True if the UAM is colliding, false otherwise. */
    bool checkCollision();

protected:

    /** @brief The host IP address of the CoppeliaSim application. */
    std::string host_;

    /** @brief The handle of the remote API object */
    RemoteAPIObject::sim sim_{nullptr};

    /** @brief The collection of the UAM */
    int64_t coll_;

    /** @brief The handle of the base */
    int64_t baseHandle_;

    /** @brief The handle of the joint */
    int64_t jointHandle_;

    /** @brief The offset of the joint angle from the default position to home position */
    double jointAngleOffset_{M_PI/6};

    /** @brief The length of the UAM's link */
    double linkLength_{0.95};
};
} // namespace collision_checker



#endif //COLLISIONCHECKER_H
