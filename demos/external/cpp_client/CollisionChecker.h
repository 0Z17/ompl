//
// Created by 91520 on 24-12-16.
//

#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <string>
namespace collision_checker {

class CollisionChecker {
public:
    CollisionChecker(const std::string &host = "172.31.16.1");
    ~CollisionChecker() = default;

    std::string getHost() const;
protected:
    std::string host_;
};
} // namespace collision_checker



#endif //COLLISIONCHECKER_H
