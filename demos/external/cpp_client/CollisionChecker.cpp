//
// Created by 91520 on 24-12-16.
//

#include "CollisionChecker.h"
#include "RemoteAPIClient.h"
#include <iostream>

collision_checker::CollisionChecker::CollisionChecker(const std::string &host) {
    host_ = host;
    RemoteAPIClient client(host);
    auto sim = client.getObject().sim();
    auto handle =sim.getObject("./skyvortex");
    std::cout << handle << std::endl;
    std::cout << "CollisionChecker created" << std::endl;
}

std::string collision_checker::CollisionChecker::getHost() const {
    return host_;
}

