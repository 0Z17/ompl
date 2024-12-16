//
// Created by 91520 on 24-12-16.
//

#include "CollisionChecker.h"
#include <iostream>

namespace cc = collision_checker;

int main()
{
    std::cout << "Hello, world!" << std::endl;
    cc::CollisionChecker checker;
    std::cout << checker.getHost() << std::endl;
    return 0;
}