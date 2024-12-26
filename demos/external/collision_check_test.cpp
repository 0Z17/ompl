//
// Created by 91520 on 24-12-16.
//

#include "CollisionChecker.h"
#include <iostream>

namespace cc = collision_checker;

int main()
{
    cc::CollisionChecker checker;
    checker.setConfig(std::vector<double>{1,1,1,0,M_PI/2});
    return 0;
}