// ============================================================
//  NurbsClosestPoint.cpp
//  Load the same fitted surface as PlanningCompare.cpp and
//  query the closest surface parameter (u, v) for a 3-D point.
//  Usage: demo_NurbsClosestPoint <config.yaml> <x> <y> <z>
// ============================================================

#include "PlanningConfig.h"
#include "nurbs.h"

#include <Eigen/Dense>
#include <charconv>
#include <cstdlib>
#include <iostream>
#include <string>

namespace pc = planning_config;
namespace sr = surface_reconstructor;

namespace
{

bool parseDouble(const char *text, double &value)
{
    const std::string input(text);
    const char *begin = input.data();
    const char *end = begin + input.size();

    auto result = std::from_chars(begin, end, value);
    return result.ec == std::errc{} && result.ptr == end;
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program << " <config.yaml> <x> <y> <z>\n";
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        printUsage(argv[0]);
        return 1;
    }

    pc::PlanningConfig cfg;
    if (!cfg.loadFromFile(argv[1]))
    {
        std::cerr << "Failed to load config: " << argv[1] << "\n";
        return 1;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!parseDouble(argv[2], x) || !parseDouble(argv[3], y) || !parseDouble(argv[4], z))
    {
        std::cerr << "Invalid 3-D point. x, y, z must be numeric values.\n";
        printUsage(argv[0]);
        return 1;
    }

    if (cfg.output.surface_normal.size() != 3)
    {
        std::cerr << "Invalid config: output.surface_normal must contain 3 values.\n";
        return 1;
    }

    sr::Nurbs nurbs(cfg.files.pcd_file);
    const Eigen::Vector3d surfNormal(cfg.output.surface_normal[0],
                                     cfg.output.surface_normal[1],
                                     cfg.output.surface_normal[2]);

    // Keep the fitting path identical to PlanningCompare.cpp.
    if (nurbs.fitSurface(surfNormal) != 0)
    {
        std::cerr << "Failed to fit NURBS surface.\n";
        return 1;
    }

    const Eigen::Vector3d queryPoint(x, y, z);
    double u = 0.0;
    double v = 0.0;
    if (nurbs.getClosestPoint(queryPoint, u, v) != 0)
    {
        std::cerr << "Failed to compute closest point on fitted surface.\n";
        return 1;
    }

    Eigen::Vector3d closestPoint;
    if (nurbs.getPos(u, v, closestPoint) != 0)
    {
        std::cerr << "Failed to evaluate closest surface point.\n";
        return 1;
    }

    std::cout.precision(17);
    std::cout << "u,v,x,y,z,distance\n"
              << u << "," << v << ","
              << closestPoint[0] << "," << closestPoint[1] << "," << closestPoint[2] << ","
              << (queryPoint - closestPoint).norm() << "\n";

    return 0;
}
