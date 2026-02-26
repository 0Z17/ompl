// ============================================================
//  PcdToStl.cpp — 读取点云文件，拟合 NURBS 曲面，导出 STL
// ============================================================

#include "nurbs.h"
#include <iostream>
#include <cstdlib>

int main(int argc, char** argv)
{
    // --- 输入/输出路径（可通过环境变量或命令行参数覆盖）---
    std::string pcd_file  = "/home/wsl/proj/T_mech_R1/S1/blade_segment.pcd";
    std::string stl_file  = "/home/wsl/proj/T_mech_R1/S1/blade_segment.stl";
    double      resolution = 50.0;

    // 命令行参数：PcdToStl <pcd_file> [stl_file] [resolution]
    if (argc >= 2) pcd_file   = argv[1];
    if (argc >= 3) stl_file   = argv[2];
    if (argc >= 4) resolution = std::stod(argv[3]);

    // 环境变量覆盖
    if (const char* env = std::getenv("INPUT_PCD"))  pcd_file   = env;
    if (const char* env = std::getenv("OUTPUT_STL")) stl_file   = env;

    std::cout << "[PcdToStl] Input PCD : " << pcd_file   << std::endl;
    std::cout << "[PcdToStl] Output STL: " << stl_file   << std::endl;
    std::cout << "[PcdToStl] Resolution: " << resolution << std::endl;

    // 1. 从 PCD 文件构造 Nurbs 对象
    surface_reconstructor::Nurbs nurbs(pcd_file);

    // 2. 拟合曲面
    std::cout << "[PcdToStl] Fitting NURBS surface..." << std::endl;
    if (nurbs.fitSurface(-Eigen::Vector3d::UnitX()) != 0) {
        std::cerr << "[PcdToStl] ERROR: fitSurface() failed." << std::endl;
        return 1;
    }
    std::cout << "[PcdToStl] Surface fitted successfully." << std::endl;

    // 3. 导出 STL
    std::cout << "[PcdToStl] Saving STL..." << std::endl;
    if (!nurbs.saveSurfaceAsStl(stl_file, resolution)) {
        std::cerr << "[PcdToStl] ERROR: saveSurfaceAsStl() failed." << std::endl;
        return 1;
    }
    std::cout << "[PcdToStl] STL saved to: " << stl_file << std::endl;

    return 0;
}
