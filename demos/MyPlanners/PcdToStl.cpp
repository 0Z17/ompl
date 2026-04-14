// ============================================================
//  PcdToStl.cpp — 读取点云文件，拟合 NURBS 曲面，导出 STL
// ============================================================

#include "nurbs.h"
#include <iostream>
#include <cstdlib>
#include <cstring>

static void printUsage(const char* prog)
{
    std::cerr << "Usage: " << prog
              << " --input <file.pcd> --output <file.stl> [--resolution <N>]\n"
              << "  -i, --input      输入 PCD 文件路径（必填）\n"
              << "  -o, --output     输出 STL 文件路径（必填）\n"
              << "  -r, --resolution 网格分辨率，默认 50\n"
              << "  -h, --help       显示此帮助\n";
}

int main(int argc, char** argv)
{
    std::string pcd_file;
    std::string stl_file;
    double      resolution = 50.0;

    for (int i = 1; i < argc; ++i) {
        auto arg = [&](const char* s) { return std::strcmp(argv[i], s) == 0; };
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[PcdToStl] ERROR: " << argv[i] << " 需要一个参数\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if (arg("-i") || arg("--input"))           pcd_file   = next();
        else if (arg("-o") || arg("--output"))      stl_file   = next();
        else if (arg("-r") || arg("--resolution"))  resolution = std::stod(next());
        else if (arg("-h") || arg("--help"))        { printUsage(argv[0]); return 0; }
        else { std::cerr << "[PcdToStl] 未知选项: " << argv[i] << "\n"; printUsage(argv[0]); return 1; }
    }

    if (pcd_file.empty() || stl_file.empty()) {
        std::cerr << "[PcdToStl] ERROR: --input 和 --output 为必填项\n";
        printUsage(argv[0]);
        return 1;
    }

    std::cout << "[PcdToStl] Input PCD : " << pcd_file   << std::endl;
    std::cout << "[PcdToStl] Output STL: " << stl_file   << std::endl;
    std::cout << "[PcdToStl] Resolution: " << resolution << std::endl;

    // 1. 从 PCD 文件构造 Nurbs 对象
    surface_reconstructor::Nurbs nurbs(pcd_file);

    // 2. 拟合曲面
    std::cout << "[PcdToStl] Fitting NURBS surface..." << std::endl;
    if (nurbs.fitSurfaceByCorners(-Eigen::Vector3d::UnitX()) != 0) {
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
