// ============================================================
//  PcdToMeshCsv.cpp — 读取点云，拟合 NURBS 曲面，导出网格 CSV
//
//  CSV 格式：每行一个格点 (x,y,z)，按 u 外层、v 内层行优先排列。
//  共 (N+1)*(N+1) 行，可在 MATLAB 中直接 reshape 后用 mesh() 绘制：
//
//    data = readmatrix('surface_mesh.csv');
//    N    = <分辨率>;          % 与 --resolution 参数一致
//    X    = reshape(data(:,1), N+1, N+1);
//    Y    = reshape(data(:,2), N+1, N+1);
//    Z    = reshape(data(:,3), N+1, N+1);
//    figure; mesh(X, Y, Z);
//    xlabel('X'); ylabel('Y'); zlabel('Z');
// ============================================================

#include "nurbs.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <iomanip>

static void printUsage(const char* prog)
{
    std::cerr << "Usage: " << prog
              << " --input <file.pcd> --output <file.csv> [--resolution <N>]\n"
              << "  -i, --input      输入 PCD 文件路径（必填）\n"
              << "  -o, --output     输出 CSV 文件路径（必填）\n"
              << "  -r, --resolution UV 网格分辨率，默认 20（生成 21x21 格点）\n"
              << "  -h, --help       显示此帮助\n";
}

int main(int argc, char** argv)
{
    std::string pcd_file;
    std::string csv_file;
    int         resolution = 20;

    for (int i = 1; i < argc; ++i) {
        auto arg  = [&](const char* s) { return std::strcmp(argv[i], s) == 0; };
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[PcdToMeshCsv] ERROR: " << argv[i] << " 需要一个参数\n";
                std::exit(1);
            }
            return argv[++i];
        };

        if      (arg("-i") || arg("--input"))       pcd_file   = next();
        else if (arg("-o") || arg("--output"))       csv_file   = next();
        else if (arg("-r") || arg("--resolution"))   resolution = std::atoi(next());
        else if (arg("-h") || arg("--help"))         { printUsage(argv[0]); return 0; }
        else {
            std::cerr << "[PcdToMeshCsv] 未知选项: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (pcd_file.empty() || csv_file.empty()) {
        std::cerr << "[PcdToMeshCsv] ERROR: --input 和 --output 为必填项\n";
        printUsage(argv[0]);
        return 1;
    }
    if (resolution < 1) {
        std::cerr << "[PcdToMeshCsv] ERROR: --resolution 必须 >= 1\n";
        return 1;
    }

    std::cout << "[PcdToMeshCsv] Input PCD  : " << pcd_file   << "\n"
              << "[PcdToMeshCsv] Output CSV : " << csv_file   << "\n"
              << "[PcdToMeshCsv] Resolution : " << resolution
              << "  (" << (resolution+1) << "x" << (resolution+1) << " = "
              << (resolution+1)*(resolution+1) << " 格点)\n";

    // 1. 构造 Nurbs 对象
    surface_reconstructor::Nurbs nurbs(pcd_file);

    // 2. 拟合曲面
    std::cout << "[PcdToMeshCsv] Fitting NURBS surface...\n";
    if (nurbs.fitSurfaceByCorners(-Eigen::Vector3d::UnitX()) != 0) {
        std::cerr << "[PcdToMeshCsv] ERROR: fitSurfaceByCorners() failed.\n";
        return 1;
    }
    std::cout << "[PcdToMeshCsv] Surface fitted successfully.\n";

    // 3. 采样格点并写 CSV
    std::ofstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "[PcdToMeshCsv] ERROR: 无法打开输出文件: " << csv_file << "\n";
        return 1;
    }

    // 文件头：注明分辨率，方便 MATLAB 端读取
    file << "# resolution=" << resolution << "\n";
    file << "x,y,z\n";
    file << std::fixed << std::setprecision(8);

    int success = 0, fail = 0;
    for (int i = 0; i <= resolution; ++i) {
        double u = static_cast<double>(i) / resolution;
        for (int j = 0; j <= resolution; ++j) {
            double v = static_cast<double>(j) / resolution;
            Eigen::Vector3d pos;
            if (nurbs.getPos(u, v, pos) == 0) {
                file << pos[0] << "," << pos[1] << "," << pos[2] << "\n";
                ++success;
            } else {
                // 保留行占位，避免 reshape 错位；填 NaN 让 MATLAB 自动跳过
                file << "NaN,NaN,NaN\n";
                ++fail;
            }
        }
    }

    file.close();
    std::cout << "[PcdToMeshCsv] CSV saved to: " << csv_file << "\n"
              << "[PcdToMeshCsv] 成功格点: " << success
              << "，失败(NaN): " << fail << "\n";

    // 4. 打印 MATLAB 使用提示
    std::cout << "\n--- MATLAB 使用方法 ---\n"
              << "  data = readmatrix('" << csv_file << "', 'NumHeaderLines', 2);\n"
              << "  N    = " << resolution << ";\n"
              << "  X    = reshape(data(:,1), N+1, N+1);\n"
              << "  Y    = reshape(data(:,2), N+1, N+1);\n"
              << "  Z    = reshape(data(:,3), N+1, N+1);\n"
              << "  figure; mesh(X, Y, Z);\n"
              << "  xlabel('X'); ylabel('Y'); zlabel('Z');\n"
              << "-----------------------\n";

    return 0;
}
