#include "mujoco_client.h"
#include <vector>
#include <random>
#include "nurbs.h"
#include "invkin.h"

namespace sr = surface_reconstructor;
namespace dp = dynamic_planning;

int main() {
    const char* model_path = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
    mujoco_client::MujocoClient client(model_path);

    std::string pcdFile = "/home/wsl/proj/planning_ws/src/surface_reconstructor/data/pointcloud_plane.pcd";
    std::string modelFile = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
    // auto nurbs = new sr::Nurbs(pcdFile);
    // auto ik = new dp::InvKin(nurbs);
    std::shared_ptr<sr::Nurbs> nurbs = std::make_shared<sr::Nurbs>(pcdFile);
    std::shared_ptr<dp::InvKin> ik = std::make_shared<dp::InvKin>(nurbs.get());
    nurbs->fitSurface();


    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> pos_rnd(-1.0f, 1.0f);


    while (true) {
        client.setConfig(std::vector<double>{pos_rnd(gen), pos_rnd(gen), pos_rnd(gen), pos_rnd(gen), pos_rnd(gen)});
        client.render();
        glfwPollEvents();
    };

    return 0;
}