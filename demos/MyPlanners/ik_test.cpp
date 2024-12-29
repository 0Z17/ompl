#include "invkin.h"
#include "nurbs.h"
#include <pcl/io/pcd_io.h>
#include <ompl/util/Time.h>
#include <random>
#include "mujoco_client.h"

namespace dp = dynamic_planning;
namespace sr = surface_reconstructor;
namespace mc = mujoco_client;

int main()
{
    // Load the point cloud
    // std::string pcdFile = "/home/wsl/proj/pcl/test/milk.pcd";
    std::string pcdFile = "/home/wsl/proj/skyvortex_mujoco/assets/NURBS.pcd";
    const auto nurbs = new sr::Nurbs(pcdFile);
    nurbs->fitSurface();
    auto ik = new dp::InvKin(nurbs);
    ik->setLinkLength(1.0);

    double time_sum = 0.0;
    ompl::time::point start_time = ompl::time::now();

    // Generate a random target point
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    int num = 5000;

    std::string modelFile = "/home/wsl/proj/skyvortex_mujoco/scene.xml";
    mujoco_client::MujocoClient client(modelFile.c_str());

    unsigned int count = 0;

    for (int i = 0; i < num; i++)
    {
        if (count % 100 == 0)
        {
            // auto q = ik->xToQ(dis(gen), dis(gen));
            auto q = ik->xToQ(0.2, 0.2);
            std::vector<double> q_vec(q.data(), q.data() + q.size());
            client.setConfig(q_vec);
            std::cout << "goal is collision: " << (client.isCollision() ? "true" : "false") << std::endl;
            client.printCollisionInfo();
            count = 0;
        }
        count++;
        client.render();
        glfwPollEvents();
        // ik->dxToDqe(dis(gen), dis(gen),0,1);
        // ik->dxToDqe(dis(gen), dis(gen),1,0);
        time_sum += ompl::time::seconds(ompl::time::now() - start_time);
        start_time = ompl::time::now();
    }
    std::cout << "Sum of time of ik solve: " << time_sum << " seconds" << std::endl;
    std::cout << "Average time of ik solve: " << time_sum / num << " seconds" << std::endl;

    start_time = ompl::time::now();

    for (int i = 0; i < num; i++)
    {

        // ik->xToQs(dis(gen), dis(gen));
        ik->dxToDqe(dis(gen), dis(gen),0,1);
        ik->dxToDqe(dis(gen), dis(gen),1,0);
        time_sum += ompl::time::seconds(ompl::time::now() - start_time);
        start_time = ompl::time::now();
    }
    std::cout << "Sum of time of dxToDqe: " << time_sum << " seconds" << std::endl;
    std::cout << "Average time of dxToDqe: " << time_sum / num << " seconds" << std::endl;

}