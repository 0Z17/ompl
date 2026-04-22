#pragma once
// ============================================================
//  FastCollisionChecker.h
//  Lightweight primitive-based collision checker.
//  Robot capsules vs. axis-aligned obstacle boxes (AABBs).
//  No MuJoCo / PCL dependency.
//
//  Robot config: q = [x, y, z, psi, theta]
//    x,y,z  : base_link world position
//    psi    : base_link yaw (rad)
//    theta  : operator_1_joint angle (rad)
//
//  Usage:
//    FastCollisionChecker checker;
//    checker.addBox({cx,cy,cz}, {hx,hy,hz});   // add obstacle
//    if (checker.isCollision(q)) ...
//    checker.printStats();                       // print timing
// ============================================================

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <chrono>
#include <atomic>
#include <cstdio>

class FastCollisionChecker
{
public:
    struct Box {
        Eigen::Vector3d center;
        Eigen::Vector3d halfSize;
    };

    struct Capsule {
        Eigen::Vector3d localCenter;
        Eigen::Matrix3d localRot;     // local z-axis = cylinder axis
        double radius;
        double halfLen;
        bool onOperator;
    };

    FastCollisionChecker() { buildPrimitives(); }

    void addBox(const Eigen::Vector3d& center, const Eigen::Vector3d& halfSize)
    {
        boxes_.push_back({center, halfSize});
    }

    void clearBoxes() { boxes_.clear(); }

    // Returns true if any capsule overlaps any box.
    // If no boxes added, always returns false (zero overhead).
    bool isCollision(const double* q) const
    {
        const auto t0 = std::chrono::steady_clock::now();
        const bool result = boxes_.empty() ? false : isCollisionImpl(q);
        const auto t1 = std::chrono::steady_clock::now();
        totalNs_.fetch_add(
            std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
        callCount_.fetch_add(1);
        return result;
    }

    bool isCollision(const std::vector<double>& q) const
    {
        return isCollision(q.data());
    }

    void printStats() const
    {
        const long   calls   = callCount_.load();
        const double totalUs = totalNs_.load() / 1000.0;
        const double avgUs   = calls > 0 ? totalUs / calls : 0.0;
        std::printf("[FastCollisionChecker] calls=%ld  total=%.3f ms  avg=%.4f us\n",
                    calls, totalUs / 1000.0, avgUs);
    }

    void resetStats()
    {
        callCount_.store(0);
        totalNs_.store(0);
    }

private:
    std::vector<Box>     boxes_;
    std::vector<Capsule> capsules_;
    mutable std::atomic<long> callCount_{0};
    mutable std::atomic<long> totalNs_{0};

    static constexpr double jointOffset_ = -M_PI / 6.0;

    bool isCollisionImpl(const double* q) const
    {
        const double psi   = q[3];
        const double theta = q[4];

        const Eigen::Matrix3d Rbase = Rz(psi);
        const Eigen::Vector3d tbase(q[0], q[1], q[2]);

        static const Eigen::Vector3d opOffset(0.000870575, 0.000171621, 0.0284551);
        static const Eigen::Vector3d opAxis =
            Eigen::Vector3d(-0.0169901, -0.999856, 0.0).normalized();
        const Eigen::Matrix3d Rop =
            Rbase * Eigen::AngleAxisd(theta + jointOffset_, opAxis).toRotationMatrix();
        const Eigen::Vector3d top = tbase + Rbase * opOffset;

        for (const auto& cap : capsules_)
        {
            const Eigen::Matrix3d& R = cap.onOperator ? Rop   : Rbase;
            const Eigen::Vector3d& t = cap.onOperator ? top   : tbase;

            const Eigen::Vector3d center_w = t + R * cap.localCenter;
            const Eigen::Vector3d axis_w   = (R * cap.localRot).col(2);
            const Eigen::Vector3d A = center_w - axis_w * cap.halfLen;
            const Eigen::Vector3d B = center_w + axis_w * cap.halfLen;

            for (const auto& box : boxes_)
                if (capsuleBoxOverlap(A, B, cap.radius, box))
                    return true;
        }
        return false;
    }

    static Eigen::Matrix3d Rz(double a)
    {
        const double c = std::cos(a), s = std::sin(a);
        Eigen::Matrix3d R;
        R << c, -s, 0,
             s,  c, 0,
             0,  0, 1;
        return R;
    }

    static Eigen::Matrix3d quatToMat(double w, double x, double y, double z)
    {
        return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
    }

    void buildPrimitives()
    {
        // 4 landing legs — radius=0.030, halfLen=0.175
        capsules_.push_back({ {0.030677,  -0.25355, -0.17398},
            quatToMat(0.95305, -0.122402, -0.152147, -0.231442), 0.030, 0.175, false });
        capsules_.push_back({ {-0.23492,  -0.10021, -0.17398},
            quatToMat(0.718355, 0.152077, 0.119159, 0.668311),   0.030, 0.175, false });
        capsules_.push_back({ {0.23492,    0.10021, -0.17398},
            quatToMat(0.718355, -0.152077, -0.119159, 0.668311), 0.030, 0.175, false });
        capsules_.push_back({ {-0.030677,  0.25355, -0.17398},
            quatToMat(0.95305, 0.122402, 0.152147, -0.231442),   0.030, 0.175, false });

        // 6 rotor discs — radius=0.228, halfLen=0.02
        capsules_.push_back({ {0.44207,   0.2778,    0.033856},
            quatToMat(0.922577, -0.190485, -0.184513, -0.280218), 0.228, 0.02, false });
        capsules_.push_back({ {0.019547,  0.52175,   0.033856},
            quatToMat(0.683013, 0.183012, 0.183012, 0.683013),    0.228, 0.02, false });
        capsules_.push_back({ {-0.46162,  0.24395,   0.033856},
            quatToMat(0.922493, 0.190542, -0.184455, 0.280495),   0.228, 0.02, false });
        capsules_.push_back({ {-0.46162, -0.24395,   0.033856},
            quatToMat(0.922577, -0.190485, -0.184513, -0.280218), 0.228, 0.02, false });
        capsules_.push_back({ {0.019547, -0.52175,   0.033856},
            quatToMat(0.683013, 0.183012, 0.183012, 0.683013),    0.228, 0.02, false });
        capsules_.push_back({ {0.442074, -0.277802,  0.0338557},
            quatToMat(0.922493, 0.190542, -0.184455, 0.280495),   0.228, 0.02, false });

        // Operator arm — radius=0.04, halfLen=0.4
        capsules_.push_back({ {0.4, 0.0, 0.235},
            quatToMat(0.86271, 0.0129416, 0.505476, -0.00758272), 0.04, 0.4, true });
    }

    static bool capsuleBoxOverlap(const Eigen::Vector3d& A,
                                  const Eigen::Vector3d& B,
                                  double r,
                                  const Box& box)
    {
        const Eigen::Vector3d d  = B - A;
        const double len2 = d.squaredNorm();
        double t = (len2 > 1e-12) ? (box.center - A).dot(d) / len2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        const Eigen::Vector3d closest = A + t * d;

        const Eigen::Vector3d delta = closest - box.center;
        const Eigen::Vector3d clamped(
            std::max(-box.halfSize.x(), std::min(box.halfSize.x(), delta.x())),
            std::max(-box.halfSize.y(), std::min(box.halfSize.y(), delta.y())),
            std::max(-box.halfSize.z(), std::min(box.halfSize.z(), delta.z()))
        );
        return (closest - (box.center + clamped)).squaredNorm() <= r * r;
    }
};
