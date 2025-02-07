#include "invkin.h"
#include <cmath>

using namespace dynamic_planning;
namespace sr = surface_reconstructor;

InvKin::InvKin(sr::Nurbs* nurbs) : nurbs_(nurbs) {}

void InvKin::setLinkLength(const double link_length)
{
    link_length_ = link_length;
}

void InvKin::updateSurface(sr::Nurbs* surface)
{
    nurbs_ = surface;
}

Vector6d InvKin::xToQs(const double u, const double v) const
{
    Vector6d qe;
    Eigen::Vector3d pos;
    Eigen::Vector3d normal;

    nurbs_->getPos(u, v, pos);
    nurbs_->getNormal(u, v, normal);
    qe << pos, normal;
    return qe;
} 

Vector5d InvKin::qsToQe(const Vector6d& qs)
{
    // calculate the yaw and the joint angle of the UAM
    const Eigen::Vector3d ps = qs.head<3>();
    Eigen::Vector3d n = qs.tail<3>();
    const double psi_e = std::atan2(-n(1), -n(0));
    const double theta_e = std::atan2(-n(2), n.head<2>().norm());

    // calculate the joint angle of the UAM
    Eigen::VectorXd state(5);
    state << ps, psi_e, theta_e; 

    return state;
}

Vector5d InvKin::qsToQ(const Vector6d& qs) const {
    Vector5d q = qsToQe(qs);
    q(0) = q(0) + qs(3) * link_length_;
    q(1) = q(1) + qs(4) * link_length_;
    q(2) = q(2) + qs(5) * link_length_;
    return q;
}

Vector5d InvKin::xToQ(double u, double v) const {
    Vector6d qs = xToQs(u, v);
    return qsToQ(qs);
}

Eigen::Vector3d InvKin::getDpos(double u, double v, double du, double dv) const
{
    Eigen::Vector3d dps_u, dps_v;
    nurbs_->getPosDeriv(u, v, dps_u, dps_v);
    Eigen::Vector3d dps = dps_u * du + dps_v * dv;
    return dps;
}

double InvKin::getDpsi(Eigen::Vector3d& n, Eigen::Vector3d& dn_u, Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double dpsi = n(0) / (n(0) * n(0) + n(1) * n(1)) * dn(1) -
                        n(1) / (n(0) * n(0) + n(1) * n(1)) * dn(0);
    return dpsi;
}

double InvKin::getDtheta(Eigen::Vector3d& n, const Eigen::Vector3d& dn_u, const Eigen::Vector3d& dn_v,
                                double du, double dv)
{
    Eigen::Vector3d dn = dn_u * du + dn_v * dv;
    const double n_t = std::sqrt(n(0) * n(0) + n(1) * n(1));

    double dtheta = -n_t / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) * dn(2) +
                    n(2) / (n(0) * n(0) + n(1) * n(1) + n(2) * n(2)) *
                    (n(0) / n_t * dn(0) + n(1) / n_t * dn(1));

    return dtheta;
}

Vector5d InvKin::dxToDqe(double u, double v, double du, double dv) const
{
    Eigen::Vector3d n;
    nurbs_->getNormal(u, v, n);
    Eigen::Vector3d dn_u, dn_v;
    nurbs_->getDNormal(u, v, dn_u, dn_v);

    const Eigen::Vector3d dpe = getDpos(u, v, du, dv);
    const double dpsi = getDpsi(n, dn_u, dn_v, du, dv);
    const double dtheta = getDtheta(n, dn_u, dn_v, du, dv);

    Vector5d dqe;
    dqe << dpe, dpsi, dtheta;
    return dqe;
}

std::pair<Vector5d,Vector5d> InvKin::getTangentVec(double u, double v) const {
    const auto dqe_u = InvKin::dxToDqe(u,v,1,0);
    const auto dqe_v = InvKin::dxToDqe(u,v,0,1);

    return {dqe_u, dqe_v};
}
