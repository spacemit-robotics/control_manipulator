/*
* Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
* SPDX-License-Identifier: Apache-2.0
*
* @file kinematics_pinocchio.cpp
* @brief Pinocchio-based kinematics solver (FK / IK)
*
* Uses the Pinocchio rigid-body dynamics library:
*   - FK: forwardKinematics + updateFramePlacements
*   - IK: CLIK (Closed-Loop Inverse Kinematics) with damped least-squares
*
* Build requirement:
*   find_package(pinocchio REQUIRED)
*   target_link_libraries(... pinocchio::pinocchio)
*
* Registered as "pinocchio" via REGISTER_KIN_SOLVER.
*/

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

extern "C" {
#include "../include/kinematics_interface.h"
}

/* ==========================================================================
* Logging (printf-based, matching framework convention)
* ========================================================================== */

#define PIN_TAG  "[KIN-Pinocchio] "
#define pin_info(fmt, ...)  fprintf(stdout, PIN_TAG fmt "\n", ##__VA_ARGS__)
#define pin_err(fmt, ...)   fprintf(stderr, PIN_TAG fmt "\n", ##__VA_ARGS__)

/* ==========================================================================
* Private data
* ========================================================================== */

struct pinocchio_priv {
    pinocchio::Model model;
    pinocchio::Data  data;

    std::string urdf_path;
    std::string base_link;
    std::string tip_link;

    pinocchio::FrameIndex tip_frame_id;
    int num_joints;          /* model.nq */

    /* Default IK parameters */
    double ik_epsilon;       /* convergence tolerance (m) */
    int    ik_max_iter;      /* max CLIK iterations      */
};

/* ==========================================================================
* SE3 ↔ kin_pose_t helpers
* ========================================================================== */

static pinocchio::SE3 pose_to_se3(const kin_pose_t *p) {
    Eigen::Quaterniond q(p->qw, p->qx, p->qy, p->qz);
    q.normalize();
    return pinocchio::SE3(q.toRotationMatrix(),
                            Eigen::Vector3d(p->x, p->y, p->z));
}

static void se3_to_pose(const pinocchio::SE3 &se3, kin_pose_t *p) {
    const auto &t = se3.translation();
    p->x = t[0];  p->y = t[1];  p->z = t[2];

    Eigen::Quaterniond q(se3.rotation());
    q.normalize();
    p->qw = q.w();  p->qx = q.x();  p->qy = q.y();  p->qz = q.z();
}

/* ==========================================================================
* Pose-error: position ± orientation
* ========================================================================== */

static Eigen::Matrix<double, 6, 1> compute_pose_error(
    const pinocchio::SE3 &current,
    const pinocchio::SE3 &target,
    bool position_only) {
    Eigen::Matrix<double, 6, 1> err;
    err.setZero();
    err.head<3>() = target.translation() - current.translation();
    if (!position_only) {
            pinocchio::SE3 dse3 = target.actInv(current);
            err.tail<3>() = pinocchio::log3(dse3.rotation());
    }
    return err;
}

/* ==========================================================================
* kin_joints_t → Eigen::VectorXd (pad with zeros to model size)
* ========================================================================== */

static Eigen::VectorXd joints_to_eigen(const kin_joints_t *j, int model_nq) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_nq);
    int n = std::min(static_cast<int>(j->count), model_nq);
    for (int i = 0; i < n; i++)
            q[i] = j->q[i];
    return q;
}

/* ==========================================================================
* FK
* ========================================================================== */

static int pinocchio_forward(kin_solver_t *solver,
                                const kin_joints_t *joints,
                                kin_pose_t *out) {
    if (!solver || !joints || !out)
            return KIN_ERR_PARAM;

    auto *priv = static_cast<pinocchio_priv *>(solver->priv_data);

    if (static_cast<int>(joints->count) > priv->num_joints) {
            pin_err("FK: too many joints (model %d, got %d)",
                            priv->num_joints, static_cast<int>(joints->count));
            return KIN_ERR_PARAM;
    }

    Eigen::VectorXd q = joints_to_eigen(joints, priv->num_joints);

    try {
            pinocchio::forwardKinematics(priv->model, priv->data, q);
            pinocchio::updateFramePlacements(priv->model, priv->data);
            se3_to_pose(priv->data.oMf[priv->tip_frame_id], out);
            return KIN_OK;
    } catch (const std::exception &e) {
            pin_err("FK exception: %s", e.what());
            return KIN_ERR_FK_FAIL;
    }
}

/* ==========================================================================
* IK  (CLIK — Closed-Loop Inverse Kinematics)
* ========================================================================== */

/* Tuning constants — adjust as needed for different robots */
static constexpr double IK_DAMPING       = 1e-4; /* λ for damped LS       */
static constexpr double IK_STEP_INIT     = 0.5;  /* initial step-size α   */
static constexpr double IK_STEP_MIN      = 0.01; /* stop shrinking below  */
static constexpr double IK_DQ_MAX        = 0.5;  /* max ‖Δq‖ per iter    */
static constexpr double IK_DIVERGE_RATIO = 1.5;  /* error growth trigger  */
static constexpr double IK_STALL_TOL     = 1e-8; /* |Δerror| stall thr.   */
static constexpr int    IK_STALL_LIMIT   = 10;   /* consecutive stalls    */

static int pinocchio_inverse(kin_solver_t *solver,
                                const kin_pose_t *target,
                                const kin_joints_t *q_init,
                                const kin_ik_params_t *params,
                                kin_joints_t *out) {
    if (!solver || !target || !out)
            return KIN_ERR_PARAM;

    auto *priv = static_cast<pinocchio_priv *>(solver->priv_data);

    /* ---------- parameter setup ---------- */
    const double epsilon =
            (params && params->epsilon > 0)
                    ? params->epsilon
                    : priv->ik_epsilon;
    const int    max_iter = priv->ik_max_iter;
    /* position_weight: 1.0=仅位置, <1.0=加权混合 */
    const double pos_w = (params && params->position_weight > 0.0)
                            ? params->position_weight : 1.0;
    const bool   pos_only = (pos_w >= 1.0);
    const double ori_w = 1.0 - pos_w;

    const int out_count =
            (q_init && q_init->count > 0)
                    ? std::min(
                            static_cast<int>(q_init->count),
                            priv->num_joints)
                    : priv->num_joints;

    /* ---------- initial configuration ---------- */
    Eigen::VectorXd q = (q_init && q_init->count > 0)
            ? joints_to_eigen(q_init, priv->num_joints)
            : pinocchio::neutral(priv->model);

    const pinocchio::SE3 tgt = pose_to_se3(target);

    /* ---------- CLIK loop ---------- */
    try {
        double prev_err  = 1e10;
        double step      = IK_STEP_INIT;
        int    stall_cnt = 0;

        for (int it = 0; it < max_iter; it++) {
            pinocchio::forwardKinematics(priv->model, priv->data, q);
            pinocchio::updateFramePlacements(priv->model, priv->data);

            const pinocchio::SE3 &cur = priv->data.oMf[priv->tip_frame_id];
            auto err   = compute_pose_error(cur, tgt, pos_only);

            /* 应用权重到误差 */
            if (!pos_only) {
                err.head<3>() *= pos_w;
                err.tail<3>() *= ori_w;
            }

            double en  = pos_only ? err.head<3>().norm() : err.norm();

            /* convergence */
            if (en < epsilon) {
                    out->count = static_cast<uint8_t>(out_count);
                    for (int i = 0; i < out_count; i++)
                            out->q[i] = q[i];
                    return KIN_OK;
            }

            /* stall detection */
            if (std::fabs(en - prev_err) < IK_STALL_TOL) {
                    if (++stall_cnt > IK_STALL_LIMIT)
                            break;
            } else {
                    stall_cnt = 0;
            }

            /* divergence → shrink step */
            if (it > 10 && en > prev_err * IK_DIVERGE_RATIO) {
                    step *= 0.5;
                    if (step < IK_STEP_MIN)
                            break;
            }
            prev_err = en;

            /* Jacobian */
            pinocchio::Data::Matrix6x J(6, priv->model.nv);
            J.setZero();
            pinocchio::computeFrameJacobian(
                    priv->model, priv->data, q,
                    priv->tip_frame_id,
                    pinocchio::LOCAL_WORLD_ALIGNED, J);

            /* damped least-squares: Δq = Jᵀ (J Jᵀ + λI)⁻¹ e */
            Eigen::VectorXd dq;
            if (pos_only) {
                    Eigen::MatrixXd Jp = J.topRows<3>();
                    Eigen::Matrix3d JJt = Jp * Jp.transpose();
                    JJt.diagonal().array() += IK_DAMPING;
                    dq = Jp.transpose() * JJt.ldlt().solve(err.head<3>());
            } else {
                    /* 应用权重到 Jacobian */
                    Eigen::MatrixXd J_weighted = J;
                    J_weighted.topRows<3>() *= pos_w;
                    J_weighted.bottomRows<3>() *= ori_w;

                    Eigen::MatrixXd JJt = J_weighted * J_weighted.transpose();
                    JJt.diagonal().array() += IK_DAMPING;
                    dq = J_weighted.transpose() * JJt.ldlt().solve(err);
            }

            /* clamp Δq */
            double dqn = dq.norm();
            if (dqn > IK_DQ_MAX)
                    dq *= (IK_DQ_MAX / dqn);

            /* integrate & clamp to joint limits */
            q = pinocchio::integrate(priv->model, q, step * dq);
            q = q.cwiseMax(priv->model.lowerPositionLimit)
                        .cwiseMin(priv->model.upperPositionLimit);
        }

        pin_err("IK did not converge (iter=%d eps=%.1e pos_only=%d)",
                        max_iter, epsilon, pos_only);
        return KIN_ERR_IK_FAIL;
    } catch (const std::exception &e) {
        pin_err("IK exception: %s", e.what());
        return KIN_ERR_IK_FAIL;
    }
}

/* ==========================================================================
    * Query helpers
    * ========================================================================== */

static int pinocchio_get_num_joints(kin_solver_t *solver) {
        if (!solver) return 0;
        return static_cast<pinocchio_priv *>(solver->priv_data)->num_joints;
}

static int pinocchio_get_joint_limits(
        kin_solver_t *solver,
        double *lower,
        double *upper) {
        if (!solver || !lower || !upper)
                return KIN_ERR_PARAM;

        auto *priv = static_cast<pinocchio_priv *>(solver->priv_data);
        for (int i = 0; i < priv->num_joints; i++) {
                lower[i] = priv->model.lowerPositionLimit[i];
                upper[i] = priv->model.upperPositionLimit[i];
        }
        return KIN_OK;
}

static void pinocchio_destroy(kin_solver_t *solver) {
        if (!solver) return;
        delete static_cast<pinocchio_priv *>(solver->priv_data);
        free(const_cast<char *>(solver->name));
        free(solver);
}

/* ==========================================================================
* Ops table & factory
* ========================================================================== */

static const struct kin_ops pinocchio_ops = {
    .forward          = pinocchio_forward,
    .inverse          = pinocchio_inverse,
    .get_num_joints   = pinocchio_get_num_joints,
    .get_joint_limits = pinocchio_get_joint_limits,
    .destroy          = pinocchio_destroy,
};

extern "C" {

static kin_solver_t *pinocchio_factory(const char *urdf_path,
                                        const char *base_link,
                                        const char *tip_link) {
    if (!urdf_path || !tip_link) {
        pin_err("factory: urdf_path and tip_link are required");
        return nullptr;
    }

    pinocchio_priv *priv = nullptr;
    try {
        priv = new pinocchio_priv();
    } catch (...) {
        pin_err("factory: allocation failed");
        return nullptr;
    }

    priv->urdf_path   = urdf_path;
    priv->base_link   = base_link ? base_link : "";
    priv->tip_link    = tip_link;
    priv->ik_epsilon  = 5e-4;   /* 0.5 mm */
    priv->ik_max_iter = 200;

    try {
        pinocchio::urdf::buildModel(urdf_path, priv->model);
        priv->data = pinocchio::Data(priv->model);

        if (!priv->model.existFrame(tip_link)) {
            pin_err("tip frame '%s' not found in URDF", tip_link);
            delete priv;
            return nullptr;
        }
        priv->tip_frame_id = priv->model.getFrameId(tip_link);
        priv->num_joints   = priv->model.nq;

        pin_info("URDF loaded: %s  DOF=%d  tip=%s (frame %lu)",
                            priv->model.name.c_str(), priv->model.nq,
                            tip_link,
                            static_cast<size_t>(priv->tip_frame_id));
    } catch (const std::exception &e) {
        pin_err("URDF load failed: %s", e.what());
        delete priv;
        return nullptr;
    }

    auto *s = static_cast<kin_solver_t *>(calloc(1, sizeof(kin_solver_t)));
    if (!s) {
        delete priv;
        return nullptr;
    }

    s->name      = strdup("pinocchio");
    s->ops       = &pinocchio_ops;
    s->priv_data = priv;
    return s;
}
REGISTER_KIN_SOLVER("pinocchio", pinocchio_factory)

} /* extern "C" */
