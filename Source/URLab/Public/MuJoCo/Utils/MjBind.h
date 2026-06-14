// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with, 
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are 
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0), 
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once
 
#include <mujoco/mujoco.h>
#include <string>
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"

// ==========================================
// 1. The View Structs
// ==========================================

// Helper to format 3D vectors
inline FString FormatVec3(const mjtNum* v) {
    if (!v) return TEXT("NULL");
    return FString::Printf(TEXT("[%.3f, %.3f, %.3f]"), v[0], v[1], v[2]);
}

// Helper to format quaternions
inline FString FormatQuat(const mjtNum* q) {
    if (!q) return TEXT("NULL");
    return FString::Printf(TEXT("[%.3f, %.3f, %.3f, %.3f]"), q[0], q[1], q[2], q[3]);
}

/**
 * @struct GeomView
 * @brief Lightweight wrapper around MuJoCo geom data (model and data).
 * provides easy access to configuration and state.
 */
struct GeomView {
    static constexpr mjtObj obj_type = mjOBJ_GEOM;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_GeomView_FIELDS_START ---
    int geom_type;
    int geom_contype;
    int geom_conaffinity;
    int geom_condim;
    int geom_bodyid;
    int geom_dataid;
    int geom_matid;
    int geom_group;
    int geom_priority;
    int geom_plugin;
    mjtByte* geom_sameframe;
    mjtNum* geom_solmix;
    mjtNum* geom_solref;
    mjtNum* geom_solimp;
    mjtNum* geom_size;
    mjtNum* geom_aabb;
    mjtNum* geom_rbound;
    mjtNum* geom_pos;
    mjtNum* geom_quat;
    mjtNum* geom_friction;
    mjtNum* geom_margin;
    mjtNum* geom_gap;
    mjtNum* geom_fluid;
    mjtNum* geom_user;
    float* geom_rgba;
    mjtNum* geom_xpos;
    mjtNum* geom_xmat;
    // --- CODEGEN_VIEW_GeomView_FIELDS_END ---

    GeomView() : _m(nullptr), _d(nullptr), id(-1) {}

    GeomView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->ngeom);
        name = (m->name_geomadr[id] >= 0) ? m->names + m->name_geomadr[id] : nullptr;

    // --- CODEGEN_VIEW_GeomView_BIND_START ---
    geom_type = m->geom_type[id];
    geom_contype = m->geom_contype[id];
    geom_conaffinity = m->geom_conaffinity[id];
    geom_condim = m->geom_condim[id];
    geom_bodyid = m->geom_bodyid[id];
    geom_dataid = m->geom_dataid[id];
    geom_matid = m->geom_matid[id];
    geom_group = m->geom_group[id];
    geom_priority = m->geom_priority[id];
    geom_plugin = m->geom_plugin[id];
    geom_sameframe = m->geom_sameframe + id * 1;
    geom_solmix = m->geom_solmix + id * 1;
    geom_solref = m->geom_solref + id * mjNREF;
    geom_solimp = m->geom_solimp + id * mjNIMP;
    geom_size = m->geom_size + id * 3;
    geom_aabb = m->geom_aabb + id * 6;
    geom_rbound = m->geom_rbound + id * 1;
    geom_pos = m->geom_pos + id * 3;
    geom_quat = m->geom_quat + id * 4;
    geom_friction = m->geom_friction + id * 3;
    geom_margin = m->geom_margin + id * 1;
    geom_gap = m->geom_gap + id * 1;
    geom_fluid = m->geom_fluid + id * mjNFLUID;
    geom_user = m->geom_user + id * m->nuser_geom;
    geom_rgba = m->geom_rgba + id * 4;
    geom_xpos = d->geom_xpos + id * 3;
    geom_xmat = d->geom_xmat + id * 9;
    // --- CODEGEN_VIEW_GeomView_BIND_END ---
    }

    /** @brief Sets the friction coefficient (tangential) for this geom. */
    void SetFriction(float Value) {
        if (geom_friction) geom_friction[0] = (mjtNum)Value;
    }

    /** @brief Sets the contact solver reference parameters (time constant, damping ratio). */
    void SetSolRef(float TimeConst, float DampRatio) {
        if (geom_solref) {
            geom_solref[0] = (mjtNum)TimeConst;
            geom_solref[1] = (mjtNum)DampRatio;
        }
    }

    /** @brief Sets the contact solver impedance parameters. */
    void SetSolImp(float Dmin, float Dmax, float Width) {
        if (geom_solimp) {
            geom_solimp[0] = (mjtNum)Dmin;
            geom_solimp[1] = (mjtNum)Dmax;
            geom_solimp[2] = (mjtNum)Width;
        }
    }

    FString ToString() const {
        FString Info = FString::Printf(TEXT("=== Geom ID: %d (%s) ===\n"), id, name ? *MjUtils::MjToString(name) : TEXT("None"));
        Info += FString::Printf(TEXT("    Type: %d | Size: %s\n"), geom_type, *FormatVec3(geom_size));
        Info += FString::Printf(TEXT("    World Pos: %s\n"), *FormatVec3(geom_xpos));
        return Info;
    }
};

/**
 * @struct JointView
 * @brief Lightweight wrapper around MuJoCo joint data.
 */
struct JointView {
    static constexpr mjtObj obj_type = mjOBJ_JOINT;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_JointView_FIELDS_START ---
    int jnt_type;
    int jnt_qposadr;
    int jnt_dofadr;
    int jnt_bodyid;
    int jnt_actuatorid;
    int jnt_group;
    mjtByte* jnt_limited;
    mjtByte* jnt_actfrclimited;
    mjtByte* jnt_actgravcomp;
    mjtNum* jnt_solref;
    mjtNum* jnt_solimp;
    mjtNum* jnt_pos;
    mjtNum* jnt_axis;
    mjtNum* jnt_stiffness;
    mjtNum* jnt_stiffnesspoly;
    mjtNum* jnt_range;
    mjtNum* jnt_actfrcrange;
    mjtNum* jnt_margin;
    mjtNum* jnt_user;
    int dof_bodyid;
    int dof_jntid;
    int dof_parentid;
    int dof_treeid;
    int dof_Madr;
    int dof_simplenum;
    mjtNum* dof_solref;
    mjtNum* dof_solimp;
    mjtNum* dof_frictionloss;
    mjtNum* dof_armature;
    mjtNum* dof_damping;
    mjtNum* dof_dampingpoly;
    mjtNum* dof_invweight0;
    mjtNum* dof_M0;
    mjtNum* dof_length;
    mjtNum* qpos;
    mjtNum* qvel;
    mjtNum* qacc;
    mjtNum* xanchor;
    mjtNum* xaxis;
    // --- CODEGEN_VIEW_JointView_FIELDS_END ---

    JointView() : _m(nullptr), _d(nullptr), id(-1) {}

    JointView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->njnt);
        name = (m->name_jntadr[id] >= 0) ? m->names + m->name_jntadr[id] : nullptr;

    // --- CODEGEN_VIEW_JointView_BIND_START ---
    const int qpos_adr = m->jnt_qposadr[id];
    const int dof_adr = m->jnt_dofadr[id];
    jnt_type = m->jnt_type[id];
    jnt_qposadr = m->jnt_qposadr[id];
    jnt_dofadr = m->jnt_dofadr[id];
    jnt_bodyid = m->jnt_bodyid[id];
    jnt_actuatorid = m->jnt_actuatorid[id];
    jnt_group = m->jnt_group[id];
    jnt_limited = m->jnt_limited + id * 1;
    jnt_actfrclimited = m->jnt_actfrclimited + id * 1;
    jnt_actgravcomp = m->jnt_actgravcomp + id * 1;
    jnt_solref = m->jnt_solref + id * mjNREF;
    jnt_solimp = m->jnt_solimp + id * mjNIMP;
    jnt_pos = m->jnt_pos + id * 3;
    jnt_axis = m->jnt_axis + id * 3;
    jnt_stiffness = m->jnt_stiffness + id * 1;
    jnt_stiffnesspoly = m->jnt_stiffnesspoly + id * mjNPOLY;
    jnt_range = m->jnt_range + id * 2;
    jnt_actfrcrange = m->jnt_actfrcrange + id * 2;
    jnt_margin = m->jnt_margin + id * 1;
    jnt_user = m->jnt_user + id * m->nuser_jnt;
    dof_bodyid = m->dof_bodyid[dof_adr];
    dof_jntid = m->dof_jntid[dof_adr];
    dof_parentid = m->dof_parentid[dof_adr];
    dof_treeid = m->dof_treeid[dof_adr];
    dof_Madr = m->dof_Madr[dof_adr];
    dof_simplenum = m->dof_simplenum[dof_adr];
    dof_solref = m->dof_solref + dof_adr * mjNREF;
    dof_solimp = m->dof_solimp + dof_adr * mjNIMP;
    dof_frictionloss = m->dof_frictionloss + dof_adr * 1;
    dof_armature = m->dof_armature + dof_adr * 1;
    dof_damping = m->dof_damping + dof_adr * 1;
    dof_dampingpoly = m->dof_dampingpoly + dof_adr * mjNPOLY;
    dof_invweight0 = m->dof_invweight0 + dof_adr * 1;
    dof_M0 = m->dof_M0 + dof_adr * 1;
    dof_length = m->dof_length + dof_adr * 1;
    qpos = d->qpos + qpos_adr * 1;
    qvel = d->qvel + dof_adr * 1;
    qacc = d->qacc + dof_adr * 1;
    xanchor = d->xanchor + id * 3;
    xaxis = d->xaxis + id * 3;
    // --- CODEGEN_VIEW_JointView_BIND_END ---
    }

    /** @brief Gets the current joint position (radians for hinges, meters for sliders). */
    float GetPosition() const {
        return (qpos) ? (float)qpos[0] : 0.0f;
    }

    /** @brief Directly sets the joint position. Warning: teleports the physics state. */
    void SetPosition(float Value) {
        if (qpos) qpos[0] = (mjtNum)Value;
    }

    FString ToString() const {
        return FString::Printf(TEXT("=== Joint ID: %d (%s) Type: %d ===\n"), id, name ? *MjUtils::MjToString(name) : TEXT("None"), jnt_type);
    }
};

/**
 * @struct ActuatorView
 * @brief Lightweight wrapper around MuJoCo actuator data.
 */
struct ActuatorView {
    static constexpr mjtObj obj_type = mjOBJ_ACTUATOR;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_ActuatorView_FIELDS_START ---
    int actuator_trntype;
    int actuator_dyntype;
    int actuator_gaintype;
    int actuator_biastype;
    int* actuator_trnid;
    mjtNum* actuator_damping;
    mjtNum* actuator_dampingpoly;
    mjtNum* actuator_armature;
    int actuator_actadr;
    int actuator_actnum;
    int actuator_group;
    int* actuator_history;
    int actuator_historyadr;
    mjtNum* actuator_delay;
    mjtByte* actuator_ctrllimited;
    mjtByte* actuator_forcelimited;
    mjtByte* actuator_actlimited;
    mjtNum* actuator_dynprm;
    mjtNum* actuator_gainprm;
    mjtNum* actuator_biasprm;
    mjtByte* actuator_actearly;
    mjtNum* actuator_ctrlrange;
    mjtNum* actuator_forcerange;
    mjtNum* actuator_actrange;
    mjtNum* actuator_gear;
    mjtNum* actuator_cranklength;
    mjtNum* actuator_acc0;
    mjtNum* actuator_length0;
    mjtNum* actuator_lengthrange;
    mjtNum* actuator_user;
    int actuator_plugin;
    mjtNum* ctrl;
    mjtNum* actuator_force;
    mjtNum* actuator_length;
    mjtNum* actuator_velocity;
    mjtNum* actuator_moment;
    mjtNum* act;
    // --- CODEGEN_VIEW_ActuatorView_FIELDS_END ---

    ActuatorView() : _m(nullptr), _d(nullptr), id(-1) {}

    ActuatorView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->nu);
        name = (m->name_actuatoradr[id] >= 0) ? m->names + m->name_actuatoradr[id] : nullptr;

    // --- CODEGEN_VIEW_ActuatorView_BIND_START ---
    actuator_trntype = m->actuator_trntype[id];
    actuator_dyntype = m->actuator_dyntype[id];
    actuator_gaintype = m->actuator_gaintype[id];
    actuator_biastype = m->actuator_biastype[id];
    actuator_trnid = m->actuator_trnid + id * 2;
    actuator_damping = m->actuator_damping + id * 1;
    actuator_dampingpoly = m->actuator_dampingpoly + id * mjNPOLY;
    actuator_armature = m->actuator_armature + id * 1;
    actuator_actadr = m->actuator_actadr[id];
    actuator_actnum = m->actuator_actnum[id];
    actuator_group = m->actuator_group[id];
    actuator_history = m->actuator_history + id * 2;
    actuator_historyadr = m->actuator_historyadr[id];
    actuator_delay = m->actuator_delay + id * 1;
    actuator_ctrllimited = m->actuator_ctrllimited + id * 1;
    actuator_forcelimited = m->actuator_forcelimited + id * 1;
    actuator_actlimited = m->actuator_actlimited + id * 1;
    actuator_dynprm = m->actuator_dynprm + id * mjNDYN;
    actuator_gainprm = m->actuator_gainprm + id * mjNGAIN;
    actuator_biasprm = m->actuator_biasprm + id * mjNBIAS;
    actuator_actearly = m->actuator_actearly + id * 1;
    actuator_ctrlrange = m->actuator_ctrlrange + id * 2;
    actuator_forcerange = m->actuator_forcerange + id * 2;
    actuator_actrange = m->actuator_actrange + id * 2;
    actuator_gear = m->actuator_gear + id * 6;
    actuator_cranklength = m->actuator_cranklength + id * 1;
    actuator_acc0 = m->actuator_acc0 + id * 1;
    actuator_length0 = m->actuator_length0 + id * 1;
    actuator_lengthrange = m->actuator_lengthrange + id * 2;
    actuator_user = m->actuator_user + id * m->nuser_actuator;
    actuator_plugin = m->actuator_plugin[id];
    ctrl = d->ctrl + id * 1;
    actuator_force = d->actuator_force + id * 1;
    actuator_length = d->actuator_length + id * 1;
    actuator_velocity = d->actuator_velocity + id * 1;
    actuator_moment = d->actuator_moment + id * m->nv;
    act = (m->actuator_actadr[id] >= 0) ? d->act + m->actuator_actadr[id] : nullptr;
    // --- CODEGEN_VIEW_ActuatorView_BIND_END ---
    }
};

/**
 * @struct TendonView
 * @brief Lightweight wrapper around MuJoCo tendon data.
 */
struct TendonView {
    static constexpr mjtObj obj_type = mjOBJ_TENDON;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_TendonView_FIELDS_START ---
    int tendon_adr;
    int tendon_num;
    int tendon_matid;
    int tendon_actuatorid;
    int tendon_group;
    int tendon_treenum;
    int* tendon_treeid;
    int ten_J_rownnz;
    int ten_J_rowadr;
    int ten_J_colind;
    mjtByte* tendon_limited;
    mjtByte* tendon_actfrclimited;
    mjtNum* tendon_width;
    mjtNum* tendon_solref_lim;
    mjtNum* tendon_solimp_lim;
    mjtNum* tendon_solref_fri;
    mjtNum* tendon_solimp_fri;
    mjtNum* tendon_range;
    mjtNum* tendon_actfrcrange;
    mjtNum* tendon_margin;
    mjtNum* tendon_stiffness;
    mjtNum* tendon_stiffnesspoly;
    mjtNum* tendon_damping;
    mjtNum* tendon_dampingpoly;
    mjtNum* tendon_armature;
    mjtNum* tendon_frictionloss;
    mjtNum* tendon_lengthspring;
    mjtNum* tendon_length0;
    mjtNum* tendon_invweight0;
    mjtNum* tendon_user;
    float* tendon_rgba;
    mjtNum* ten_length;
    mjtNum* ten_velocity;
    // --- CODEGEN_VIEW_TendonView_FIELDS_END ---

    TendonView() : _m(nullptr), _d(nullptr), id(-1), name(nullptr) {}

    TendonView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->ntendon);
        name = (m->name_tendonadr[id] >= 0) ? m->names + m->name_tendonadr[id] : nullptr;

    // --- CODEGEN_VIEW_TendonView_BIND_START ---
    tendon_adr = m->tendon_adr[id];
    tendon_num = m->tendon_num[id];
    tendon_matid = m->tendon_matid[id];
    tendon_actuatorid = m->tendon_actuatorid[id];
    tendon_group = m->tendon_group[id];
    tendon_treenum = m->tendon_treenum[id];
    tendon_treeid = m->tendon_treeid + id * 2;
    ten_J_rownnz = m->ten_J_rownnz[id];
    ten_J_rowadr = m->ten_J_rowadr[id];
    ten_J_colind = m->ten_J_colind[id];
    tendon_limited = m->tendon_limited + id * 1;
    tendon_actfrclimited = m->tendon_actfrclimited + id * 1;
    tendon_width = m->tendon_width + id * 1;
    tendon_solref_lim = m->tendon_solref_lim + id * mjNREF;
    tendon_solimp_lim = m->tendon_solimp_lim + id * mjNIMP;
    tendon_solref_fri = m->tendon_solref_fri + id * mjNREF;
    tendon_solimp_fri = m->tendon_solimp_fri + id * mjNIMP;
    tendon_range = m->tendon_range + id * 2;
    tendon_actfrcrange = m->tendon_actfrcrange + id * 2;
    tendon_margin = m->tendon_margin + id * 1;
    tendon_stiffness = m->tendon_stiffness + id * 1;
    tendon_stiffnesspoly = m->tendon_stiffnesspoly + id * mjNPOLY;
    tendon_damping = m->tendon_damping + id * 1;
    tendon_dampingpoly = m->tendon_dampingpoly + id * mjNPOLY;
    tendon_armature = m->tendon_armature + id * 1;
    tendon_frictionloss = m->tendon_frictionloss + id * 1;
    tendon_lengthspring = m->tendon_lengthspring + id * 2;
    tendon_length0 = m->tendon_length0 + id * 1;
    tendon_invweight0 = m->tendon_invweight0 + id * 1;
    tendon_user = m->tendon_user + id * m->nuser_tendon;
    tendon_rgba = m->tendon_rgba + id * 4;
    ten_length = d->ten_length + id * 1;
    ten_velocity = d->ten_velocity + id * 1;
    // --- CODEGEN_VIEW_TendonView_BIND_END ---
    }

    /** @brief Gets the current tendon length (meters). */
    float GetLength() const {
        return ten_length ? (float)*ten_length : 0.0f;
    }

    /** @brief Gets the current tendon velocity (m/s). */
    float GetVelocity() const {
        return ten_velocity ? (float)*ten_velocity : 0.0f;
    }
};

/**
 * @struct SensorView
 * @brief Lightweight wrapper around MuJoCo sensor data.
 */
struct SensorView {
    static constexpr mjtObj obj_type = mjOBJ_SENSOR;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_SensorView_FIELDS_START ---
    int sensor_type;
    int sensor_datatype;
    int sensor_needstage;
    int sensor_objtype;
    int sensor_objid;
    int sensor_reftype;
    int sensor_refid;
    int* sensor_intprm;
    int sensor_dim;
    int sensor_adr;
    mjtNum* sensor_cutoff;
    mjtNum* sensor_noise;
    int* sensor_history;
    int sensor_historyadr;
    mjtNum* sensor_delay;
    mjtNum* sensor_interval;
    mjtNum* sensor_user;
    int sensor_plugin;
    mjtNum* sensordata;
    // --- CODEGEN_VIEW_SensorView_FIELDS_END ---

    SensorView() : _m(nullptr), _d(nullptr), id(-1) {}

    SensorView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->nsensor);
        name = (m->name_sensoradr[id] >= 0) ? m->names + m->name_sensoradr[id] : nullptr;

    // --- CODEGEN_VIEW_SensorView_BIND_START ---
    sensor_type = m->sensor_type[id];
    sensor_datatype = m->sensor_datatype[id];
    sensor_needstage = m->sensor_needstage[id];
    sensor_objtype = m->sensor_objtype[id];
    sensor_objid = m->sensor_objid[id];
    sensor_reftype = m->sensor_reftype[id];
    sensor_refid = m->sensor_refid[id];
    sensor_intprm = m->sensor_intprm + id * mjNSENS;
    sensor_dim = m->sensor_dim[id];
    sensor_adr = m->sensor_adr[id];
    sensor_cutoff = m->sensor_cutoff + id * 1;
    sensor_noise = m->sensor_noise + id * 1;
    sensor_history = m->sensor_history + id * 2;
    sensor_historyadr = m->sensor_historyadr[id];
    sensor_delay = m->sensor_delay + id * 1;
    sensor_interval = m->sensor_interval + id * 2;
    sensor_user = m->sensor_user + id * m->nuser_sensor;
    sensor_plugin = m->sensor_plugin[id];
    sensordata = (sensor_adr >= 0 && sensor_adr < m->nsensordata) ? d->sensordata + sensor_adr : nullptr;
    // --- CODEGEN_VIEW_SensorView_BIND_END ---
    }
};
/**
 * @struct SiteView
 * @brief Lightweight wrapper around MuJoCo site data.
 */
struct SiteView {
    static constexpr mjtObj obj_type = mjOBJ_SITE;

    const mjModel* _m;
    mjData* _d;
    int id;
    const char* name;

    // --- CODEGEN_VIEW_SiteView_FIELDS_START ---
    int site_type;
    int site_bodyid;
    int site_matid;
    int site_group;
    mjtByte* site_sameframe;
    mjtNum* site_size;
    mjtNum* site_pos;
    mjtNum* site_quat;
    mjtNum* site_user;
    float* site_rgba;
    mjtNum* site_xpos;
    mjtNum* site_xmat;
    // --- CODEGEN_VIEW_SiteView_FIELDS_END ---

    SiteView() : _m(nullptr), _d(nullptr), id(-1), name(nullptr) {}

    SiteView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->nsite);
        name = (m->name_siteadr[id] >= 0) ? m->names + m->name_siteadr[id] : nullptr;

    // --- CODEGEN_VIEW_SiteView_BIND_START ---
    site_type = m->site_type[id];
    site_bodyid = m->site_bodyid[id];
    site_matid = m->site_matid[id];
    site_group = m->site_group[id];
    site_sameframe = m->site_sameframe + id * 1;
    site_size = m->site_size + id * 3;
    site_pos = m->site_pos + id * 3;
    site_quat = m->site_quat + id * 4;
    site_user = m->site_user + id * m->nuser_site;
    site_rgba = m->site_rgba + id * 4;
    site_xpos = d->site_xpos + id * 3;
    site_xmat = d->site_xmat + id * 9;
    // --- CODEGEN_VIEW_SiteView_BIND_END ---
    }

    FVector GetWorldPosition() const {
        return MjUtils::MjToUEPosition(site_xpos);
    }

    FString ToString() const {
        FString Info = FString::Printf(TEXT("=== Site ID: %d (%s) ===\n"), id, name ? *MjUtils::MjToString(name) : TEXT("None"));
        Info += FString::Printf(TEXT("    Type: %d | Size: %s\n"), site_type, *FormatVec3(site_size));
        Info += FString::Printf(TEXT("    World Pos: %s\n"), *FormatVec3(site_xpos));
        return Info;
    }
};

/**
 * @struct BodyView
 * @brief Lightweight wrapper around MuJoCo body data.
 * Can spawn children views for traversal.
 */
struct BodyView {
    static constexpr mjtObj obj_type = mjOBJ_BODY;

    // We must store these to spawn children views later
    const mjModel* _m;
    mjData* _d;
    int id;

    // --- Identification ---
    const char* name;

    // --- CODEGEN_VIEW_BodyView_FIELDS_START ---
    int body_parentid;
    int body_rootid;
    int body_weldid;
    int body_mocapid;
    int body_jntnum;
    int body_jntadr;
    int body_dofnum;
    int body_dofadr;
    int body_treeid;
    int body_geomnum;
    int body_geomadr;
    mjtByte* body_simple;
    mjtByte* body_sameframe;
    mjtNum* body_pos;
    mjtNum* body_quat;
    mjtNum* body_ipos;
    mjtNum* body_iquat;
    mjtNum* body_mass;
    mjtNum* body_subtreemass;
    mjtNum* body_inertia;
    mjtNum* body_invweight0;
    mjtNum* body_gravcomp;
    mjtNum* body_margin;
    mjtNum* body_user;
    int body_plugin;
    int body_contype;
    int body_conaffinity;
    int body_bvhadr;
    int body_bvhnum;
    int tree_bodyadr;
    int tree_bodynum;
    int tree_dofadr;
    int tree_dofnum;
    int tree_sleep_policy;
    mjtNum* xpos;
    mjtNum* xquat;
    mjtNum* xmat;
    mjtNum* cvel;
    mjtNum* cinert;
    mjtNum* xfrc_applied;
    // --- CODEGEN_VIEW_BodyView_FIELDS_END ---

    // Initialize critical members to safe defaults so a failed bind doesn't cause a crash later
    BodyView() : _m(nullptr), _d(nullptr), id(-1), name(nullptr) {}

    BodyView(const mjModel* m, mjData* d, int id_in) : _m(m), _d(d), id(id_in) {
        check(id >= 0 && id < m->nbody);
        // Name resolution
        name = (m->name_bodyadr[id] >= 0) ? m->names + m->name_bodyadr[id] : nullptr;

    // --- CODEGEN_VIEW_BodyView_BIND_START ---
    body_parentid = m->body_parentid[id];
    body_rootid = m->body_rootid[id];
    body_weldid = m->body_weldid[id];
    body_mocapid = m->body_mocapid[id];
    body_jntnum = m->body_jntnum[id];
    body_jntadr = m->body_jntadr[id];
    body_dofnum = m->body_dofnum[id];
    body_dofadr = m->body_dofadr[id];
    body_treeid = m->body_treeid[id];
    body_geomnum = m->body_geomnum[id];
    body_geomadr = m->body_geomadr[id];
    body_simple = m->body_simple + id * 1;
    body_sameframe = m->body_sameframe + id * 1;
    body_pos = m->body_pos + id * 3;
    body_quat = m->body_quat + id * 4;
    body_ipos = m->body_ipos + id * 3;
    body_iquat = m->body_iquat + id * 4;
    body_mass = m->body_mass + id * 1;
    body_subtreemass = m->body_subtreemass + id * 1;
    body_inertia = m->body_inertia + id * 3;
    body_invweight0 = m->body_invweight0 + id * 2;
    body_gravcomp = m->body_gravcomp + id * 1;
    body_margin = m->body_margin + id * 1;
    body_user = m->body_user + id * m->nuser_body;
    body_plugin = m->body_plugin[id];
    body_contype = m->body_contype[id];
    body_conaffinity = m->body_conaffinity[id];
    body_bvhadr = m->body_bvhadr[id];
    body_bvhnum = m->body_bvhnum[id];
    tree_bodyadr = m->tree_bodyadr[id];
    tree_bodynum = m->tree_bodynum[id];
    tree_dofadr = m->tree_dofadr[id];
    tree_dofnum = m->tree_dofnum[id];
    tree_sleep_policy = m->tree_sleep_policy[id];
    xpos = d->xpos + id * 3;
    xquat = d->xquat + id * 4;
    xmat = d->xmat + id * 9;
    cvel = d->cvel + id * 6;
    cinert = d->cinert + id * 10;
    xfrc_applied = d->xfrc_applied + id * 6;
    // --- CODEGEN_VIEW_BodyView_BIND_END ---
    }

    FVector GetWorldPosition() const {
        return MjUtils::MjToUEPosition(xpos);
    }

    FQuat GetWorldRotation() const {
        return MjUtils::MjToUERotation(xquat);
    }

    // Traversal Methods (Declared here, Implemented at the bottom)
    TArray<BodyView> Bodies() const;
    TArray<GeomView> Geoms() const;
    TArray<JointView> Joints() const;

    FString ToString() const {
        FString Info = FString::Printf(TEXT("=== Body ID: %d (%s) ===\n"), id, name ? *MjUtils::MjToString(name) : TEXT("None"));

        Info += TEXT("  [Config]\n");
        Info += FString::Printf(TEXT("    Mass: %.4f\n"), *body_mass);
        Info += FString::Printf(TEXT("    Rel Pos: %s\n"), *FormatVec3(body_pos));

        Info += TEXT("  [State]\n");
        Info += FString::Printf(TEXT("    World Pos: %s\n"), *FormatVec3(xpos));
        return Info;
    }
};

// --- Body View ---
// Returns a list of geoms attached to this body
inline TArray<GeomView> BodyView::Geoms() const {
    TArray<GeomView> geoms;
    int num = _m->body_geomnum[id];
    int start_adr = _m->body_geomadr[id];
    
    // Check for -1 (no geoms) just in case, though num would be 0
    if (start_adr >= 0) {
        for (int i = 0; i < num; ++i) {
            geoms.Emplace(_m, _d, start_adr + i);
        }
    }
    return geoms;
}
inline TArray<BodyView> BodyView::Bodies() const {
    TArray<BodyView> children;
    // Iterate over all bodies to find those whose parent is the current id
    for (int i = 0; i < _m->nbody; ++i) {
        if (_m->body_parentid[i] == id) {
            children.Emplace(_m, _d, i);
        }
    }
    return children;
}
// Returns a list of joints attached to this body
inline TArray<JointView> BodyView::Joints() const {
    TArray<JointView> joints;
    int num = _m->body_jntnum[id];
    int start_adr = _m->body_jntadr[id];

    if (start_adr >= 0) {
        for (int i = 0; i < num; ++i) {
            joints.Emplace(_m, _d, start_adr + i);
        }
    }
    return joints;
}
// ==========================================
// 2. The Templated Binder Function
// ==========================================

// Recursive function to traverse the hierarchy
inline void LogBodyHierarchy(const BodyView& RootBody, int IndentLevel = 0) {
    // 1. Create an indent string for visual hierarchy
    FString IndentString;
    for (int i = 0; i < IndentLevel; ++i) IndentString += TEXT("  |  ");

    // 2. Log the current Body
    FString BodyLog = FString::Printf(TEXT("%s[Body ID: %d] %s"), 
        *IndentString, 
        RootBody.id, 
        RootBody.name ? *MjUtils::MjToString(RootBody.name) : TEXT("Unnamed"));
    
    UE_LOG(LogURLabBind, Log, TEXT("%s"), *BodyLog);

    // 3. Log all Geoms attached to this Body
    TArray<GeomView> Geoms = RootBody.Geoms();
    for (const GeomView& Geom : Geoms) {
        FString GeomLog = FString::Printf(TEXT("%s  - (Geom ID: %d) Type: %d | Size: %s"), 
            *IndentString, 
            Geom.id, 
            Geom.geom_type,
            *FormatVec3(Geom.geom_size)); // Uses the FormatVec3 helper defined previously
        
        UE_LOG(LogURLabBind, Verbose, TEXT("%s"), *GeomLog);
    }

    // 4. Recurse into Child Bodies
    TArray<BodyView> Children = RootBody.Bodies();
    for (const BodyView& Child : Children) {
        LogBodyHierarchy(Child, IndentLevel + 1);
    }
}
template <typename T>
T bind(const mjModel* m, mjData* d, const std::string& name) {
    int id = mj_name2id(m, T::obj_type, name.c_str());
    if (id == -1) {
        UE_LOG(LogURLabBind, Warning, TEXT("MuJoCo Bind: Could not find '%hs'"), name.c_str());
        return T();
    }
    return T(m, d, id);
}