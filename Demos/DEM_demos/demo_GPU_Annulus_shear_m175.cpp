// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Ruochun Zhang
// =============================================================================
// Chrono::Gpu demo using SMC method. A body whose geometry is described by an
// OBJ file is time-integrated in Chrono and interacts with a granular wave tank
// in Chrono::Gpu via the co-simulation framework. The entire simulation consists
// of 2 runs: the settling phase (which outputs a checkpoint file), and a restarted
// phase (which load the checkpoint file and then drop the ball, literally).
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"


#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

void WriteCylinderVTK(const std::string& filename,
    double radius,
    double length,
    const ChFrame<>& frame,
    unsigned int res) {
    std::ofstream outf;
    outf.open(filename, std::ios::app);
    outf << "# vtk DataFile Version 1.0\nUnstructured Grid Example\nASCII\n\n" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID\nPOINTS " << 2 * res << " float\n";

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector<>(radius * cos(2 * i * 3.1415 / res), -1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    for (int i = 0; i < res; i++) {
        auto w = frame.TransformPointLocalToParent(
            ChVector<>(radius * cos(2 * i * 3.1415 / res), +1 * length / 2, radius * sin(2 * i * 3.1415 / res)));
        outf << w.x() << " " << w.y() << " " << w.z() << "\n";
    }

    outf << "\n\nCELLS " << res + res << "\t" << 5 * (res + res) << "\n";

    for (int i = 0; i < res - 1; i++) {
        outf << "4 " << i << " " << i + 1 << " " << i + res + 1 << " " << i + res << "\n";
    }
    outf << "4 " << res - 1 << " " << 0 << " " << res << " " << 2 * res - 1 << "\n";

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i << " " << i + 1 << " " << +res / 2 - i - 1 << " " << +res / 2 - i << "\n";
    }

    for (int i = 0; i < res / 4; i++) {
        outf << "4 " << i + res << " " << i + 1 + res << " " << +res / 2 - i - 1 + res << " " << +res / 2 - i + res
            << "\n";
    }

    outf << "4 " << +res / 2 << " " << 1 + res / 2 << " " << +res - 1 << " " << 0 << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + res / 2 << " " << i + 1 + res / 2 << " " << +res / 2 - i - 1 + res / 2 << " "
            << +res / 2 - i + res / 2 << "\n";
    }

    outf << "4 " << 3 * res / 2 << " " << 1 + 3 * res / 2 << " " << +2 * res - 1 << " " << +res << "\n";

    for (int i = 1; i < res / 4; i++) {
        outf << "4 " << i + 3 * res / 2 << " " << i + 1 + 3 * res / 2 << " " << +2 * res - i - 1 << " " << +2 * res - i
            << "\n";
    }

    outf << "\nCELL_TYPES " << res + res << "\n";

    for (int iele = 0; iele < (res + res); iele++) {
        outf << "9\n";
    }
}


// Output frequency
float out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 2000;

//*******************************************************************************************************************************************

void runPlateDrop(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {

    bool verbose = true;
    double angVel = 1.0 * CH_C_PI / 180.0;
    double linVel = 0;
    auto actuator = chrono_types::make_shared<ChLinkLinActuator>();
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    bool actuator_on = false;


    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.9f);
    mysurfmaterial->SetRestitution(0.4f);
    mysurfmaterial->SetAdhesion(0);

    
    ChVector<> IniVel(0.0, 0.0, 0.0);

    // Add a Plate mesh to the GPU system
    float plate_radius = 60.f;
    float plate_height = 15.f;
    float plate_mass = 175000;
    gpu_sys.AddMesh(GetChronoDataFile("models/Annulus.obj"), ChVector<float>(0), ChMatrix33<float>(plate_radius),
        plate_mass/2);

    // One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
    // let's overload that option.
    gpu_sys.EnableMeshCollision(true);

    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    // Create rigid plate_body simulation
    ChSystemSMC sys_plate;
    sys_plate.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_plate.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    sys_plate.Set_G_acc(ChVector<>(0, 0, -980));

    double inertia_z = 1.0 / 2.0 * plate_mass * plate_radius * plate_radius;
    double inertia_x = plate_mass * (plate_radius * plate_radius / 4 + plate_height * plate_height / 12);
    double inertia_y = plate_mass * (plate_radius * plate_radius / 4 + plate_height * plate_height / 12);
    //ChVector<> plate_initial_pos(0, 0, params.box_Z / 4.0 + 2 * params.sphere_radius);
    ChVector<> plate_initial_pos(0, 0, -57.5);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(params.box_X , params.box_Y , 0.001, 1000, false, true, mysurfmaterial);
    ground->SetPos(ChVector<>(0.0, 0.0, -100 ));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);
    sys_plate.AddBody(ground);

    std::shared_ptr<ChBody> plate_body(sys_plate.NewBody());
    plate_body->SetMass(plate_mass/2);
    plate_body->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    plate_body->SetPos(plate_initial_pos);

    
    ChQuaternion<> rotatio = Q_from_AngX(CH_C_PI / 2.0);
    //ChVector<> gyration = chrono::utils::CalcCylinderGyration(p_radius, p_length / 2).diagonal();
    //plate_body->SetInertiaXX(plate_mass * gyration);
    plate_body->SetPos_dt(IniVel);
    plate_body->SetRot(rotatio);

    // Set the collision type of the plate
    
    plate_body->SetCollide(true);
    plate_body->SetBodyFixed(false);
    //plate_body->GetCollisionModel()->ClearModel();
    //plate_body->GetCollisionModel()->SetSafeMargin(0.01);
    //chrono::utils::AddCylinderGeometry(plate_body.get(), mysurfmaterial, plate_radius, plate_height,ChVector<>(0.0), QUNIT);
    //plate_body->GetCollisionModel()->BuildModel();
    
    
    
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().rad = plate_radius;
    cyl->GetCylinderGeometry().p1 = plate_body->GetPos() + ChVector<>(0 ,0 , plate_height / 2);
    cyl->GetCylinderGeometry().p2 = plate_body->GetPos() - ChVector<>(0, 0 , plate_height / 2);
    plate_body->AddVisualShape(cyl);
    
    

    sys_plate.AddBody(plate_body);

    // Create the axle 
    auto axle = chrono_types::make_shared<ChBody>();
    // axle->SetIdentifier(Id_axle);
    axle->SetMass(plate_mass);
    axle->SetInertiaXX(ChVector<>(100000, 100000, 100000));
    axle->SetPos(plate_body->GetPos());
    axle->SetCollide(false);
    axle->SetBodyFixed(false);

    // Add geometry of the axle.
    axle->GetCollisionModel()->ClearModel();
    chrono::utils::AddSphereGeometry(axle.get(), mysurfmaterial, 0.5, ChVector<>(0.0, 0.0, -100));
    axle->GetCollisionModel()->BuildModel();
    sys_plate.AddBody(axle);

    // Connect the axle to the ground through a translational joint and create a linear actuator.
    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(ground, axle, ChCoordsys<>(axle->GetPos(), QUNIT));
    prismatic->SetName("prismatic_axle_ground");
    sys_plate.AddLink(prismatic);

    if (actuator_on) {
        auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0 , linVel);
        actuator->Initialize(ground, axle, false, ChCoordsys<>(axle->GetPos(), QUNIT),
            ChCoordsys<>(axle->GetPos() + ChVector<>(0, 0, 100), QUNIT));
        actuator->SetName("actuator");
        actuator->SetDistanceOffset(100);
        actuator->SetActuatorFunction(actuator_fun);
        sys_plate.AddLink(actuator);
    }

    // Connect the plate to the axle through a engine joint.
    motor->SetName("motor_plate_axle");
    motor->Initialize(plate_body, axle, ChFrame<>(plate_body->GetPos(), QUNIT));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_plate.AddLink(motor);
    
    //waiting for download
    ChVector<> force = actuator->Get_react_force();
    ChVector<> torque = motor->Get_react_torque();
    ChVector<> p_pos = plate_body->GetPos();


    ChGpuVisualization gpu_vis(&gpu_sys);
    if (render) {
        gpu_vis.SetTitle("Chrono::Gpu ball cosim demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -200, 100), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));

    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));
    
    // Create oputput directories  创建输出目录
    if (!filesystem::create_subdirectory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
    }
    
    if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
    }
     if (!filesystem::create_subdirectory(filesystem::path(out_dir + "/data"))) {
        std::cerr << "Error creating directory " << out_dir + "/data" << std::endl;
    }

    float iteration_step = params.step_size;
    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    std::cout << "Rendering at " << render_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    unsigned int curr_step = 0;
    
     // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP;
    std::ofstream myTorque;

    
    myFile.open(out_dir + "/data/results.txt", std::ios::trunc);
    myDBP.open(out_dir + "/data/DBP.txt", std::ios::trunc);
    myTorque.open(out_dir + "/data/Torque.txt", std::ios::trunc);

    
        
        

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        gpu_sys.ApplyMeshMotion(0,plate_body->GetPos(), plate_body->GetRot(), plate_body->GetPos_dt(),
            plate_body->GetWvel_par());

        ChVector<> ball_force;
        ChVector<> ball_torque;
        gpu_sys.CollectMeshContactForces(0, ball_force, ball_torque);

        plate_body->Empty_forces_accumulators();
        plate_body->Accumulate_force(ball_force, plate_body->GetPos(), false);
        plate_body->Accumulate_torque(ball_torque, false);
        
        //waiting for download
        force = actuator->Get_react_force();
        torque = motor->Get_react_torque();
        p_pos = plate_body->GetPos();

        double pos = (-57.5 - p_pos.z())/100;
        if (verbose) {
            std::cout << "Time: " << t << std::endl;
            std::cout << "  Plate position:         " << pos << std::endl;
            std::cout << "  Force:           " << force << std::endl;
            std::cout << "  Plate torque:           " << torque << std::endl;

        }
        
        
        myFile << t << "\t" << p_pos.x() << "\t" << p_pos.y() << "\t" << p_pos.z() << "\t" << torque.x() << "\t" << torque.y() << "\t" << torque.z() << "\t" << "\n";

        myDBP << t << "\t" << pos << "\t"<< force.x() << "\t" << force.y() << "\t" << force.z() << "\n";

        myTorque << t << "\t" << pos << "\t" << torque.x() << "\t" << torque.y() << "\t" << torque.z() << "\n";

    
    
    
    

        if (curr_step % out_steps == 0) {
            
            std::string file = out_dir + "/vtk/plate." + std::to_string(currframe) + ".vtk";
            WriteCylinderVTK(file, plate_radius, plate_height, sys_plate.Get_bodylist()[1]->GetFrame_REF_to_abs(), total_frames);
            
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            char mesh_filename[100];
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe);
            sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), currframe++); //*************mesh�ļ�ֻ����һ��****************
            gpu_sys.WriteParticleFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(mesh_filename));
        }

        
        //if (render && curr_step % render_steps == 0) { if (!gpu_vis.Render())  break; }
        

        gpu_sys.AdvanceSimulation(iteration_step);
        sys_plate.DoStepDynamics(iteration_step);
    }
    
    myFile.close();
    myDBP.close();
    myTorque.close();

	

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;
}


//*******************************************************************************************************************************************


int main(int argc, char* argv[]) {
    cudaSetDevice(0);
    std::string inputJson = GetChronoDataFile("gpu/demo_GPU_ballcosim.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_GPU_ballcosim <json_file>" << std::endl;
        return 1;
    }

    ChGpuSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    if (params.run_mode > 2) {
        std::cout << "ERROR: unknown run_mode specified" << std::endl;
        return 1;
    }

    // Output directory
    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    std::string checkpoint_file = out_dir + "/checkpoint.dat";

    if (params.run_mode == 2) {
        // run_mode = 2, this is a restarted run

        // Load checkpoint file.
        // Note that with current version, user defined meshes and boundaries are not stored in the checkpoint file,
        // so they must be manually set later. This behavior will be improved in later patches.
        // Simulation parameters and particle states are all in with this file loaded.
        ChSystemGpuMesh gpu_sys(checkpoint_file);

        // Add a ball through a mesh, whose dynamics are managed by Chrono Core, and run this co-simulation.
        runPlateDrop(gpu_sys, params);

        return 0;
    }
    
    

    // run_mode = 0, this is a newly started run. We have to set all simulation params.
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    printf(
        "Now run_mode == 0, this run is particle settling phase.\n"
        "After it is done, you will have a settled bed of granular material.\n"
        "A checkpoint file will be generated in the output directory to store this state.\n"
        "You can then open the JSON file, change \"run_mode\" from 0 to 1, then run this demo again,\n"
        "to proceed with the ball drop part of this demo.\n\n");

    float iteration_step = params.step_size;
    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - params.sphere_radius, params.box_Y / 2 - params.sphere_radius, 0);
    ChVector<> center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        // You can uncomment this line to see a report on particle creation process.
        // std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }
    std::cout << body_points.size() << " particles sampled!" << std::endl;

    gpu_sys.SetParticles(body_points);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);
	
	gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    gpu_sys.SetParticleOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.SetBDFixed(true);

    // In the settling run we disable the mesh.
    gpu_sys.EnableMeshCollision(false);

    /*
    // We could prescribe the motion of the big box domain. But here in this demo we will not do that.
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = CH_C_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    gpu_sys.setBDWallsMotionFunction(pos_func_wave);
    */

    gpu_sys.Initialize();

    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);
    int currframe = 0;
    unsigned int curr_step = 0;
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));
        }

        gpu_sys.AdvanceSimulation(iteration_step);
    }

    // This is settling phase, so output a checkpoint file
    gpu_sys.WriteCheckpointFile(checkpoint_file);

    return 0;
}
