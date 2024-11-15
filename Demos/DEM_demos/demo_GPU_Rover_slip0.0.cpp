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
// Authors: Yuemin Zhang, Wei Hu
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
#include "chrono/ChConfig.h"
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
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"
#include "chrono/core/ChTimer.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;
using namespace chrono::geometry;
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

void Rover(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {

    auto motor1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor3 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor4 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor5 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto motor6 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    auto actuator = chrono_types::make_shared<ChLinkLinActuator>();
    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.7f);
    mysurfmaterial->SetRestitution(0.4f);
    mysurfmaterial->SetAdhesion(0);

    ChVector<> IniVel(0.0, 0.0, 0.0);

    float wheel_mass = 17000;
    float chassis_mass = 17000;
    bool verbose = true;
    bool actuator_on = true;
    
    double wheel_radius = 25;
    double scale_ratio = wheel_radius  / 0.25;
    double wheel_slip = 0.0;
    double linVel = 50;
    double angVel = linVel / ((1 - wheel_slip) * wheel_radius);
    std::string wheel_obj = GetChronoDataFile("models/rover_wheel.obj");
    //Add six wheel mesh
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass /2);
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass / 2);
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass / 2);
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass / 2);
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass / 2);
    gpu_sys.AddMesh(GetChronoDataFile(wheel_obj), ChVector<float>(0), ChMatrix33<float>(scale_ratio),
        wheel_mass / 2);


    // One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
    // let's overload that option.
    gpu_sys.EnableMeshCollision(true);
    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    double inertia_x = 1e8;
    double inertia_z = 1e8;
    double inertia_y = 1e8;

    ChSystemSMC sys_rover;
    sys_rover.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_rover.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    sys_rover.Set_G_acc(ChVector<>(0, 0, -980));

    
    
    ChVector<> Rover_initial_pos(-500, 0, -12);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBodyEasyBox>(10000, 10000, 1.0, 1000, false, true, mysurfmaterial);
    ground->SetPos(ChVector<>(0.0, 0.0,0.0 ));
    ground->SetCollide(true);
    ground->SetBodyFixed(true);

    sys_rover.AddBody(ground);



    // Create the chassis
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(chassis_mass);
    chassis->SetPos(Rover_initial_pos);
    chassis->SetInertiaXX(ChVector<>(1e8, 1e8, 1e8));
    chassis->SetCollide(false);
    chassis->SetBodyFixed(false);
    chassis->SetPos_dt(IniVel);

    sys_rover.AddBody(chassis);

    //Create the constant velocity moving chassis
    auto moving_chassis = chrono_types::make_shared<ChBody>();
    moving_chassis->SetMass(chassis_mass);
    moving_chassis->SetPos(Rover_initial_pos);
    moving_chassis->SetInertiaXX(ChVector<>(1e8, 1e8, 1e8));
    moving_chassis->SetCollide(false);
    moving_chassis->SetBodyFixed(false);
    moving_chassis->SetPos_dt(IniVel);

    sys_rover.AddBody(moving_chassis);

    if (actuator_on) 
    {
        auto actuator_fun = chrono_types::make_shared<ChFunction_Ramp>(0.0, linVel);
        actuator->Initialize(ground, moving_chassis, false, ChCoordsys<>(chassis->GetPos(), QUNIT),
                ChCoordsys<>(chassis->GetPos() + ChVector<>(1.0, 0, 0), QUNIT));
        actuator->SetName("actuator");
        actuator->SetDistanceOffset(1);
        actuator->SetActuatorFunction(actuator_fun);
        sys_rover.AddLink(actuator);
    }
    
    auto prismatic_chassis = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic_chassis->Initialize(ground, moving_chassis, ChCoordsys<>(Rover_initial_pos, Q_from_AngY(CH_C_PI_2)));
    sys_rover.AddLink(prismatic_chassis);



    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic->Initialize(chassis, moving_chassis, ChCoordsys<>(chassis->GetPos(), QUNIT));
    sys_rover.AddLink(prismatic);

    //auto lock_link = chrono_types::make_shared<ChLinkLockLock>();
    //lock_link->Initialize(chassis, axle, ChCoordsys<>(Rover_initial_pos, Q_from_AngX(CH_C_PI_2)));
    //sys_rover.AddLink(lock_link);

    //=======================================================================================================================================================
    //Create wheel 1
    auto wheel_body1 = chrono_types::make_shared<ChBody>();
    wheel_body1->SetMass(wheel_mass /2);
    wheel_body1->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body1->SetPos(chassis->GetPos() + ChVector<>(-113.6, -92.3, 0));
    wheel_body1->SetPos_dt(IniVel);
    wheel_body1->SetBodyFixed(false);
    wheel_body1->SetCollide(false);

    sys_rover.AddBody(wheel_body1);


    // Create the axle 1
    auto axle1 = chrono_types::make_shared<ChBody>();
    axle1->SetMass(wheel_mass/2);
    axle1->SetInertiaXX(ChVector<>(1e9,1e9,1e9));
    axle1->SetPos(wheel_body1->GetPos());
    axle1->SetCollide(false);
    axle1->SetBodyFixed(false);

    sys_rover.AddBody(axle1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(chassis, axle1, ChCoordsys<>(chassis->GetPos()+ ChVector<>(-113.6, -92.3, 0), QUNIT));
    prismatic1->SetName("prismatic1_axle_chassis");
    sys_rover.AddLink(prismatic1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle1, ChCoordsys<>(chassis->GetPos() + ChVector<>(-113.6, -92.3, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic2->SetName("prismatic2_axle_chassis");
    sys_rover.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor1->SetName("motor1_plate_axle");
    motor1->Initialize(wheel_body1, axle1, ChFrame<>(wheel_body1->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor1->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor1);

    //=======================================================================================================================================================
    //Create wheel 2
    auto wheel_body2 = chrono_types::make_shared<ChBody>();
    wheel_body2->SetMass(wheel_mass / 2);
    wheel_body2->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body2->SetPos(chassis->GetPos() + ChVector<>(-113.6, 92.3, 0));
    wheel_body2->SetPos_dt(IniVel);
    wheel_body2->SetBodyFixed(false);
    wheel_body2->SetCollide(false);

    sys_rover.AddBody(wheel_body2);


    // Create the axle 2
    auto axle2 = chrono_types::make_shared<ChBody>();
    axle2->SetMass(wheel_mass / 2);
    axle2->SetInertiaXX(ChVector<>(1e9, 1e9, 1e9));
    axle2->SetPos(wheel_body2->GetPos());
    axle2->SetCollide(false);
    axle2->SetBodyFixed(false);

    sys_rover.AddBody(axle2);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic3 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic3->Initialize(chassis, axle2, ChCoordsys<>(chassis->GetPos() + ChVector<>(-113.6, 92.3, 0), QUNIT));
    prismatic3->SetName("prismatic3_axle_chassis");
    sys_rover.AddLink(prismatic3);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic4 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic4->Initialize(chassis, axle2, ChCoordsys<>(chassis->GetPos() + ChVector<>(-113.6, 92.3, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic4->SetName("prismatic4_axle_chassis");
    sys_rover.AddLink(prismatic4);

    // Connect the wheel to the axle through a engine joint.
    motor2->SetName("motor2_plate_axle");
    motor2->Initialize(wheel_body2, axle2, ChFrame<>(wheel_body2->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor2->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor2);



    //=======================================================================================================================================================
   //Create wheel 3
    auto wheel_body3 = chrono_types::make_shared<ChBody>();
    wheel_body3->SetMass(wheel_mass / 2);
    wheel_body3->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body3->SetPos(chassis->GetPos() + ChVector<>(0, 99.9, 0));
    wheel_body3->SetPos_dt(IniVel);
    wheel_body3->SetBodyFixed(false);
    wheel_body3->SetCollide(false);

    sys_rover.AddBody(wheel_body3);


    // Create the axle 3
    auto axle3 = chrono_types::make_shared<ChBody>();
    axle3->SetMass(wheel_mass / 2);
    axle3->SetInertiaXX(ChVector<>(1e9, 1e9, 1e9));
    axle3->SetPos(wheel_body3->GetPos());
    axle3->SetCollide(false);
    axle3->SetBodyFixed(false);

    sys_rover.AddBody(axle3);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic5 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic5->Initialize(chassis, axle3, ChCoordsys<>(chassis->GetPos() + ChVector<>(0, 99.9, 0), QUNIT));
    prismatic5->SetName("prismatic5_axle_chassis");
    sys_rover.AddLink(prismatic5);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic6 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic6->Initialize(chassis, axle3, ChCoordsys<>(chassis->GetPos() + ChVector<>(0, 99.9, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic6->SetName("prismatic6_axle_chassis");
    sys_rover.AddLink(prismatic6);

    // Connect the wheel to the axle through a engine joint.
    motor3->SetName("motor3_plate_axle");
    motor3->Initialize(wheel_body3, axle3, ChFrame<>(wheel_body3->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor3->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor3);
    

    //=======================================================================================================================================================
   //Create wheel 4
    auto wheel_body4 = chrono_types::make_shared<ChBody>();
    wheel_body4->SetMass(wheel_mass / 2);
    wheel_body4->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body4->SetPos(chassis->GetPos() + ChVector<>(0, -99.9, 0));
    wheel_body4->SetPos_dt(IniVel);
    wheel_body4->SetBodyFixed(false);
    wheel_body4->SetCollide(false);

    sys_rover.AddBody(wheel_body4);


    // Create the axle 4
    auto axle4 = chrono_types::make_shared<ChBody>();
    axle4->SetMass(wheel_mass / 2);
    axle4->SetInertiaXX(ChVector<>(1e9, 1e9, 1e9));
    axle4->SetPos(wheel_body4->GetPos());
    axle4->SetCollide(false);
    axle4->SetBodyFixed(false);

    sys_rover.AddBody(axle4);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic7 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic7->Initialize(chassis, axle4, ChCoordsys<>(chassis->GetPos() + ChVector<>(0, -99.9, 0), QUNIT));
    prismatic7->SetName("prismatic7_axle_chassis");
    sys_rover.AddLink(prismatic7);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic8 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic8->Initialize(chassis, axle4, ChCoordsys<>(chassis->GetPos() + ChVector<>(0, -99.9, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic8->SetName("prismatic8_axle_chassis");
    sys_rover.AddLink(prismatic8);

    // Connect the wheel to the axle through a engine joint.
    motor4->SetName("motor4_plate_axle");
    motor4->Initialize(wheel_body4, axle4, ChFrame<>(wheel_body4->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor4->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor4);


    //=======================================================================================================================================================
   //Create wheel 5
    auto wheel_body5 = chrono_types::make_shared<ChBody>();
    wheel_body5->SetMass(wheel_mass / 2);
    wheel_body5->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body5->SetPos(chassis->GetPos() + ChVector<>(113.6, -92.3, 0));
    wheel_body5->SetPos_dt(IniVel);
    wheel_body5->SetBodyFixed(false);
    wheel_body5->SetCollide(false);

    sys_rover.AddBody(wheel_body5);


    // Create the axle 5
    auto axle5 = chrono_types::make_shared<ChBody>();
    axle5->SetMass(wheel_mass / 2);
    axle5->SetInertiaXX(ChVector<>(1e9, 1e9, 1e9));
    axle5->SetPos(wheel_body5->GetPos());
    axle5->SetCollide(false);
    axle5->SetBodyFixed(false);

    sys_rover.AddBody(axle5);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic9 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic9->Initialize(chassis, axle5, ChCoordsys<>(chassis->GetPos() + ChVector<>(113.6, -92.3, 0), QUNIT));
    prismatic9->SetName("prismatic9_axle_chassis");
    sys_rover.AddLink(prismatic9);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic10 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic10->Initialize(chassis, axle5, ChCoordsys<>(chassis->GetPos() + ChVector<>(113.6, -92.3, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic10->SetName("prismatic10_axle_chassis");
    sys_rover.AddLink(prismatic10);

    // Connect the wheel to the axle through a engine joint.
    motor5->SetName("motor5_plate_axle");
    motor5->Initialize(wheel_body5, axle5, ChFrame<>(wheel_body5->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor5->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor5);


    //=======================================================================================================================================================
   //Create wheel 6
    auto wheel_body6 = chrono_types::make_shared<ChBody>();
    wheel_body6->SetMass(wheel_mass / 2);
    wheel_body6->SetInertiaXX(ChVector<>(inertia_x, inertia_y, inertia_z));
    wheel_body6->SetPos(chassis->GetPos() + ChVector<>(113.6, 92.3, 0));
    wheel_body6->SetPos_dt(IniVel);
    wheel_body6->SetBodyFixed(false);
    wheel_body6->SetCollide(false);

    sys_rover.AddBody(wheel_body6);


    // Create the axle 6
    auto axle6 = chrono_types::make_shared<ChBody>();
    axle6->SetMass(wheel_mass / 2);
    axle6->SetInertiaXX(ChVector<>(1e9, 1e9, 1e9));
    axle6->SetPos(wheel_body6->GetPos());
    axle6->SetCollide(false);
    axle6->SetBodyFixed(false);

    sys_rover.AddBody(axle6);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic11 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic11->Initialize(chassis, axle6, ChCoordsys<>(chassis->GetPos() + ChVector<>(113.6, 92.3, 0), QUNIT));
    prismatic11->SetName("prismatic11_axle_chassis");
    sys_rover.AddLink(prismatic11);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic12 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic12->Initialize(chassis, axle6, ChCoordsys<>(chassis->GetPos() + ChVector<>(113.6, 92.3, 0), Q_from_AngY(CH_C_PI_2)));
    prismatic12->SetName("prismatic12_axle_chassis");
    sys_rover.AddLink(prismatic12);

    // Connect the wheel to the axle through a engine joint.
    motor6->SetName("motor6_plate_axle");
    motor6->Initialize(wheel_body6, axle6, ChFrame<>(wheel_body6->GetPos(), chrono::Q_from_AngAxis(-CH_C_PI / 2.0, ChVector<>(1, 0, 0))));
    motor6->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, angVel));
    sys_rover.AddLink(motor6);



    //waiting for download
    ChVector<> torque = motor1->Get_react_torque();
    ChVector<> p_pos = wheel_body1->GetPos();
    



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
    
    // Create oputput directories  �������Ŀ¼
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
    std::ofstream myPos;
    std::ofstream mychassis;

    
    myFile.open(out_dir + "/data/results.txt", std::ios::trunc);
    myDBP.open(out_dir + "/data/DBP.txt", std::ios::trunc);
    myTorque.open(out_dir + "/data/Torque.txt", std::ios::trunc);
    myPos.open(out_dir + "/data/Pos.txt", std::ios::trunc);
    mychassis.open(out_dir + "/data/chassis.txt", std::ios::trunc);
    
        
        

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        gpu_sys.ApplyMeshMotion(0, wheel_body1->GetPos(), wheel_body1->GetRot(), wheel_body1->GetPos_dt(), wheel_body1->GetWvel_par());
        gpu_sys.ApplyMeshMotion(1, wheel_body2->GetPos(), wheel_body2->GetRot(), wheel_body2->GetPos_dt(), wheel_body2->GetWvel_par());
        gpu_sys.ApplyMeshMotion(2, wheel_body3->GetPos(), wheel_body3->GetRot(), wheel_body3->GetPos_dt(), wheel_body3->GetWvel_par());
        gpu_sys.ApplyMeshMotion(3, wheel_body4->GetPos(), wheel_body4->GetRot(), wheel_body4->GetPos_dt(), wheel_body4->GetWvel_par());
        gpu_sys.ApplyMeshMotion(4, wheel_body5->GetPos(), wheel_body5->GetRot(), wheel_body5->GetPos_dt(), wheel_body5->GetWvel_par());
        gpu_sys.ApplyMeshMotion(5, wheel_body6->GetPos(), wheel_body6->GetRot(), wheel_body6->GetPos_dt(), wheel_body6->GetWvel_par());

        ChVector<> ball_force1;
        ChVector<> ball_force2;
        ChVector<> ball_force3;
        ChVector<> ball_force4;
        ChVector<> ball_force5;
        ChVector<> ball_force6;

        ChVector<> ball_torque1;
        ChVector<> ball_torque2;
        ChVector<> ball_torque3;
        ChVector<> ball_torque4;
        ChVector<> ball_torque5;
        ChVector<> ball_torque6;

        gpu_sys.CollectMeshContactForces(0, ball_force1, ball_torque1);
        gpu_sys.CollectMeshContactForces(1, ball_force2, ball_torque2);
        gpu_sys.CollectMeshContactForces(2, ball_force3, ball_torque3);
        gpu_sys.CollectMeshContactForces(3, ball_force4, ball_torque4);
        gpu_sys.CollectMeshContactForces(4, ball_force5, ball_torque5);
        gpu_sys.CollectMeshContactForces(5, ball_force6, ball_torque6);

        wheel_body1->Empty_forces_accumulators();
        wheel_body1->Accumulate_force(ball_force1, wheel_body1->GetPos(), false);
        wheel_body1->Accumulate_torque(ball_torque1, false);

        wheel_body2->Empty_forces_accumulators();
        wheel_body2->Accumulate_force(ball_force2, wheel_body2->GetPos(), false);
        wheel_body2->Accumulate_torque(ball_torque2, false);

        wheel_body3->Empty_forces_accumulators();
        wheel_body3->Accumulate_force(ball_force3, wheel_body3->GetPos(), false);
        wheel_body3->Accumulate_torque(ball_torque3, false);

        wheel_body4->Empty_forces_accumulators();
        wheel_body4->Accumulate_force(ball_force4, wheel_body4->GetPos(), false);
        wheel_body4->Accumulate_torque(ball_torque4, false);

        wheel_body5->Empty_forces_accumulators();
        wheel_body5->Accumulate_force(ball_force5, wheel_body5->GetPos(), false);
        wheel_body5->Accumulate_torque(ball_torque5, false);

        wheel_body6->Empty_forces_accumulators();
        wheel_body6->Accumulate_force(ball_force6, wheel_body6->GetPos(), false);
        wheel_body6->Accumulate_torque(ball_torque6, false);
        


        //waiting for download
        torque = motor1->Get_react_torque();
        p_pos = wheel_body1->GetPos();
        

        double pos = (-12 - p_pos.z())/100;
        if (verbose) {
            std::cout << "Time: " << t << std::endl;
            std::cout << "  Plate position:         " << p_pos << std::endl;
            std::cout << "  Plate torque:           " << torque << std::endl;
            std::cout << "  Wvel:           " << wheel_body1->GetWvel_par() << std::endl;
            std::cout << "  wheel_V:           " << wheel_body1->GetPos_dt() << std::endl;
            std::cout << "  chassis_V:           " << chassis->GetPos_dt() << std::endl;
            std::cout << "  axle_V:           " << axle1->GetPos_dt() << std::endl;
            std::cout << "  Ball_force:           " << ball_force1 << std::endl;
            

        }
        myDBP << t << "\t" << wheel_slip << "\t" << pos << "\t" << ball_force1.x() << "\t" << ball_force1.y() << "\t" << ball_force1.z() << "\n";

        myTorque << t << "\t" << wheel_slip << "\t" << pos << "\t" << torque.x() << "\t" << torque.y() << "\t" << torque.z() << "\n";
        
        
        myFile << t << "\t" << p_pos.x() << "\t" << p_pos.y() << "\t" << p_pos.z() << "\t" << torque.x() << "\t" << torque.y() << "\t" << torque.z() << "\t" << "\n";


        myPos << t << "\t" << p_pos.x() << "\t" << p_pos.y() << "\t" << p_pos.z() << "\t" << wheel_body1->GetRot() << "\n";

        mychassis<<t<< "\t" << chassis->GetPos_dt()<< "\n";


    
    
    

        if (curr_step % out_steps == 0) {
            
            std::string file = out_dir + "/vtk/plate." + std::to_string(currframe) + ".vtk";
            //WriteCylinderVTK(file, plate_radius, plate_height, sys_rover.Get_bodylist()[1]->GetFrame_REF_to_abs(), total_frames);
            
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            char mesh_filename[100];
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe);
            sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), currframe++); //*************mesh ? ?    ?  ****************
            gpu_sys.WriteParticleFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(mesh_filename));
        }

        
        //if (render && curr_step % render_steps == 0) { if (!gpu_vis.Render())  break; }
        

        gpu_sys.AdvanceSimulation(iteration_step);
        sys_rover.DoStepDynamics(iteration_step);
    }
    
    myFile.close();
    myDBP.close();
    myTorque.close();
    myPos.close();
    mychassis.close();
	

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;
}



int main(int argc, char* argv[]) {
cudaSetDevice(1);
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
        gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);

        // Add a ball through a mesh, whose dynamics are managed by Chrono Core, and run this co-simulation.
        Rover(gpu_sys, params);

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
