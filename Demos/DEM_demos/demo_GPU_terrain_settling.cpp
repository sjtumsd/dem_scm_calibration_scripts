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


// Output frequency
float out_fps = 10;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 2000;

//*******************************************************************************************************************************************

int main(int argc, char* argv[]) {
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
