# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
# 
# Author: Wei Hu
# 
# =============================================================================
# Calibration of the SCM parameters using Bayesian Optimization
# =============================================================================

import numpy as np
#import scipy.io as sio
# import matplotlib.pyplot as plt
# import ctypes
# from numpy.ctypeslib import ndpointer
# import pymc3 as pm
# import arviz as az
# # import pred
# import theano
# import theano.tensor as tt

import pychrono.core as chrono
import pychrono.vehicle as veh
import time
import math
import random

# Three different wheels
for n in range(3):
    slip_hmmwv = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    slip_cylinder = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    slip_rover = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

    if n == 0:
        print("Run single wheel test for HMMWV wheel")
    if n == 1:
        print("Run single wheel test for Cylinder wheel")
    if n == 1:
        print("Run single wheel test for Rover wheel")

    slip = []
    if n == 0:
        slip = slip_hmmwv
    if n == 1:
        slip = slip_cylinder
    if n == 2:
        slip = slip_rover

    nSims = len(slip)
    print("Number of data is: ", nSims)

    T_tot = 15 + 1.0e-6
    Dt = 0.0005
    N_step = int(T_tot / Dt) + 1
    print("Total time steps: ", N_step)

    FT_tot = np.zeros((N_step, 2 * nSims + 1))
    PV_tot = np.zeros((N_step, 4 * nSims + 1))
    FT_Vs_Slip = np.zeros((nSims, 3))

    for i in range(nSims):
        # Create the mechanical system
        mysystem = chrono.ChSystemSMC()

        # The path to the Chrono data directory containing various assets
        chrono.SetChronoDataPath('/home/yueminzhang/zym/build_py/data/')

        # Parameters for wheel
        wheel_rad = 0.47
        if n == 2:
            wheel_rad = 0.25

        # wheel_width = 0.293

        wheel_center = chrono.ChVectorD(0, wheel_rad+0.01, -2.5)

        total_mass = 20.0
        part_mass = total_mass / 2.0

        wheel_slip = slip[i]

        lin_vel = 1

        ang_vel = lin_vel / ( (1.0 - wheel_slip) * wheel_rad )

        # Create the ground
        ground = chrono.ChBody()
        ground.SetBodyFixed(True)
        mysystem.Add(ground)

        # Create the rigid wheel with contact mesh
        wheel = chrono.ChBody()
        mysystem.Add(wheel)
        wheel.SetMass(total_mass - part_mass)
        wheel.SetInertiaXX(chrono.ChVectorD(10, 10,10))
        wheel.SetPos(wheel_center)
        wheel.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

        # Load mesh
        mesh = chrono.ChTriangleMeshConnected()
        if n == 0:
            mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/hmmwv/hmmwv_tire_fine.obj'))
        if n == 1:
            mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('robot/viper/obj/cylinder_47.obj'))
        if n == 1:
            mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('robot/viper/obj/rover_wheel.obj'))
        # mesh.LoadWavefrontMesh('wheel_10mm_grouser.obj')
        # mesh.LoadWavefrontMesh('hmmwv_tire_coarse_fine.obj')
        mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.Q_from_AngZ(math.pi / 2)))  

        # Set visualization assets
        # vis_shape = chrono.ChTriangleMeshShape()
        # vis_shape.SetMesh(mesh)
        # wheel.AddAsset(vis_shape)
        # wheel.AddAsset(chrono.ChColorAsset(0.3, 0.3, 0.3))

        # Set collision shape
        material = chrono.ChMaterialSurfaceSMC()
        #material.SetFriction(0.9);
        #material.SetRollingFriction(0.9)
        #material.SetSpinningFriction(0.9)
        wheel.GetCollisionModel().ClearModel()
        wheel.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
        wheel.GetCollisionModel().BuildModel()
        wheel.SetCollide(True)

        # wheel.GetCollisionModel().ClearModel()
        # wheel.GetCollisionModel().AddCylinder(material, wheel_rad, wheel_rad, wheel_width * 0.5, chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.Q_from_AngZ(math.pi/2)))
        # wheel.GetCollisionModel().BuildModel()
        # wheel.SetCollide(True)

        # Create chassis
        chassis = chrono.ChBody()
        mysystem.Add(chassis)
        chassis.SetMass(part_mass)
        chassis.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
        chassis.SetPos(wheel_center)
        chassis.SetCollide(False)

        # Create axle
        axle = chrono.ChBody()
        mysystem.Add(axle)
        axle.SetMass(part_mass)
        axle.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
        axle.SetPos(wheel_center)
        axle.SetCollide(False)

        # Create motor, actuator, and links
        motor = chrono.ChLinkMotorRotationAngle()
        # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
        motor.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
        motor.Initialize(wheel, axle, chrono.ChFrameD(wheel_center, chrono.Q_from_AngY(math.pi/2)))
        mysystem.Add(motor)

        actuator = chrono.ChLinkLinActuator()
        actuator.SetActuatorFunction(chrono.ChFunction_Ramp(0, lin_vel))
        actuator.SetDistanceOffset(2)
        actuator.Initialize(ground, chassis, False, chrono.ChCoordsysD(wheel_center, chrono.Q_from_AngY(math.pi/2)), 
            chrono.ChCoordsysD(wheel_center + chrono.ChVectorD(0, 0, 2.0), chrono.Q_from_AngY(math.pi/2)) )
        mysystem.AddLink(actuator)

        prismatic1 = chrono.ChLinkLockPrismatic()
        prismatic1.Initialize(ground, chassis, chrono.ChCoordsysD(wheel_center, chrono.QUNIT))
        mysystem.AddLink(prismatic1)

        prismatic2 = chrono.ChLinkLockPrismatic()
        prismatic2.Initialize(axle, chassis, chrono.ChCoordsysD(wheel_center, chrono.Q_from_AngX(-math.pi/2)))
        mysystem.AddLink(prismatic2)

        # Create SCM terrain patch
        # Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). 
        # Since the mechanism is modeled here in a Y-up global frame, we rotate
        # the terrain plane by -90 degrees about the X axis.
        terrain = veh.SCMDeformableTerrain(mysystem)
        terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-math.pi/2)))
        #terrain.Initialize(2.0, 20.0, 0.02)#路长，网格大小最后一个，可调
        terrain.Initialize(2.0, 30.0, 0.01)
        #terrain.EnableBulldozing(True)

        # Constant soil properties
        damping = 2e6
       
        terrain.SetSoilParameters(  235605,      # Bekker Kphi 0.2e6
                                    -4957,      # Bekker Kc 0
                                    0.883,        # Bekker n exponent 1.1
                                    21.872,       # Mohr cohesive limit (Pa) 0
                                    21.259,       # Mohr friction limit (degrees) 30
                                    0.0062,       # Janosi shear coefficient (m) 0.01
                                    1.2e6,        # Elastic stiffness (Pa/m), before plastic yield, must be > Kphi 4e7
                                    damping     # Damping (Pa s/m), proportional to negative vertical speed (optional) 3e4
        )

        F_final = 0.0
        T_final = 0.0
        S_final = 0.0
        ni = 0
        nn = 0
        # Run the simulation
        while(mysystem.GetChTime() < T_tot):    
            mysystem.DoStepDynamics(Dt)
            if mysystem.GetChTime() > 2.0:
                F_final = F_final - motor.Get_react_force().x
                T_final = T_final - motor.Get_react_torque().z
                S_final = S_final + (wheel_rad - wheel.GetPos().y)
                ni = ni + 1
            FT_tot[nn][0] = mysystem.GetChTime()
            FT_tot[nn][i + 1] = - motor.Get_react_force().x
            FT_tot[nn][i + nSims + 1] = - motor.Get_react_torque().z
            PV_tot[nn][0] = mysystem.GetChTime()
            PV_tot[nn][i + 0 * nSims + 1] = wheel.GetPos().z
            PV_tot[nn][i + 1 * nSims + 1] = wheel.GetPos().y
            PV_tot[nn][i + 2 * nSims + 1] = wheel.GetPos_dt().z
            PV_tot[nn][i + 3 * nSims + 1] = wheel.GetPos_dt().y
            nn = nn + 1

        F_final = F_final / ni 
        T_final = T_final / ni
        S_final = S_final / ni

        print(wheel_slip, " ", F_final, " ", T_final, " ", S_final)
        FT_Vs_Slip[i][0] = slip[i]
        FT_Vs_Slip[i][1] = F_final
        FT_Vs_Slip[i][2] = T_final
        # FT_Vs_Slip[i][3] = slip[i]

        del mysystem
    txt_pos = ""
    if n == 0:
        np.savetxt(txt_pos + "DBP_Torque_vs_Time_HMMWV_Wheel_SCM.txt", FT_tot, delimiter=" ")
        np.savetxt(txt_pos + "DBP_Torque_vs_Slip_HMMWV_Wheel_SCM.txt", FT_Vs_Slip, delimiter=" ")
        np.savetxt(txt_pos + "Pos_Vel_vs_Time_HMMWV_Wheel_SCM.txt", PV_tot, delimiter=" ")
    if n == 1:
        np.savetxt(txt_pos + "DBP_Torque_vs_Time_CYLINDER_Wheel_SCM.txt", FT_tot, delimiter=" ")
        np.savetxt(txt_pos + "DBP_Torque_vs_Slip_CYLINDER_Wheel_SCM.txt", FT_Vs_Slip, delimiter=" ")
        np.savetxt(txt_pos + "Pos_Vel_vs_Time_CYLINDER_Wheel_SCM.txt", PV_tot, delimiter=" ")
    if n == 2:
        np.savetxt(txt_pos + "DBP_Torque_vs_Time_ROVER_Wheel_SCM.txt", FT_tot, delimiter=" ")
        np.savetxt(txt_pos + "DBP_Torque_vs_Slip_ROVER_Wheel_SCM.txt", FT_Vs_Slip, delimiter=" ")
        np.savetxt(txt_pos + "Pos_Vel_vs_Time_ROVER_Wheel_SCM.txt", PV_tot, delimiter=" ")
 