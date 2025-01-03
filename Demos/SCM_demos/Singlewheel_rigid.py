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

# Two different wheels
for n in range(1):
    slip_hmmwv = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    #slip_hmmwv = [0.0]
    #slip_hmmwv = [0.0,  0.2,  0.4,  0.6,  0.8]
    slip_viper = [0.0, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

    if n == 0:
        print("Run single wheel test for HMMWV wheel")
    if n == 1:
        print("Run single wheel test for VIPER wheel")

    slip = []
    if n == 0:
        slip = slip_hmmwv
    if n == 1:
        slip = slip_viper

    nSims = len(slip)
    print("Number of data is: ", nSims)

    T_tot = 15 + 1.0e-6
    Dt = 0.001
    N_step = int(T_tot / Dt) + 1
    print("Total time steps: ", N_step)

    FT_tot = np.zeros((N_step, 2 * nSims + 1))
    PV_tot = np.zeros((N_step, 4 * nSims + 1))
    FT_Vs_Slip = np.zeros((nSims, 3))

    for i in range(nSims):
        # Create the mechanical system
        mysystem = chrono.ChSystemSMC()
        mysystem.Set_G_acc(chrono.ChVectorD(0,-1.62,0.0))
        # The path to the Chrono data directory containing various assets
        chrono.SetChronoDataPath('/home/yueminzhang/zym/build_py/data/')

        # Parameters for wheel
        wheel_rad = 0.257
        

        wheel_center = chrono.ChVectorD(0, wheel_rad+0.5, -5)

        total_mass = 22.0
        part_mass = total_mass / 2.0

        wheel_slip = slip[i]

        ang_vel = 0.33
        

        lin_vel = ang_vel * ( (1.0 - wheel_slip) * wheel_rad )


        material = chrono.ChMaterialSurfaceSMC()
        material.SetFriction(0.9);
        #material.SetRollingFriction(0.9)
        #material.SetSpinningFriction(0.9)
        
        #Create the ground
        #ground = chrono.ChBody()
        ground = chrono.ChBodyEasyBox(10, 1, 30, 1000, False, True, material);
        ground.SetPos(chrono.ChVectorD(0.0, 0.0, 0.0 ));
        ground.SetCollide(True)
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
            #mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/hmmwv/cylinder_47.obj'))
            mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/hmmwv/wheelcn.obj'))
        if n == 1:
            mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('robot/viper/obj/viper_wheel.obj'))
        # mesh.LoadWavefrontMesh('wheel_10mm_grouser.obj')
        # mesh.LoadWavefrontMesh('hmmwv_tire_coarse_fine.obj')
        mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.Q_from_AngZ(math.pi / 2)))  

        # Set visualization assets
        # vis_shape = chrono.ChTriangleMeshShape()
        # vis_shape.SetMesh(mesh)
        # wheel.AddAsset(vis_shape)
        # wheel.AddAsset(chrono.ChColorAsset(0.3, 0.3, 0.3))

        
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

        F_final = 0.0
        T_final = 0.0
        S_final = 0.0
        ni = 0
        nn = 0
        # Run the simulation
        while(mysystem.GetChTime() < T_tot):    
            mysystem.DoStepDynamics(Dt)
            if mysystem.GetChTime() > 5.0:
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
        #np.savetxt(txt_pos + "Torque_vs_Time_.txt", FT_tot, delimiter=" ")
        np.savetxt(txt_pos + "DBP_Torque_vs_Slip_Moon_Gravity_0.33rad.txt", FT_Vs_Slip, delimiter=" ")
        #np.savetxt(txt_pos + "Pos_Vel_vs_Time.txt", PV_tot, delimiter=" ")
    if n == 1:
        np.savetxt(txt_pos + "DBP_Torque_vs_Time_VIPER_Wheel_SCM.txt", FT_tot, delimiter=" ")
        np.savetxt(txt_pos + "DBP_Torque_vs_Slip_VIPER_Wheel_SCM.txt", FT_Vs_Slip, delimiter=" ")
        np.savetxt(txt_pos + "Pos_Vel_vs_Time_VIPER_Wheel_SCM.txt", PV_tot, delimiter=" ")
 