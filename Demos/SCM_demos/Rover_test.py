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


T_tot = 6 + 1.0e-6
Dt = 0.001
N_step = int(T_tot / Dt) + 1
print("Total time steps: ", N_step)


  

# Parameters for wheel
wheel_rad = 0.257
        
chassis_centre = chrono.ChVectorD(0, wheel_rad+0.001, 0)
wheel_mass = 15
chassis_mass=15
slip =  [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]

nSims = len(slip)
#PV_tot = np.zeros((N_step, 4))

FT_tot = np.zeros((N_step, 2 * nSims + 1))
PV_tot = np.zeros((N_step, 4 * nSims + 1))
FT_Vs_Slip = np.zeros((nSims, 3))
#wheel_center = chrono.ChVectorD(0, wheel_rad+0.01, -2.5)

Loc1=chrono.ChVectorD(-0.923,0,-1.136)
Loc2=chrono.ChVectorD(0.923,0,-1.136)
Loc3=chrono.ChVectorD(-0.999,0,0)
Loc4=chrono.ChVectorD(0.999,0,0)
Loc5=chrono.ChVectorD(-0.923,0,1.136)
Loc6=chrono.ChVectorD(0.923,0,1.136)

for i in range(nSims):
    mysystem = chrono.ChSystemSMC()
    mysystem.Set_G_acc(chrono.ChVectorD(0,-9.8,0))
    # The path to the Chrono data directory containing various assets
    chrono.SetChronoDataPath('/home/yueminzhang/zym/build_py/data/')


    wheel_slip=slip[i]
    lin_vel=1    
    ang_vel=lin_vel/((1-wheel_slip)*wheel_rad )

    # Create the ground
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    mysystem.Add(ground)

    # Create the rigid wheel with contact mesh
    wheel_1 = chrono.ChBody()
    mysystem.Add(wheel_1)
    wheel_1.SetMass(wheel_mass/2.0)
    wheel_1.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_1.SetPos(chassis_centre+Loc1)
    wheel_1.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    wheel_2 = chrono.ChBody()
    mysystem.Add(wheel_2)
    wheel_2.SetMass(wheel_mass/2.0)
    wheel_2.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_2.SetPos(chassis_centre+Loc2)
    wheel_2.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    wheel_3 = chrono.ChBody()
    mysystem.Add(wheel_3)
    wheel_3.SetMass(wheel_mass/2.0)
    wheel_3.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_3.SetPos(chassis_centre+Loc3)
    wheel_3.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    wheel_4 = chrono.ChBody()
    mysystem.Add(wheel_4)
    wheel_4.SetMass(wheel_mass/2.0)
    wheel_4.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_4.SetPos(chassis_centre+Loc4)
    wheel_4.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    wheel_5 = chrono.ChBody()
    mysystem.Add(wheel_5)
    wheel_5.SetMass(wheel_mass/2.0)
    wheel_5.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_5.SetPos(chassis_centre+Loc5)
    wheel_5.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    wheel_6 = chrono.ChBody()
    mysystem.Add(wheel_6)
    wheel_6.SetMass(wheel_mass/2.0)
    wheel_6.SetInertiaXX(chrono.ChVectorD(10, 10,10))
    wheel_6.SetPos(chassis_centre+Loc6)
    wheel_6.SetPos_dt(chrono.ChVectorD(0.0, 0.0, 0.0))

    # Load mesh
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('vehicle/hmmwv/wheelcn.obj'))
            
    mesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.Q_from_AngZ(math.pi / 2)))  
            # Set visualization assets
            # vis_shape = chrono.ChTriangleMeshShape()
            # vis_shape.SetMesh(mesh)
            # wheel.AddAsset(vis_shape)
            # wheel.AddAsset(chrono.ChColorAsset(0.3, 0.3, 0.3))
            
            # Set collision shape
    material = chrono.ChMaterialSurfaceSMC()
    #material.SetFriction(0.9)
    #material.SetRollingFriction(0.9)
    #material.SetSpinningFriction(0.9)
    wheel_1.GetCollisionModel().ClearModel()
    wheel_1.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_1.GetCollisionModel().BuildModel()
    wheel_1.SetCollide(True)

    wheel_2.GetCollisionModel().ClearModel()
    wheel_2.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_2.GetCollisionModel().BuildModel()
    wheel_2.SetCollide(True)

    wheel_3.GetCollisionModel().ClearModel()
    wheel_3.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_3.GetCollisionModel().BuildModel()
    wheel_3.SetCollide(True)

    wheel_4.GetCollisionModel().ClearModel()
    wheel_4.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_4.GetCollisionModel().BuildModel()
    wheel_4.SetCollide(True)

    wheel_5.GetCollisionModel().ClearModel()
    wheel_5.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_5.GetCollisionModel().BuildModel()
    wheel_5.SetCollide(True)

    wheel_6.GetCollisionModel().ClearModel()
    wheel_6.GetCollisionModel().AddTriangleMesh(material,                  # contact material
                                                    mesh,                    # the mesh 
                                                    False,                   # is it static?
                                                    False,                   # is it convex?
                                                    chrono.ChVectorD(0,0,0), # position on wheel
                                                    chrono.ChMatrix33D(1),   # orientation on wheel 
                                                    0.01)                    # "thickness" for increased robustness
    wheel_6.GetCollisionModel().BuildModel()
    wheel_6.SetCollide(True)




    # Create chassis
    chassis = chrono.ChBody()
    mysystem.Add(chassis)
    chassis.SetMass(chassis_mass)
    chassis.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    chassis.SetPos(chassis_centre)
    chassis.SetCollide(False)


    #Create the constant velocity moving chassis
    moving_chassis = chrono.ChBody()
    mysystem.Add(moving_chassis)
    moving_chassis.SetMass(chassis_mass)
    moving_chassis.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    moving_chassis.SetPos(chassis_centre)
    moving_chassis.SetCollide(False)

    actuator = chrono.ChLinkLinActuator()
    actuator.SetActuatorFunction(chrono.ChFunction_Ramp(0, lin_vel))
    actuator.SetDistanceOffset(2)
    actuator.Initialize(ground, moving_chassis, False, chrono.ChCoordsysD(chassis_centre, chrono.Q_from_AngY(math.pi/2)), 
                chrono.ChCoordsysD(chassis_centre + chrono.ChVectorD(0, 0, 2.0), chrono.Q_from_AngY(math.pi/2)) )
    mysystem.AddLink(actuator)


    prismatic_chassis = chrono.ChLinkLockPrismatic()
    prismatic_chassis.Initialize(ground, moving_chassis, chrono.ChCoordsysD(chassis_centre, chrono.QUNIT))
    mysystem.AddLink(prismatic_chassis)
            
            
    # Create axle
    axle = chrono.ChBody()
    mysystem.Add(axle)
    axle.SetMass(chassis_mass)
    axle.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle.SetPos(chassis_centre)
    axle.SetCollide(False)


    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(axle, moving_chassis, chrono.ChCoordsysD(chassis_centre, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic)

    #m_revolute = chrono.ChLinkLockRevolute()
    #m_revolute.Initialize(chassis, axle, chrono.ChCoordsysD(chassis_centre, chrono.Q_from_AngY(math.pi/2)))
    #mysystem.AddLink(m_revolute)

    lock_link = chrono.ChLinkLockLock()
    lock_link.Initialize(chassis, axle, chrono.ChCoordsysD(chassis_centre, chrono.QUNIT))
    mysystem.AddLink(lock_link)





    # Create axle
    axle1 = chrono.ChBody()
    mysystem.Add(axle1)
    axle1.SetMass(wheel_mass/2)
    axle1.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle1.SetPos(wheel_1.GetPos())
    axle1.SetCollide(False)

    axle2 = chrono.ChBody()
    mysystem.Add(axle2)
    axle2.SetMass(wheel_mass/2)
    axle2.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle2.SetPos(wheel_2.GetPos())
    axle2.SetCollide(False)

    axle3 = chrono.ChBody()
    mysystem.Add(axle3)
    axle3.SetMass(wheel_mass/2)
    axle3.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle3.SetPos(wheel_3.GetPos())
    axle3.SetCollide(False)

    axle4 = chrono.ChBody()
    mysystem.Add(axle4)
    axle4.SetMass(wheel_mass/2)
    axle4.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle4.SetPos(wheel_4.GetPos())
    axle4.SetCollide(False)

    axle5 = chrono.ChBody()
    mysystem.Add(axle5)
    axle5.SetMass(wheel_mass/2)
    axle5.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle5.SetPos(wheel_5.GetPos())
    axle5.SetCollide(False)

    axle6 = chrono.ChBody()
    mysystem.Add(axle6)
    axle6.SetMass(wheel_mass/2)
    axle6.SetInertiaXX(chrono.ChVectorD(10, 10, 10))
    axle6.SetPos(wheel_6.GetPos())
    axle6.SetCollide(False)



    # Create motor, actuator, and links
    motor1 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor1.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor1.Initialize(wheel_1, axle1, chrono.ChFrameD(wheel_1.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor1)

    # Create motor, actuator, and links
    motor2 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor2.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor2.Initialize(wheel_2, axle2, chrono.ChFrameD(wheel_2.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor2)

    # Create motor, actuator, and links
    motor3 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor3.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor3.Initialize(wheel_3, axle3, chrono.ChFrameD(wheel_3.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor3)

    # Create motor, actuator, and links
    motor4 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor4.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor4.Initialize(wheel_4, axle4, chrono.ChFrameD(wheel_4.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor4)

    # Create motor, actuator, and links
    motor5 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor5.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor5.Initialize(wheel_5, axle5, chrono.ChFrameD(wheel_5.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor5)

    # Create motor, actuator, and links
    motor6 = chrono.ChLinkMotorRotationAngle()
    # motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
    motor6.SetAngleFunction(chrono.ChFunction_Ramp(0, ang_vel))
    motor6.Initialize(wheel_6, axle6, chrono.ChFrameD(wheel_6.GetPos(), chrono.Q_from_AngY(math.pi/2)))
    mysystem.Add(motor6)

    prismatic1 = chrono.ChLinkLockPrismatic()
    prismatic1.Initialize(chassis, axle1, chrono.ChCoordsysD(chassis_centre+Loc1, chrono.QUNIT))
    mysystem.AddLink(prismatic1)

    prismatic2 = chrono.ChLinkLockPrismatic()
    prismatic2.Initialize(chassis, axle1, chrono.ChCoordsysD(chassis_centre+Loc1, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic2)

    prismatic3 = chrono.ChLinkLockPrismatic()
    prismatic3.Initialize(chassis, axle2, chrono.ChCoordsysD(chassis_centre+Loc2, chrono.QUNIT))
    mysystem.AddLink(prismatic3)

    prismatic4 = chrono.ChLinkLockPrismatic()
    prismatic4.Initialize(chassis, axle2, chrono.ChCoordsysD(chassis_centre+Loc2, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic4)

    prismatic5 = chrono.ChLinkLockPrismatic()
    prismatic5.Initialize(chassis, axle3, chrono.ChCoordsysD(chassis_centre+Loc3, chrono.QUNIT))
    mysystem.AddLink(prismatic5)

    prismatic6 = chrono.ChLinkLockPrismatic()
    prismatic6.Initialize(chassis, axle3, chrono.ChCoordsysD(chassis_centre+Loc3, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic6)

    prismatic7 = chrono.ChLinkLockPrismatic()
    prismatic7.Initialize(chassis, axle4, chrono.ChCoordsysD(chassis_centre+Loc4, chrono.QUNIT))
    mysystem.AddLink(prismatic7)

    prismatic8 = chrono.ChLinkLockPrismatic()
    prismatic8.Initialize(chassis, axle4, chrono.ChCoordsysD(chassis_centre+Loc4, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic8)

    prismatic9 = chrono.ChLinkLockPrismatic()
    prismatic9.Initialize(chassis, axle5, chrono.ChCoordsysD(chassis_centre+Loc5, chrono.QUNIT))
    mysystem.AddLink(prismatic9)

    prismatic10 = chrono.ChLinkLockPrismatic()
    prismatic10.Initialize(chassis, axle5, chrono.ChCoordsysD(chassis_centre+Loc5, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic10)

    prismatic11 = chrono.ChLinkLockPrismatic()
    prismatic11.Initialize(chassis, axle6, chrono.ChCoordsysD(chassis_centre+Loc6, chrono.QUNIT))
    mysystem.AddLink(prismatic11)

    prismatic12 = chrono.ChLinkLockPrismatic()
    prismatic12.Initialize(chassis, axle6, chrono.ChCoordsysD(chassis_centre+Loc6, chrono.Q_from_AngX(-math.pi/2)))
    mysystem.AddLink(prismatic12)








    # Create SCM terrain patch
    # Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). 
    # Since the mechanism is modeled here in a Y-up global frame, we rotate
    # the terrain plane by -90 degrees about the X axis.
    terrain = veh.SCMDeformableTerrain(mysystem)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-math.pi/2)))
            #terrain.Initialize(2.0, 20.0, 0.02)#路长，网格大小最后一个，可调
    terrain.Initialize(4.0, 40.0, 0.01)
    #terrain.EnableBulldozing(True)

            # Constant soil properties
    #damping = 0.8e6
    damping = 2e6
           
    terrain.SetSoilParameters(  235605,      # Bekker Kphi 0.2e6
                                        -4957,      # Bekker Kc 0
                                        0.883,        # Bekker n exponent 1.1
                                        21.872,       # Mohr cohesive limit (Pa) 0
                                        21.259,       # Mohr friction limit (degrees) 30
                                        0.0062,       # Janosi shear coefficient (m) 0.01
                                        1e6,        # Elastic stiffness (Pa/m), before plastic yield, must be > Kphi 4e7
                                        damping     # Damping (Pa s/m), proportional to negative vertical speed (optional) 3e4
    )

            
    #nn = 0
    # Run the simulation
    #while(mysystem.GetChTime() < T_tot):    
    #    mysystem.DoStepDynamics(Dt)
              
    #    PV_tot[nn][0] = mysystem.GetChTime()
    #    PV_tot[nn][1] = chassis.GetPos_dt().z
    #    PV_tot[nn][2] = chassis.GetPos_dt().y
    #    PV_tot[nn][2] = chassis.GetPos_dt().x
    #    nn = nn + 1


    F_final = 0.0
    T_final = 0.0
    S_final = 0.0
    ni = 0
    nn = 0
    # Run the simulation
    while(mysystem.GetChTime() < T_tot):    
        mysystem.DoStepDynamics(Dt)
        if mysystem.GetChTime() > 2.0:
            F_final = F_final - actuator.Get_react_force().x
            T_final = T_final - motor1.Get_react_torque().z
            S_final = S_final + (wheel_rad - wheel_1.GetPos().y)
            ni = ni + 1
        FT_tot[nn][0] = mysystem.GetChTime()
        FT_tot[nn][i + 1] = - motor1.Get_react_force().x
        FT_tot[nn][i + nSims + 1] = - motor1.Get_react_torque().z
        PV_tot[nn][0] = mysystem.GetChTime()
        PV_tot[nn][i + 0 * nSims + 1] = wheel_1.GetPos().z
        PV_tot[nn][i + 1 * nSims + 1] = wheel_1.GetPos().y
        PV_tot[nn][i + 2 * nSims + 1] = wheel_1.GetPos_dt().z
        PV_tot[nn][i + 3 * nSims + 1] = wheel_1.GetPos_dt().y
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
#txt_pos = ""
#np.savetxt(txt_pos + "DBP_Torque_vs_Time_HMMWV_Wheel_SCM.txt", FT_tot, delimiter=" ")
np.savetxt(txt_pos + "DBP_Torque.txt", FT_Vs_Slip, delimiter=" ")
#np.savetxt(txt_pos + "Pos_Vel_vs_Time_HMMWV_Wheel_SCM.txt", PV_tot, delimiter=" ")
#np.savetxt(txt_pos + "V_T_SLOPE=0.txt", PV_tot, delimiter=" ")
    
 
 