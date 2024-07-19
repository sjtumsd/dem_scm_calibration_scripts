# ------------------------------------------------------------------------------
# Additional include paths and libraries
# ------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${CH_GPU_INCLUDES})

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_gpu
    ChronoModels_robot
)

# ------------------------------------------------------------------------------
# List of all executables
# ------------------------------------------------------------------------------

SET(DEMOS
demo_GPU_terrain_settling

demo_GPU_sinkage_v05_r20
demo_GPU_sinkage_v05_r30
demo_GPU_sinkage_v10_r20
demo_GPU_sinkage_v10_r30
demo_GPU_sinkage_v025_r20
demo_GPU_sinkage_v025_r30

demo_GPU_Annulus_shear_m25
demo_GPU_Annulus_shear_m50
demo_GPU_Annulus_shear_m75
demo_GPU_Annulus_shear_m100
demo_GPU_Annulus_shear_m125
demo_GPU_Annulus_shear_m150
demo_GPU_Annulus_shear_m175
demo_GPU_Annulus_shear_m200

demo_GPU_Singlewheel_Test_slip=0.0(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.1(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.2(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.3(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.4(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.5(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.6(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.7(Hmmwv)
demo_GPU_Singlewheel_Test_slip=0.8(Hmmwv)

demo_GPU_Singlewheel_Test_slip=0.0(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.1(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.2(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.3(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.4(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.5(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.6(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.7(Cylinder)
demo_GPU_Singlewheel_Test_slip=0.8(Cylinder)

demo_GPU_Singlewheel_Test_slip=0.0(Rover)
demo_GPU_Singlewheel_Test_slip=0.1(Rover)
demo_GPU_Singlewheel_Test_slip=0.2(Rover)
demo_GPU_Singlewheel_Test_slip=0.3(Rover)
demo_GPU_Singlewheel_Test_slip=0.4(Rover)
demo_GPU_Singlewheel_Test_slip=0.5(Rover)
demo_GPU_Singlewheel_Test_slip=0.6(Rover)
demo_GPU_Singlewheel_Test_slip=0.7(Rover)
demo_GPU_Singlewheel_Test_slip=0.8(Rover)

demo_GPU_Rover_slip=0.0
demo_GPU_Rover_slip=0.1
demo_GPU_Rover_slip=0.2
demo_GPU_Rover_slip=0.3
demo_GPU_Rover_slip=0.4
demo_GPU_Rover_slip=0.5
demo_GPU_Rover_slip=0.6
demo_GPU_Rover_slip=0.7
demo_GPU_Rover_slip=0.8

)

# ------------------------------------------------------------------------------
# Add all executables
# ------------------------------------------------------------------------------

MESSAGE(STATUS "Demo programs for Gpu module...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    ADD_EXECUTABLE(${PROGRAM}  "${PROGRAM}.cpp")
    SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    SET_TARGET_PROPERTIES(${PROGRAM} PROPERTIES
                          FOLDER demos
                          COMPILE_FLAGS "${CH_CXX_FLAGS} ${CH_GPU_CXX_FLAGS}"
                          LINK_FLAGS "${CH_LINKERFLAG_EXE}")
    SET_PROPERTY(TARGET ${PROGRAM} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:${PROGRAM}>")
    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION ${CH_INSTALL_DEMO})
ENDFOREACH(PROGRAM)
