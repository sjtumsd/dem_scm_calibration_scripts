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

demo_GPU_Singlewheel_Test_slip0.0Hmmwv
demo_GPU_Singlewheel_Test_slip0.1Hmmwv
demo_GPU_Singlewheel_Test_slip0.2Hmmwv
demo_GPU_Singlewheel_Test_slip0.3Hmmwv
demo_GPU_Singlewheel_Test_slip0.4Hmmwv
demo_GPU_Singlewheel_Test_slip0.5Hmmwv
demo_GPU_Singlewheel_Test_slip0.6Hmmwv
demo_GPU_Singlewheel_Test_slip0.7Hmmwv
demo_GPU_Singlewheel_Test_slip0.8Hmmwv

demo_GPU_Singlewheel_Test_slip0.0Cylinder
demo_GPU_Singlewheel_Test_slip0.1Cylinder
demo_GPU_Singlewheel_Test_slip0.2Cylinder
demo_GPU_Singlewheel_Test_slip0.3Cylinder
demo_GPU_Singlewheel_Test_slip0.4Cylinder
demo_GPU_Singlewheel_Test_slip0.5Cylinder
demo_GPU_Singlewheel_Test_slip0.6Cylinder
demo_GPU_Singlewheel_Test_slip0.7Cylinder
demo_GPU_Singlewheel_Test_slip0.8Cylinder

demo_GPU_Singlewheel_Test_slip0.0Rover
demo_GPU_Singlewheel_Test_slip0.1Rover
demo_GPU_Singlewheel_Test_slip0.2Rover
demo_GPU_Singlewheel_Test_slip0.3Rover
demo_GPU_Singlewheel_Test_slip0.4Rover
demo_GPU_Singlewheel_Test_slip0.5Rover
demo_GPU_Singlewheel_Test_slip0.6Rover
demo_GPU_Singlewheel_Test_slip0.7Rover
demo_GPU_Singlewheel_Test_slip0.8Rover

demo_GPU_Rover_slip0.0
demo_GPU_Rover_slip0.1
demo_GPU_Rover_slip0.2
demo_GPU_Rover_slip0.3
demo_GPU_Rover_slip0.4
demo_GPU_Rover_slip0.5
demo_GPU_Rover_slip0.6
demo_GPU_Rover_slip0.7
demo_GPU_Rover_slip0.8

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
