# CMakeLists.txt for main component
# Step 1: Include all swarm module files

idf_component_register(
    SRCS 
        "main.c"
        "swarm_transport.c"
        "membership.c"
        "leader_election.c"
        "scheduler.c"
        "command_engine.c"
        "formation.c"
        "obstacle_sense.c"
        "avoidance.c"
    INCLUDE_DIRS 
        "."
)

# Optional: Support DEMO_ROLE as build-time define
# Usage: idf.py -D DEMO_ROLE=1 build  (for initiator)
#        idf.py build                 (for responder, default)
if(DEFINED ENV{DEMO_ROLE})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEMO_ROLE=$ENV{DEMO_ROLE})
    message(STATUS "Building with DEMO_ROLE=$ENV{DEMO_ROLE}")
endif()
