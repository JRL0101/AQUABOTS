# CMakeLists.txt for main component
# Step 2: Added node_config module

idf_component_register(
    SRCS 
        "main.c"
        "node_config.c"
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