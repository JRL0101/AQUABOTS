# CMakeLists.txt for main component
# Step 3: Updated membership module (no longer stub)

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
