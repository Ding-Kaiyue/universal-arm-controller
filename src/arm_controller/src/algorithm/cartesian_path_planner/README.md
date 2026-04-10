# Cartesian Path Planner

3D Cartesian path planning module for obstacle avoidance.

Goals:
- Run first with simple primitive obstacles and dummy distance field.
- Keep interfaces stable for future ESDF integration.
- Separate planning, map, collision checking, smoothing, and sampling.

Current status:
- Core type system and layered interfaces are scaffolded.
- A minimal 3D A* planner skeleton is provided.
- Dummy distance field and basic collision/clearance utilities are provided.

Namespace:
`arm_controller::algorithm::cartesian_path_planner`
