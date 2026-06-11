# CriticsStats.msg
# Statistics about each MPPI critic's performance, published for visualization and debugging.
# Visualize with PlotJuggler, Foxglove, or similar tools to analyze critic behavior.

builtin_interfaces/Time stamp  # Timestamp of when the statistics were computed

string[] critics                # Names of critics that were evaluated (e.g. "ConstraintCritic", "GoalCritic")
bool[]   changed                # Whether each critic modified the trajectory costs (true = added non-zero costs)
float32[] costs_sum             # Total cost contribution from each critic across all trajectory candidates
