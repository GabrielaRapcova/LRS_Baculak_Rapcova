# Full replacement snapshot; absent blocks are unknown.
# Axis-aligned geometry in header.frame_id, metres, resolution must be positive.
std_msgs/Header header
float32 resolution
geometry_msgs/Point origin
# Increment on geometry reset; sequence increases for every update in this map_id.
uint64 map_id
uint64 sequence
VoxelBlock[] blocks
