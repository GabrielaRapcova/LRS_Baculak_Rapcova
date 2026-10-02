# Full replacement snapshot. Geometry and map_id are inherited from the map.
# Costmap sequence is independent of the map sequence.
std_msgs/Header header
float32 resolution
geometry_msgs/Point origin
uint64 map_id
uint64 sequence
VoxelBlock[] blocks
