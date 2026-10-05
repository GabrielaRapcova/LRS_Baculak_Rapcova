# Replace complete blocks. An all-unknown block clears a previously known block.
# Apply only to matching map_id and sequence == current sequence + 1.
# On a gap, wait for a new snapshot; geometry comes exclusively from that snapshot.
std_msgs/Header header
uint64 map_id
uint64 sequence
VoxelBlock[] blocks
