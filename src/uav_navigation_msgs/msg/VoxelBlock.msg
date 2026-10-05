# Signed block coordinates, relative to the map origin. Edge length is 8 voxels.
# Local array index = x + 8 * (y + 8 * z), with local coordinates in [0, 7].
# Map values: 0 free, 254 occupied, 255 unknown.
# Costmap values additionally support 1-252 inflated and 253 inscribed.
uint8 FREE=0
uint8 INSCRIBED=253
uint8 LETHAL=254
uint8 UNKNOWN=255
uint8 BLOCK_SIZE=8
int32 x
int32 y
int32 z
uint8[512] occupancy
