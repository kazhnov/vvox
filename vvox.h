#include "include/vmath/vmath.h"

b8* VVOX_VerticesToVoxels(void* vertices, u32 vcount, u32 vstride, u32 voffset,
			   u32* indices,  u32 icount,
			   f32 voxel_size, u32 *voxel_count_out);

b8 VVOX_VoxelGet(b8 *voxels, u32 voxel_count[3], u32 x, u32 y, u32 z);

