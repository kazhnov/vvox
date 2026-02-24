#include "vvox.h"
#include "include/vmath/vmath.h"
#include "include/vshape/vshape.h"
#include <math.h>
#include <float.h>

#define FACE_FOUND (1 << 0)
#define FACE_CCW (1 << 1)
#define CLAMP(x, l, h) ((x) < (l) ? (l) : (x) > (h) ? (h) : (x))

typedef struct {
    f32 depth;
    u32 flags;
} FaceProbe;

typedef struct {
    FaceProbe *probes;
    u32 count;
    u32 capacity;
} FaceProbes;

void iVVOX_FaceProbesInit(FaceProbes* face_probes, u32 capacity) {
    if (capacity < 2) capacity = 2;
    face_probes->probes = malloc(capacity*sizeof(FaceProbe));
    face_probes->capacity = capacity;
    face_probes->count = 0;
}

u32 iVVOX_FaceProbesPush(FaceProbes* probes, FaceProbe probe) {
    u32 new_count = probes->count + 1;
    if (new_count > probes->capacity) {
	probes->capacity += 2;
	probes->probes = realloc(probes->probes, probes->capacity*sizeof(FaceProbe));
    }
    probes->probes[probes->count] = probe;
    probes->count = new_count;
    return new_count - 1;
}

int iVVOX_FaceProbeCompare(const void * a, const void *b) {
    const FaceProbe* probe_a = a;
    const FaceProbe* probe_b = b;
    if (probe_a->depth < probe_b->depth) return -1;
    if (probe_a->depth > probe_b->depth) return 1;
    return 0;
}

void iVVOX_FaceProbesSort(FaceProbes* probes) {
    qsort(probes->probes, probes->count, sizeof(FaceProbe), iVVOX_FaceProbeCompare);

}

FaceProbe* iVVOX_FaceProbesPointerGet(FaceProbes* probes, u32 probe) {
    if (probe >= probes->count) {
	assert(false);
	return NULL;
    }
    return probes->probes + probe;
}

void iVVOX_FaceProbesDestroy(FaceProbes* probes) {
    free(probes->probes);
}

float iVVOX_Sign(f32 p1[3], f32 p2[3], f32 p3[3]) {
    return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]);
}

u32 iVVOX_DepthToVoxel(f32 depth, f32 min, f32 max, f32 voxel_size) {
    return CLAMP(floorf((depth - min)/voxel_size), min, max);
}

FaceProbe iVVOX_TriangleFaceProbeGet(f32 triangle[3][3], f32 ray_position[3], f32 size) {
    f32 low_bound = ray_position[1] - size/2;
    f32 high_bound = ray_position[1] + size/2;
    f32 left_bound = ray_position[0] - size/2;
    f32 right_bound = ray_position[0] + size/2;

    b8 triangle_lower =
	triangle[0][1] < low_bound &&
	triangle[1][1] < low_bound &&
	triangle[2][1] < low_bound;
    if (triangle_lower) return (FaceProbe){0};
    b8 triangle_higher =
	triangle[0][1] > high_bound &&
	triangle[1][1] > high_bound &&
	triangle[2][1] > high_bound;
    if (triangle_higher) return (FaceProbe){0};
    b8 triangle_righter =
	triangle[0][0] > right_bound &&
	triangle[1][0] > right_bound &&
	triangle[2][0] > right_bound;
    if (triangle_righter) return (FaceProbe){0};    
    b8 triangle_lefter =
	triangle[0][0] < left_bound &&
	triangle[1][0] < left_bound &&
	triangle[2][0] < left_bound;
    if (triangle_lefter) return (FaceProbe){0};
    
    f32 ab[3]; f32 bc[3]; f32 ca[3];
    VM3_SubtractO(triangle[1], triangle[0], ab);
    VM3_SubtractO(triangle[2], triangle[1], bc);
    VM3_SubtractO(triangle[0], triangle[2], ca);    

    f32 d1, d2, d3;
    d1 = iVVOX_Sign(ray_position, triangle[0], triangle[1]);
    d2 = iVVOX_Sign(ray_position, triangle[1], triangle[2]);
    d3 = iVVOX_Sign(ray_position, triangle[2], triangle[0]);

    
    b8 has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    b8 has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    
    if (!(has_neg || has_pos)) return (FaceProbe){0};

    f32 weight[3];
    VM3_Set(weight, 1, 1, 1);

    f32 depth = triangle[0][2]*weight[0] + triangle[1][2]*weight[1] + triangle[2][2]*weight[2];
    depth /= 3;
    
    u32 winding_order = has_neg ? FACE_CCW : 0;
    
    return (FaceProbe) {
	.depth = depth,
	.flags = FACE_FOUND | winding_order
    };
}

b8* VVOX_VerticesToVoxels(void* vertices, u32 vcount, u32 vstride, u32 voffset,
			   u32* indices,  u32 icount,
			   f32 voxel_size, u32* out) {
    VShape box = VSHAPE_BoxFromVertices(vertices, vcount, vstride, voffset);

    u32 voxel_count[3];
    voxel_count[0] = ceilf(box.size[0]/voxel_size);
    voxel_count[1] = ceilf(box.size[1]/voxel_size);    
    voxel_count[2] = ceilf(box.size[2]/voxel_size);
    out[0] = voxel_count[0];
    out[1] = voxel_count[1];
    out[2] = voxel_count[2];
    f32 box_min[3]; f32 halfsize[3];
    VM3_ScaleO(box.size, 0.5, halfsize);
    VM3_SubtractO(box.center, halfsize, box_min);
    
    b8 *voxels = calloc(voxel_count[0]*voxel_count[1]*voxel_count[2], sizeof(b8));
    for (u32 i = 0; i < voxel_count[0]; i++) {
	for (u32 j = 0; j < voxel_count[1]; j++) {
	    FaceProbes probes;
	    iVVOX_FaceProbesInit(&probes, 2);
	    f32 ray_position[3];
	    VM3_Set(ray_position,
		    i*voxel_size + box_min[0] + voxel_size/2,
		    j*voxel_size + box_min[1] + voxel_size/2,
		    0);
		
	    
	    for (u32 face = 0; face < icount/3; face++) {
		f32 triangle[3][3];
		f32* vertex_a; f32* vertex_b; f32* vertex_c;
		vertex_a = (void*)((u8*)vertices + indices[face*3+0]*vstride + voffset);
		vertex_b = (void*)((u8*)vertices + indices[face*3+1]*vstride + voffset);
		vertex_c = (void*)((u8*)vertices + indices[face*3+2]*vstride + voffset);
		VM3_Copy(triangle[0], vertex_a);
		VM3_Copy(triangle[1], vertex_b);
		VM3_Copy(triangle[2], vertex_c);
		FaceProbe probe = iVVOX_TriangleFaceProbeGet(triangle, ray_position, voxel_size);
		if (!(probe.flags & FACE_FOUND)) continue;
		iVVOX_FaceProbesPush(&probes, probe);
	    }

//	    printf("probes.count = %d\n", probes.count);
	    if (probes.count < 2) {
		iVVOX_FaceProbesDestroy(&probes);
		continue;
	    }
	    iVVOX_FaceProbesSort(&probes);

	    const int DEPTH_MIN = box_min[2];
	    const int DEPTH_MAX = box_min[2] + voxel_count[2];

	    FaceProbe* first_probe = iVVOX_FaceProbesPointerGet(&probes, 0);
	    u32 is_ccw = first_probe->flags & FACE_CCW;
	    f32 prev_depth = first_probe->depth;
	    b8 is_inside = 1;
	    for (u32 p = 1; p < probes.count; p++) {
		FaceProbe* probe = iVVOX_FaceProbesPointerGet(&probes, p);
		if ((probe->flags & FACE_CCW) != is_ccw) {
		    if (is_inside) {
			u32 depth_from = iVVOX_DepthToVoxel(prev_depth, DEPTH_MIN, DEPTH_MAX, voxel_size);
			u32 depth_to = iVVOX_DepthToVoxel(probe->depth, DEPTH_MIN, DEPTH_MAX, voxel_size);
			for (u32 d = depth_from;
			     d <= depth_to; d++)
			{
			    voxels[d*voxel_count[1]*voxel_count[0] + j*voxel_count[0] + i] = true;
			}
//			printf("filled from %d to %d\n", depth_from, depth_to);
		    }
		    prev_depth = probe->depth;
		    is_inside = !is_inside;
		    is_ccw = probe->flags & FACE_CCW;
		}
	    }
	    iVVOX_FaceProbesDestroy(&probes);
	}
    }

    return voxels;
}

b8 VVOX_VoxelGet(b8 *voxels, u32 voxel_count[3], u32 x, u32 y, u32 z) {
    return voxels[voxel_count[1]*voxel_count[0]*z + voxel_count[0]*y + x];
}

const u32 neighbours[6][3] = {
    { 0,  0, -1 },
    { 0,  0,  1 },
    { 0, -1,  0 },
    { 0,  1,  0 },
    {-1,  0,  0 },
    { 1,  0,  0 }
};

u8 VVOX_VoxelNeighbourCountGet(b8 *voxels, u32 voxel_count[3], u32 x, u32 y, u32 z) {
    u8 total_count = 0; u32 nx, ny, nz;
    for (u32 i = 0; i < 6; i++) {
	nx = x + neighbours[i][0];
	ny = y + neighbours[i][1];
	nz = z + neighbours[i][2];
	if (nx >= voxel_count[0] || nx < 0 ||
	    ny >= voxel_count[1] || ny < 0 ||
	    nz >= voxel_count[2] || nz < 0
	    ) continue;

	total_count += VVOX_VoxelGet(voxels, voxel_count, nx, ny, nz);
    }
    return total_count;
}
