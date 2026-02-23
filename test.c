#define _VMATH_IMPLEMENTATION_
#include "vvox.h"
#include "include/vmesh/vmesh.h"
#include <float.h>
#include <stddef.h>


int main() {
    Mesh* mesh = malloc(sizeof(Mesh));
    VMESH_LoadObj(mesh, "include/vmesh/bunny.obj");
    f32 voxel_size = 0.01;

    u32 voxel_count[3];
    b8* voxels = VVOX_VerticesToVoxels(mesh->vertices, mesh->vertex_count, sizeof(Vertex), offsetof(Vertex, pos),
			   mesh->indices,  mesh->index_count,
					voxel_size, voxel_count);

    const u32 z = voxel_count[2]/12;
    printf("max_depth: %d\n", voxel_count[2]);
    for (i32 j = voxel_count[1] - 1; j >= 0; j--) {
	for (u32 i = 0; i < voxel_count[0]; i++){
	    printf("%c", voxels[z*voxel_count[0]*voxel_count[1] + j*voxel_count[0]+i] ? 'c' : ' ');
	}
	printf("\n");
    }

    
    free(voxels);
    VMESH_Destroy(mesh);
    
    return 0;
}
