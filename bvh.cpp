// otherwise static keywords in STL libraries lead to errors
#define COW_NO_STYLE_GUIDE

#include "include.cpp"

#include <vector>

// axis-aligned bounding box
struct AABB3D {
    vec3 min;
    vec3 max;

    int longest_axis() {
        vec3 size = max - min;
        if (size.x > size.y && size.x > size.z) return 0;
        if (size.y > size.x && size.y > size.z) return 1;
        if (size.z > size.x && size.z > size.y) return 2;
        return -1;
    }
};

#define REAL_MAX DBL_MAX
#define REAL_MIN DBL_MIN

void render_aabb3d(AABB3D aabb, mat4 PV, real size_in_pixels = 2., vec3 color = monokai.green, real alpha=1.) {
    eso_begin(PV, SOUP_LINES, size_in_pixels); {
        eso_color(color, alpha);

        auto min = aabb.min;
        auto max = aabb.max;

        eso_vertex(min.x, min.y, max.z);
        eso_vertex(max.x, min.y, max.z);
        eso_vertex(max.x, max.y, max.z);
        eso_vertex(min.x, max.y, max.z);
        eso_vertex(max.x, min.y, max.z);
        eso_vertex(max.x, min.y, min.z);
        eso_vertex(max.x, max.y, min.z);
        eso_vertex(max.x, max.y, max.z);
        eso_vertex(max.x, max.y, min.z);
        eso_vertex(min.x, max.y, min.z);
        eso_vertex(min.x, min.y, min.z);
        eso_vertex(min.x, max.y, min.z);
        eso_vertex(max.x, max.y, min.z);
        eso_vertex(max.x, min.y, min.z);
        eso_vertex(min.x, min.y, min.z);
        eso_vertex(max.x, min.y, min.z);
        eso_vertex(min.x, min.y, min.z);
        eso_vertex(min.x, min.y, max.z);
        eso_vertex(min.x, max.y, max.z);
        eso_vertex(min.x, max.y, min.z);
        eso_vertex(max.x, min.y, max.z);
        eso_vertex(max.x, max.y, max.z);
        eso_vertex(min.x, min.y, max.z);
        eso_vertex(min.x, max.y, max.z);
    } eso_end();
}

AABB3D mesh_bbox(IndexedTriangleMesh3D &mesh) {
    AABB3D aabb = { { REAL_MAX, REAL_MAX, REAL_MAX }, { REAL_MIN, REAL_MIN, REAL_MIN } };

    // assuming all vertices are used, there is no need to iterate over triangles.
    for (int i = 0; i < mesh.num_vertices; i++) {
        vec3 vert = mesh.vertex_positions[i];

        if (vert.x > aabb.max.x) aabb.max.x = vert.x;
        if (vert.x < aabb.min.x) aabb.min.x = vert.x;

        if (vert.y > aabb.max.y) aabb.max.y = vert.y;
        if (vert.y < aabb.min.y) aabb.min.y = vert.y;

        if (vert.z > aabb.max.z) aabb.max.z = vert.z;
        if (vert.z < aabb.min.z) aabb.min.z = vert.z;
    }

    return aabb;
}

AABB3D bbox_from_triangles(IndexedTriangleMesh3D &mesh, std::vector<int> &triangle) {
    AABB3D aabb = { { REAL_MAX, REAL_MAX, REAL_MAX }, { REAL_MIN, REAL_MIN, REAL_MIN } };

    for (int i = 0; i < triangle.size(); i++) {
        auto &tri = mesh.triangle_indices[triangle[i]];

        for (int j = 0; j < 3; j++) {
            vec3 vert = mesh.vertex_positions[tri[j]];

            if (vert.x > aabb.max.x) aabb.max.x = vert.x;
            if (vert.x < aabb.min.x) aabb.min.x = vert.x;

            if (vert.y > aabb.max.y) aabb.max.y = vert.y;
            if (vert.y < aabb.min.y) aabb.min.y = vert.y;

            if (vert.z > aabb.max.z) aabb.max.z = vert.z;
            if (vert.z < aabb.min.z) aabb.min.z = vert.z;
        }
    }

    return aabb;
}

// bounding volume hierarchy
void bvh_app()
{
    // cone has 126 triangles. shouldn't be too slow
    // IndexedTriangleMesh3D mesh = library.meshes.cone;
    IndexedTriangleMesh3D mesh = library.meshes.teapot;

    // computer aabb bounding box
    AABB3D aabb = mesh_bbox(mesh);

    int axis = aabb.longest_axis();
    std::vector<std::pair<int, real>> tri_axis_avg(mesh.num_triangles);

    // calculate the average position on the longest axis for each triangle
    for (int i = 0; i < mesh.num_triangles; i++) {
        auto &tri = mesh.triangle_indices[i];

        real a = mesh.vertex_positions[tri[0]][axis];
        real b = mesh.vertex_positions[tri[1]][axis];
        real c = mesh.vertex_positions[tri[2]][axis];

        real max = a;
        if (b > max) max = b;
        if (c > max) max = c;

        real min = a;
        if (b < min) min = b;
        if (c < min) min = c;

        tri_axis_avg[i] = { i, (max + min) / 2.0 };
    }

    // sort by second value (average position on the longest axis)
    // TODO: the two captures?? why copy not reference
    std::sort(tri_axis_avg.begin(), tri_axis_avg.end(), [=](std::pair<int, real>& a, std::pair<int, real>& b) {
        return a.second < b.second;
    });

    real mid = (aabb.max[axis] + aabb.min[axis]) / 2.0;

    // triangles with average smaller than mid
    std::vector<std::pair<int, real>> tri_axis_avg_small;
    std::copy_if(tri_axis_avg.begin(), tri_axis_avg.end(), std::back_inserter(tri_axis_avg_small), [&](std::pair<int, real>& a) {
        return a.second < mid;
    });

    // extract indices
    std::vector<int> tri_axis_avg_small_idx;
    std::transform(tri_axis_avg_small.begin(), tri_axis_avg_small.end(), std::back_inserter(tri_axis_avg_small_idx), [&](std::pair<int, real>& a) {
        return a.first;
    });

    // calculate aabb for these triangles
    AABB3D aabb_small = bbox_from_triangles(mesh, tri_axis_avg_small_idx);

    // render
    Camera3D camera = { 8.0, RAD(0.0) };

    while (cow_begin_frame()) {
        camera_move(&camera);
        camera_attach_to_gui(&camera);

        mat4 P = camera_get_P(&camera);
        mat4 V = camera_get_V(&camera);
        mat4 PV = P * V;
        mat4 M = M4_Identity();

        gui_printf("mid: %f %f %f", mid, aabb.min[axis], aabb.max[axis]);


        mesh.draw(P, V, M);

        // drawing the bounding box
        render_aabb3d(aabb, PV, 5.0);
        render_aabb3d(aabb_small, PV, 3.0, monokai.yellow);
    }
}

int main() {
    APPS {
        APP(bvh_app);
    }
    return 0;
}
