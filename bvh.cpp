// otherwise static keywords in STL libraries lead to errors
#ifndef COW_NO_STYLE_GUIDE
#define COW_NO_STYLE_GUIDE
#include "include.cpp"
#endif

#include <numeric> // std::iota
#include <vector>
#include <queue>

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

// if a node has less than this number of triangles, it is not split
#define BVH_THRESHOLD 1

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

void render_wireframe_indices(IndexedTriangleMesh3D &mesh, std::vector<int> &triangle_indices, mat4 P, mat4 V, mat4 M) {
    eso_begin(P * V * M, SOUP_LINES, 4.0);
    eso_color(monokai.red);
    for (size_t i = 0; i < triangle_indices.size(); ++i) {
        int idx = triangle_indices[i];
        for (int j = 0; j < 3; ++j) {
            eso_vertex(mesh.vertex_positions[mesh.triangle_indices[idx][j]]);
            eso_vertex(mesh.vertex_positions[mesh.triangle_indices[idx][(j + 1) % 3]]);
        }
    }
    eso_end();
}

// a bit faster than bbox_from_mesh, since it directly iterates over the vertices
// Assumption: all vertices are used in the mesh triangles
AABB3D bbox_from_mesh_fast(IndexedTriangleMesh3D &mesh) {
    AABB3D aabb = { { REAL_MAX, REAL_MAX, REAL_MAX }, { -REAL_MAX, -REAL_MAX, -REAL_MAX } };

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

AABB3D bbox_from_triangles(IndexedTriangleMesh3D &mesh, std::vector<int> &triangle_indices) {
    AABB3D aabb = { { REAL_MAX, REAL_MAX, REAL_MAX }, { -REAL_MAX, -REAL_MAX, -REAL_MAX } };

    for (size_t i = 0; i < triangle_indices.size(); i++) {
        auto &tri = mesh.triangle_indices[triangle_indices[i]];

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

AABB3D bbox_from_mesh(IndexedTriangleMesh3D &mesh) {
    std::vector<int> triangle_indices(mesh.num_triangles);
    std::iota(std::begin(triangle_indices), std::end(triangle_indices), 0);
    return bbox_from_triangles(mesh, triangle_indices);
}

struct bvh_node {
    AABB3D bbox;
    std::vector<int> triangle_indices;
    int depth;
    bvh_node *left;
    bvh_node *right;
};

bvh_node *build_bvh(IndexedTriangleMesh3D &mesh, std::vector<int> &triangle_indices, int depth = 0) {
    bvh_node *node = new bvh_node;

    // std::cout << triangle_indices.size() << std::endl;

    node->bbox = bbox_from_triangles(mesh, triangle_indices);
    node->triangle_indices = triangle_indices;
    node->depth = depth;
    node->left = nullptr;
    node->right = nullptr;

    if (triangle_indices.size() <= BVH_THRESHOLD) {
        return node;
    }

    // TODO: split using Surface Area Heuristic (SAH) to minimize the cost of tracing

    int axis = node->bbox.longest_axis();
    std::vector<std::pair<int, real>> tri_axis_med(triangle_indices.size());

    // calculate the median position on the longest axis for each triangle
    // if they are C++ std::vector's, we can use parallelized std::transform
    for (size_t i = 0; i < triangle_indices.size(); i++) {
        auto &tri = mesh.triangle_indices[triangle_indices[i]];

        real a = mesh.vertex_positions[tri[0]][axis];
        real b = mesh.vertex_positions[tri[1]][axis];
        real c = mesh.vertex_positions[tri[2]][axis];

        // real max = a;
        // if (b > max) max = b;
        // if (c > max) max = c;

        // real min = a;
        // if (b < min) min = b;
        // if (c < min) min = c;

        // tri_axis_med[i] = { i, (max + min) / 2.0 };
        tri_axis_med[i] = { i, (a + b + c) / 3.0};
    }

    // partition by average position on the longest axis
    real mid = (node->bbox.max[axis] + node->bbox.min[axis]) / 2.0;
    auto cutoff = std::stable_partition(tri_axis_med.begin(), tri_axis_med.end(), [mid](std::pair<int, real>& a) {
        return a.second <= mid;
    });

    // early stop if the partitioning is not effective
    if (cutoff == tri_axis_med.begin() || cutoff == tri_axis_med.end()) {
        // further splitting would only create nodes within this node's bounding box
        // so we should stop splitting
        return node;
    }

    // extract indices
    std::vector<int> tri_left_idx;
    std::vector<int> tri_right_idx;

    std::transform(tri_axis_med.begin(), cutoff, std::back_inserter(tri_left_idx), [&triangle_indices](std::pair<int, real>& a) {
        return triangle_indices[a.first];
    });
    std::transform(cutoff, tri_axis_med.end(), std::back_inserter(tri_right_idx), [&triangle_indices](std::pair<int, real>& a) {
        return triangle_indices[a.first];
    });

    // recursively build the left and right subtrees
    node->left = build_bvh(mesh, tri_left_idx, depth + 1);
    node->right = build_bvh(mesh, tri_right_idx, depth + 1);

    return node;
}


// bounding volume hierarchy
void bvh_app()
{
    // cone has 126 triangles. shouldn't be too slow
    // IndexedTriangleMesh3D mesh = library.meshes.cone;

    // teapot has 6,320 triangles.
    IndexedTriangleMesh3D mesh = library.meshes.teapot;

    // IndexedTriangleMesh3D mesh = library.meshes.bunny;

    std::vector<int> triangle_indices(mesh.num_triangles);
    std::iota(std::begin(triangle_indices), std::end(triangle_indices), 0);
    auto bvh_root = build_bvh(mesh, triangle_indices);

    // // get all bounding boxes at the leaf nodes of the BVH
    std::vector<AABB3D> bboxes;
    std::queue<bvh_node*> queue;
    queue.push(bvh_root);

    while (!queue.empty()) {
        bvh_node *node = queue.front();
        queue.pop();

        if ((node->left == nullptr && node->right == nullptr) || node->depth >= 8) {
            bboxes.push_back(node->bbox);
        }
        else {
            queue.push(node->left);
            queue.push(node->right);
        }
    }

    // std::cout << "num bboxes: " << bboxes.size() << std::endl;

    // render
    Camera3D camera = { 8.0, RAD(0.0) };

    while (cow_begin_frame()) {
        camera_move(&camera);
        camera_attach_to_gui(&camera);

        mat4 P = camera_get_P(&camera);
        mat4 V = camera_get_V(&camera);
        mat4 PV = P * V;
        mat4 M = M4_Identity();


        // drawing wireframe for all triangles
        eso_begin(P * V * M, SOUP_LINES, 4.0);
        eso_color(monokai.red);
        for (int i = 0; i < mesh.num_triangles; ++i) {
            for (int j = 0; j < 3; ++j) {
                eso_vertex(mesh.vertex_positions[mesh.triangle_indices[i][j]]);
                eso_vertex(mesh.vertex_positions[mesh.triangle_indices[i][(j + 1) % 3]]);
            }
        }
        eso_end();

        mesh.draw(P, V, M);

        // auto node = bvh_root->left->right->left;
        // AABB3D bbox = bbox_from_triangles(mesh, node->triangle_indices);
        // render_aabb3d(node->bbox, PV, 3.0, monokai.yellow);
        // render_wireframe_indices(mesh, node->triangle_indices, P, V, M);

        // drawing the bounding box
        for (auto &bbox : bboxes) {
            render_aabb3d(bbox, PV, 3.0, monokai.yellow);
        }
    }
}

int main() {
    APPS {
        APP(bvh_app);
    }
    return 0;
}


#undef BVH_THRESHOLD
#undef REAL_MAX
#undef REAL_MIN
