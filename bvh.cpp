// otherwise static keywords in STL libraries lead to errors
#ifndef COW_NO_STYLE_GUIDE
#define COW_NO_STYLE_GUIDE
#define SNAIL_I_SOLEMNLY_SWEAR_I_AM_UP_TO_NO_GOOD
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

struct CastRayResult {
    bool hit_at_least_one_triangle;
    double min_t;       // the minimum distance to hit
    vec3 base_color;    // color of the hit point
    vec3 N_hit;         // normal vector of point of hit
    vec3 p_hit;         // point of hit
};

struct Ray {
    vec3 o;     // origin
    vec3 dir;   // direction
};

bool intersection(const AABB3D& box, const Ray& ray) {
    // Calculate the intersection intervals for each axis
    double tmin = (box.min.x - ray.o.x) / ray.dir.x;
    double tmax = (box.max.x - ray.o.x) / ray.dir.x;
    if (tmin > tmax) std::swap(tmin, tmax);

    double tymin = (box.min.y - ray.o.y) / ray.dir.y;
    double tymax = (box.max.y - ray.o.y) / ray.dir.y;
    if (tymin > tymax) std::swap(tymin, tymax);

    double tzmin = (box.min.z - ray.o.z) / ray.dir.z;
    double tzmax = (box.max.z - ray.o.z) / ray.dir.z;
    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    // Check for intersection
    double t_enter = std::max(tmin, std::max(tymin, tzmin));
    double t_exit = std::min(tmax, std::min(tymax, tzmax));
    return t_enter <= t_exit;
}

// intersect a ray with a bounding box AABB3D



CastRayResult cast_ray(const Ray &ray, bvh_node *node, const IndexedTriangleMesh3D &mesh) {
    // if the ray does not intersect the bounding box, do not bother going down
    if (!intersection(node->bbox, ray)) {
        return {false, INFINITY, {}, {}, {}};
    }

    if (node->left == nullptr && node->right == nullptr) {
        // leaf node, check all triangles inside
        bool hit_at_least_one_triangle = false;
        double min_t = INFINITY;
        vec3 pixel_color = {};
        vec3 N_hit = {};
        vec3 p_hit = {};

        for (size_t i = 0; i < node->triangle_indices.size(); i++) {
            auto &tri = mesh.triangle_indices[node->triangle_indices[i]];

            vec3 a = mesh.vertex_positions[tri[0]];
            vec3 b = mesh.vertex_positions[tri[1]];
            vec3 c = mesh.vertex_positions[tri[2]];

            vec3 color_a, color_b, color_c;
            vec3 n;
            {
                vec3 e1 = b - a;
                vec3 e2 = c - a;
                n = normalized(cross(e1, e2));
                if (mesh.vertex_colors != NULL) {
                    color_a = mesh.vertex_colors[tri[0]];
                    color_b = mesh.vertex_colors[tri[1]];
                    color_c = mesh.vertex_colors[tri[2]];
                } else {
                    color_a = V3(.5) + .5 * mesh.vertex_normals[tri[0]];
                    color_b = V3(.5) + .5 * mesh.vertex_normals[tri[1]];
                    color_c = V3(.5) + .5 * mesh.vertex_normals[tri[2]];
                }
            }

            mat4 A = M4(
                a.x, b.x, c.x, ray.dir.x,
                a.y, b.y, c.y, ray.dir.y,
                a.z, b.z, c.z, ray.dir.z,
                1, 1, 1, 0
            );
            vec4 z = V4(ray.o.x, ray.o.y, ray.o.z, 1);
            // barycentric coords,  (-t)
            auto ans = inverse(A) * z;

            const double thresh = -0.0001;  // fix the problem of triangle intersects
            bool hit = ans.x > thresh && ans.y > thresh && ans.z > thresh;
            double t = -ans.w;

            if (hit && t > 0) {
                hit_at_least_one_triangle = true;
                if (t < min_t) {
                    // this is correct interpolation ==> barycentric coord of actual triangle (not projection of triangle on screen)
                    pixel_color = (
                        ans.x * color_a +
                        ans.y * color_b +
                        ans.z * color_c
                    );
                    min_t = t;
                    N_hit = n;
                    p_hit = ray.o + min_t * ray.dir;
                }
            }
        }

        return {hit_at_least_one_triangle, min_t, pixel_color, N_hit, p_hit};
    } else {
        // internal node, recurse down both children
        auto hit_left = cast_ray(ray, node->left, mesh);
        auto hit_right = cast_ray(ray, node->right, mesh);

        if (hit_left.hit_at_least_one_triangle && hit_right.hit_at_least_one_triangle) {
            if (hit_left.min_t < hit_right.min_t) {
                return hit_left;
            } else {
                return hit_right;
            }
        } else if (hit_left.hit_at_least_one_triangle) {
            return hit_left;
        } else if (hit_right.hit_at_least_one_triangle) {
            return hit_right;
        } else {
            return {false, INFINITY, {}, {}, {}};
        }
    }
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

        if ((node->left == nullptr && node->right == nullptr) || node->depth >= 7) {
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

    Ray ray = {{1,1,5}, {-1./5,-1./4,-1}};
    ray.dir = normalized(ray.dir);

    // std::cout << "ray: " << ray.o << " " << ray.dir << std::endl;
    std::cout << intersection(bvh_root->bbox, ray) << std::endl;

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


        // render the ray
        {
            vec3 p = ray.o;
            vec3 q = ray.o + 10 * ray.dir;
            eso_begin(P * V * M, SOUP_LINES, 4.0);
            eso_color(monokai.green);
            eso_vertex(p);
            eso_vertex(q);
            eso_end();
        }

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

int S = 256;

Texture color_buffer;
void hw8a_set_pixel(int i, int j, vec3 color) {
    color_buffer.data[4 * (j * S + i) + 0] = (unsigned char)(255 * CLAMP(color.r, 0, 1));
    color_buffer.data[4 * (j * S + i) + 1] = (unsigned char)(255 * CLAMP(color.g, 0, 1));
    color_buffer.data[4 * (j * S + i) + 2] = (unsigned char)(255 * CLAMP(color.b, 0, 1));
    color_buffer.data[4 * (j * S + i) + 3] = 255;
}

struct {
    bool draw_rays;
    bool draw_shadow_rays;
    bool draw_scene_3D = true;
    bool fully_transparent_film_plane_bg;
    bool draw_film_plane = true;
    bool draw_cube_at_observer;
    bool bunny_stress_test;
    double renderer_distance_to_film_plane = 1; // for visualization only (doesn't impact rendering)
} hw8a_tweaks;

void hw8a_draw_textured_square(mat4 P, mat4 V, mat4 M, char *texture_filename) {
    // please ignore; this function is hack-a-saurus rex
    static IndexedTriangleMesh3D square;
    if (!square.num_vertices) {
        square = library.meshes.square;
        square.vertex_normals = NULL;
    }
    square.draw(P, V, M, {}, texture_filename);
};


void ray_tracing_app() {
    // mesh    -- the current scene
    // light_p -- the position of the point light in world coordinates
    // teapot has 6,320 triangles.
    // IndexedTriangleMesh3D mesh = library.meshes.teapot;
    IndexedTriangleMesh3D mesh = library.meshes.bunny;
    vec3 light_p = V3(0, 2.5, 0);

    // build BVH
    std::vector<int> triangle_indices(mesh.num_triangles);
    std::iota(std::begin(triangle_indices), std::end(triangle_indices), 0);
    auto bvh_root = build_bvh(mesh, triangle_indices);

    // render
    // Camera3D camera = { 8.0, RAD(0.0) };
    Camera3D renderer = { 4, RAD(60) };
    Camera3D observer = { 6.5, RAD(60), RAD(30), RAD(-15), -2 };

    { // (fine to ignore)
        color_buffer= { "color_buffer", S, S, 4, (unsigned char *) malloc(S * S * 4) };
        _mesh_texture_create(color_buffer.name, color_buffer.width, color_buffer.height, color_buffer.number_of_channels, color_buffer.data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }


    while (cow_begin_frame()) {
        { // tweaks (fine to ignore)
            gui_checkbox("draw_rays", &hw8a_tweaks.draw_rays, 'z');
            gui_checkbox("draw_shadow_rays", &hw8a_tweaks.draw_shadow_rays, 'a');
            gui_checkbox("draw_film_plane", &hw8a_tweaks.draw_film_plane, 'x');
            gui_checkbox("draw_scene_3D", &hw8a_tweaks.draw_scene_3D, 's');
            gui_checkbox("draw_cube_at_observer", &hw8a_tweaks.draw_cube_at_observer, 'c');
            gui_checkbox("fully_transparent_film_plane_bg", &hw8a_tweaks.fully_transparent_film_plane_bg, 't');
            gui_checkbox("bunny_stress_test", &hw8a_tweaks.bunny_stress_test);

            gui_slider("hw8a_tweaks.renderer_distance_to_film_plane", &hw8a_tweaks.renderer_distance_to_film_plane, 0, 5);
            gui_slider("renderer.distance_to_origin", &renderer.persp_distance_to_origin, 1, 10);
            gui_slider("renderer.theta", &renderer.theta, RAD(-30), RAD(30), true);
            gui_slider("renderer.phi", &renderer.phi, RAD(-30), RAD(30), true);
            gui_slider("renderer.angle_of_view", &renderer.angle_of_view, RAD(1), RAD(178), true);
        }

        // *_renderer                   -- matrices for the renderer
        // *_observere                  -- matrices for the observer
        // film_plane_side_length_world -- the length of the film plane in world coordinates
        mat4 C_renderer, P_renderer, V_renderer;
        mat4 P_observer, V_observer, PV_observer;
        { // C_*, P_*, PV_*
            { // reset
                static Camera3D _renderer_0 = renderer;
                if (gui_button("reset C_renderer", 'r')) {
                    renderer = _renderer_0;
                }
            }
            camera_move(&observer);
            C_renderer = camera_get_C(&renderer);
            P_renderer = _window_get_P_perspective(renderer.angle_of_view); // aspect <- 1
            V_renderer = inverse(C_renderer);
            { // C_observer <- C_renderer
                static bool clicked;
                bool clicked_this_frame;
                bool selected;
                bool released_this_frame = false;
                {
                    char *name = "hold to toggle C_observer <- C_renderer";
                    clicked_this_frame = gui_button(name, COW_KEY_TAB);
                    if (clicked_this_frame) {
                        clicked = true;
                    }
                    // imgui.selected_widget_ID == COW1._gui_selected ??
                    selected = (COW1._gui_selected == (void *) name);
                    if (clicked && !selected) {
                        clicked = false;
                        released_this_frame = true;
                    }
                }

                { // memcpy's
                    static Camera3D safe;
                    static bool draw_cube_push;
                    if (clicked_this_frame) {
                        memcpy(&safe, &observer, sizeof(Camera3D));
                        draw_cube_push = hw8a_tweaks.draw_cube_at_observer;
                        hw8a_tweaks.draw_cube_at_observer = false;
                    }
                    if (selected) {
                        memcpy(&observer, &renderer, sizeof(Camera3D));
                    }
                    if (released_this_frame) {
                        memcpy(&observer, &safe, sizeof(Camera3D));
                        hw8a_tweaks.draw_cube_at_observer = draw_cube_push;
                    }
                }
            }
            P_observer = camera_get_P(&observer);
            V_observer = camera_get_V(&observer);
            PV_observer = P_observer * V_observer;
        }


        { // render (your work here!)
            { // prep (fine to ignore)
                { // clear color_buffer
                    if (hw8a_tweaks.fully_transparent_film_plane_bg) {
                        memset(color_buffer.data, 0, S * S * 4);
                    } else {
                        memset(color_buffer.data, 255, S * S * 4);
                        int pixelIndex = 0;
                        for (int j = 0; j < S; ++j) {
                            for (int i = 0; i < S; ++i) {
                                color_buffer.data[4 * pixelIndex++ + 3] = (((j + i) % 2) == 0) ? 200 : 220;
                            }
                        }
                    }
                }

                { // light_p
                    // TODO: remove the code that allows movement of the light
                    // jank_widget_translate3D(PV_observer, 1, &light_p);
                    soup_draw(PV_observer, SOUP_POINTS, 1, &light_p, NULL, monokai.white);
                }
            }

            { // render (your work here!)
                // renderer.angle_of_view -- _full_ (not half) angle of view of the renderer
                // *_renderer             -- the axes and origin of the renderer
                // NOTE: o_renderer is where rays originate from
                // NOTE: -z_renderer points from o_renderer to the center of the film plane
                vec3 x_renderer, y_renderer, z_renderer, o_renderer;
                {
                    auto system = camera_get_coordinate_system(&renderer);
                    x_renderer = system.x;
                    y_renderer = system.y;
                    z_renderer = system.z;
                    o_renderer = system.o;
                }

                { // write to color_buffer.data (your work here!)

                    // gl_* will be useful for debugging direction (dir)
                    eso_begin(PV_observer, SOUP_LINES); {


                        for (int i = 0; i < S; ++i) {
                            for (int j = 0; j < S; ++j) {

                                double theta = renderer.angle_of_view;
                                vec3 o = o_renderer;
                                // vec3 dir = V3(i - double(S) / 2, j - double(S) / 2, -(double(S) / 2) / tan(theta / 2)); // student answer from board
                                vec3 dir_tmp = V3(i - double(S) / 2, j - double(S) / 2, -(double(S) / 2) / tan(theta / 2)); // student answer from board
                                vec3 dir = dir_tmp.x * x_renderer + dir_tmp.y * y_renderer + dir_tmp.z * z_renderer;

                                if (hw8a_tweaks.draw_shadow_rays) {
                                    eso_color(monokai.green);
                                    eso_vertex(o);
                                    eso_vertex(o + 0.5 * dir);
                                }


                                // cast original ray
                                // TODO: multiple ray by M^-1
                                CastRayResult original = cast_ray({o, dir}, bvh_root, mesh);

                                if (original.hit_at_least_one_triangle) {
/*
                                    vec3 L = light_p - original.p_hit; // light direction
                                    vec3 N = original.N_hit;  // triangle normal at p_hit
                                    bool triangle_is_facing_the_light = dot(N, L) > 0;

                                    // cast shadow ray
                                    // Note that o shifted a little towards L to prevent precision-induced intersection with self
                                    CastRayResult shadow = cast_ray({original.p_hit + 0.0001 * L, L}, bvh_root, mesh);
                                    bool shadow_ray_did_not_hit_anything = !shadow.hit_at_least_one_triangle;

                                    // Blinn-Phong lighting
                                    double ambient_strength = 0.1;
                                    double diffuse_strength = 1.;
                                    double specular_strength = .5;
                                    double shininess = 32.0;

                                    bool attenuation = true;
                                    double attn_linear = 0.025;
                                    double attn_quadratic = 0.0;  // only linear to prevent high attenuation

                                    vec3 lightDir   = normalized(light_p - original.p_hit);
                                    vec3 viewDir    = normalized(o_renderer - light_p);

                                    // creative coding: inverse square law to control attenuation based on distance
                                    double luminosity = 1;
                                    double distance = norm(light_p - original.p_hit);
                                    if (attenuation) luminosity = 1./(1 + attn_linear * distance + attn_quadratic * distance*distance);

                                    // diffuse light
                                    float diff = MAX(dot(original.N_hit, lightDir), 0.0);
                                    vec3 diffuse = diffuse_strength * diff * original.base_color;

                                    // specular light (Blinn-Phong)
                                    vec3 halfwayDir = normalized(lightDir + viewDir);
                                    float spec = pow(MAX(dot(original.N_hit, halfwayDir), 0.0), shininess);
                                    vec3 specular = specular_strength * original.base_color * spec;

                                    vec3 lighting_color = V3(ambient_strength, ambient_strength, ambient_strength) + luminosity * (diffuse + specular);

                                    if (triangle_is_facing_the_light && shadow_ray_did_not_hit_anything) {
                                        hw8a_set_pixel(i, j, lighting_color);
                                        // hw8a_set_pixel(i, j, original.base_color);
                                    } else {
                                        hw8a_set_pixel(i, j, 0.5 * lighting_color);
                                        // hw8a_set_pixel(i, j, 0.5 * original.base_color);
                                        // draw shadow cast rays
                                        if (hw8a_tweaks.draw_shadow_rays) {
                                            // gl_color(original.base_color);
                                            eso_color(shadow.base_color);
                                            eso_vertex(original.p_hit);
                                            eso_vertex(original.p_hit + shadow.min_t * L);
                                        }
                                    }

*/
                                    hw8a_set_pixel(i, j, original.base_color);

                                    // draw original cast rays
                                    if (hw8a_tweaks.draw_rays) {
                                        eso_color(original.base_color);
                                        eso_vertex(o);
                                        eso_vertex(o + original.min_t * dir);
                                    }
                                }



                            }
                        }

                    } eso_end();
                }

                { // send updated texture to the GPU (fine to ignore)
                    _mesh_texture_sync_to_GPU(color_buffer.name, color_buffer.width, color_buffer.height, color_buffer.number_of_channels, color_buffer.data);
                }
            }
        }

        { // observe (fine to ignore)
            if (hw8a_tweaks.draw_scene_3D) {
                // soup_draw(PV_observer, SOUP_TRIANGLES, *mesh, V3(1, 0, 1));
                mesh.draw(P_observer, V_observer, M4_Identity());
            }
            { // bespoke widget
                library.soups.axes.draw(PV_observer * C_renderer);

                if (hw8a_tweaks.draw_film_plane) {
                    double film_plane_side_length_world = 2 * hw8a_tweaks.renderer_distance_to_film_plane * tan(renderer.angle_of_view / 2);
                    mat4 M = C_renderer * M4_Translation(0, 0, -hw8a_tweaks.renderer_distance_to_film_plane) * M4_Scaling(film_plane_side_length_world / 2);
                    hw8a_draw_textured_square(P_observer, V_observer, M, color_buffer.name);
                    { // outline
                        vec3 tmp[] = { { -1, -1, 0 }, { -1,  1, 0 }, { 1,  1, 0 }, {  1, -1, 0 }, };
                        // TODO: the original provides no color
                        soup_draw(P_observer * V_observer * M, SOUP_LINE_LOOP, 4, tmp, NULL, monokai.white);
                    }
                }
                if (hw8a_tweaks.draw_cube_at_observer) {
                    library.soups.box.draw(P_observer * C_renderer * M4_Scaling(.2), .5 * monokai.gray);
                }
            }
        }
    }
}

int main() {
    APPS {
        APP(ray_tracing_app);
        APP(bvh_app);
    }
    return 0;
}


#undef BVH_THRESHOLD
#undef REAL_MAX
#undef REAL_MIN
