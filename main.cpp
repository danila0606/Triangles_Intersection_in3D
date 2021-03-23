#pragma once

#include <fstream>
#include "Vulkan/driver.h"
#include "Vulkan/Info.h"
#include "Intersection_Geometry/Octotree.h"

std::vector<geo::Triangle> GetTrianglesFromFile(const std::string& filename);

std::pair<std::vector<Vertex>, std::vector<uint16_t>>
    TransformTrianglesToVertexesAndIndices
        (const std::vector<geo::Triangle>& triangles, const std::set<size_t>& nums);

cam::CameraInfo MakeCamInfo(const Octotree& tree);


int main() {

    try {
        auto triangles = GetTrianglesFromFile("tests/test10000.txt");

        Octotree tree(triangles);
        auto intersec_nums = tree.GetIntersecTriangles();
        auto vert_and_ind = TransformTrianglesToVertexesAndIndices(triangles, intersec_nums);

        auto cam_info = MakeCamInfo(tree);
        cam::Camera cam(cam_info);
        TrianglesDrawer app(vert_and_ind.first, vert_and_ind.second, cam);

        app.run();

    } catch (std::exception& e) {
        std::cerr<< e.what()<< std::endl;
        exit(-1);
    }

    return 0;
}

cam::CameraInfo MakeCamInfo(const Octotree& tree) {
    auto v_max = tree.GetMaxCorner();
    auto v_min = tree.GetMinCorner();

    glm::vec3 position = glm::vec3{v_max.x_, v_max.y_, v_max.z_};
    glm::vec3 center = glm::vec3{(v_min.x_ + v_max.x_) / 2, (v_min.y_ + v_max.y_) / 2, (v_min.z_ + v_max.z_) / 2};
    cam::CameraInfo info{};

    info.position_ = position;
    info.center_ = center;
    info.view_angle_ = CAM_VIEW_ANGLE;
    info.aspect_ = CAM_ASPECT;
    info.near_ = CAM_NEAR;
    info.far_ = CAM_FAR;
    info.speed_ = CAM_SPEED;
    info.rotate_speed_ = CAM_ROTATE_SPEED;

    info.up_vector_ = glm::vec3(0.f, 0.f, 1.f);

    return info;
}

std::vector<geo::Triangle> GetTrianglesFromFile(const std::string& filename) {

    std::ifstream str(filename);
    if (!str.is_open()) {
        std::cout << "Couldn't open file." << '\n';
        exit(-1);
    }

    int N = 0;
    str>>N;
    std::vector<geo::Triangle> triangles;
    triangles.reserve(N);

    for (int i = 0; i < N ; i++) {
        double x, y, z;

        std::vector<geo::Vector> t;

        str >> x >> y >> z;
        t.emplace_back(x,y,z);
        str >> x >> y >> z;
        t.emplace_back(x,y,z);
        str>> x >> y >> z;
        t.emplace_back(x,y,z);

        std::sort(t.begin(), t.end(), [](const geo::Vector l, const geo::Vector r) {
            return l.Modul() < r.Modul();
        } );

        triangles.emplace_back(t[0], t[1], t[2], i);
    }

    return triangles;
}

std::pair<std::vector<Vertex>, std::vector<uint16_t>>
TransformTrianglesToVertexesAndIndices
        (const std::vector<geo::Triangle>& triangles, const std::set<size_t>& nums) {

    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;

    vertices.reserve(triangles.size() * 3);
    indices.reserve(triangles.size() * 6);

    for(const auto& elem : triangles) {

        glm::vec3 color;
        if(nums.count(elem.number_))
            color = {1.0f, 0.0f, 0.0f};
        else
            color = {0.0f, 0.0f, 1.0f};


        const geo::Vector K01 = elem.D1_ - elem.D0_;
        const geo::Vector K02 = elem.D2_ - elem.D0_;

        geo::Vector N = K01^K02;
        geo::Normalization(N);
        glm::vec3 normal(N.x_, N.y_, N.z_);

        auto p = elem.D0_;
        glm::vec3 pos(p.x_, p.y_, p.z_);
        vertices.push_back({pos, color, normal});

        p = elem.D1_;
        pos = {p.x_, p.y_, p.z_};
        vertices.push_back({pos, color, normal});

        p = elem.D2_;
        pos = {p.x_, p.y_, p.z_};
        vertices.push_back({pos, color, normal});

        indices.push_back(static_cast<uint16_t>(vertices.size()) - 3);
        indices.push_back(static_cast<uint16_t>(vertices.size()) - 2);
        indices.push_back(static_cast<uint16_t>(vertices.size()) - 1);

        indices.push_back(static_cast<uint16_t>(vertices.size()) - 3);
        indices.push_back(static_cast<uint16_t>(vertices.size()) - 1);
        indices.push_back(static_cast<uint16_t>(vertices.size()) - 2);
    }

    return {vertices, indices};
}
