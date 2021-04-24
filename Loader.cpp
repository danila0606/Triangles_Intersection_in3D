#include "Loader.h"

Loader::Loader(std::vector<geo::R_Triangle> trs, double time) :
    rtriangles_(std::move(trs)), lifetime(time) {}

std::pair<std::vector<Vertex>, std::vector<uint16_t>>
Loader::TransformTrianglesToVertexesAndIndices
        (const std::set<size_t>& nums) const{

    std::vector<Vertex> vertices;
    std::vector<uint16_t> indices;

    vertices.reserve(rtriangles_.size() * 3);
    indices.reserve(rtriangles_.size() * 6);

    for(const auto& elem : rtriangles_) {

        glm::vec3 color;
        if(nums.count(elem.tr_.number_))
            color = {1.0f, 0.0f, 0.0f};
        else
            color = {0.0f, 0.0f, 1.0f};


        const geo::Vector K01 = elem.tr_.D1_ - elem.tr_.D0_;
        const geo::Vector K02 = elem.tr_.D2_ - elem.tr_.D0_;

        geo::Vector N = K01^K02;
        geo::Normalization(N);
        glm::vec3 normal(N.x_, N.y_, N.z_);

        auto p = elem.tr_.D0_;
        glm::vec3 pos(p.x_, p.y_, p.z_);
        vertices.push_back({pos, color, normal});


        p = elem.tr_.D1_;
        pos = {p.x_, p.y_, p.z_};
        vertices.push_back({pos, color, normal});

        p = elem.tr_.D2_;
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

cam::CameraInfo Loader::MakeCamInfo(const Octotree& tree) const {
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

void Loader::Start() {

    Octotree tree1(GetTriangles());
    auto intersec_nums1 = tree1.GetIntersecTriangles();
    auto vert_and_ind1 = TransformTrianglesToVertexesAndIndices(intersec_nums1);

    auto cam_info = MakeCamInfo(tree1);
    cam::Camera cam(cam_info);
    TrianglesDrawer app(vert_and_ind1.first, vert_and_ind1.second, cam);
    app.run();

    auto startTime = std::chrono::high_resolution_clock::now();
    double begin = 0.0;

    while (!app.IsClosed()) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

        startTime = currentTime;
        begin += time;
        if (begin < lifetime) {

            for (auto &elem: rtriangles_)
                elem.Rotate(time);

            Octotree tree(GetTriangles());
            auto intersec_nums = tree.GetIntersecTriangles();
            auto vert_and_ind = TransformTrianglesToVertexesAndIndices(intersec_nums);

            app.UpdateBuffer(vert_and_ind.first, vert_and_ind.second);
        }

        app.mainLoop(time);
    }

    app.cleanup();
}

std::vector<geo::Triangle> Loader::GetTriangles() const {

    std::vector<geo::Triangle> trs;
    trs.reserve(rtriangles_.size());
    for (const auto& elem : rtriangles_)
        trs.push_back({elem.tr_});

    return trs;
}
