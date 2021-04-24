#pragma once

#include "Vulkan/driver.h"
#include "Vulkan/Info.h"
#include "Intersection_Geometry/Octotree.h"

class Loader final {
public:
    Loader(std::vector<geo::R_Triangle> trs, double time = 0.0);

    void Start();

private:

    std::pair<std::vector<Vertex>, std::vector<uint16_t>>
    TransformTrianglesToVertexesAndIndices(const std::set<size_t>& nums) const;

    cam::CameraInfo MakeCamInfo(const Octotree& tree) const;
    std::vector<geo::Triangle> GetTriangles() const;

    std::vector<geo::R_Triangle> rtriangles_;
    const double lifetime;
};

