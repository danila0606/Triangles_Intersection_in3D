#include "Octotree.h"

std::vector<geo::Rectangle> Node::SplitRect() const noexcept {

    const geo::Vector c = (rect_.v_min_ + rect_.v_max_) / 2.0;

    const double dx = rect_.v_max_.x_ - rect_.v_min_.x_;
    const double dy = rect_.v_max_.y_ - rect_.v_min_.y_;
    const double dz = rect_.v_max_.z_ - rect_.v_min_.z_;

    std::vector<geo::Rectangle> rects(8);

    rects[0] = geo::Rectangle(rect_.v_min_, c);
    rects[1] = geo::Rectangle(rect_.v_min_ + geo::Vector(dx/2, 0, 0), c + geo::Vector(dx/2, 0, 0));
    rects[2] = geo::Rectangle(rect_.v_min_ + geo::Vector(dx/2, dy/2, 0), c + geo::Vector(dx/2, dy/2, 0));
    rects[3] = geo::Rectangle(rect_.v_min_ + geo::Vector(0, dy/2, 0), c + geo::Vector(0, dy/2, 0));
    rects[4] = geo::Rectangle(rect_.v_min_ + geo::Vector(0,0,dz/2), c + geo::Vector(0,0,dz/2));
    rects[5] = geo::Rectangle(rect_.v_min_ + geo::Vector(dx/2, 0, dz/2), c + geo::Vector(dx/2, 0, dz/2));
    rects[6] = geo::Rectangle(rect_.v_min_ + geo::Vector(dx/2, dy/2, dz/2), c + geo::Vector(dx/2, dy/2, dz/2));
    rects[7] = geo::Rectangle(rect_.v_min_ + geo::Vector(0, dy/2, dz/2), c + geo::Vector(0, dy/2, dz/2));

    return rects;
}


std::set<size_t> Node::GetIntersecTrNumbers() {

    if (s_.GetNumOfTr() <= 4) {

        return s_.GetIntersectTriangles();
    }

    const std::vector<geo::Rectangle> Child_Rects = SplitRect();
    std::vector<geo::Triangle> trs = s_.GetTriangles();

    std::vector<std::vector<geo::Triangle>> trs_in(8);

    std::vector<geo::Triangle> r_bad;
    size_t r_bad_count = 0; //count of bad triangles, which intersect 0,1...6,7 rects


    for(const auto& i : trs) {
        for (int a = 0; a < 8; a++) {

            if (Space::IsInclude(Child_Rects[a], i.limit_rect)) {
                trs_in[a].push_back(i);
                break;
            }
            else if (Space::CheckCollision(Child_Rects[a], i)) {
                r_bad.push_back(i);
                break;
            }
        }

    }

    r_bad_count = r_bad.size();

    if (r_bad_count > s_.GetNumOfTr()/2)
        return s_.GetIntersectTriangles();

    std::set<size_t> ans; //returnable answer

    for (int i = 0; i < 8; i++) {
        if (!trs_in[i].empty()) {
            Node *n = new Node(trs_in[i], Child_Rects[i]);
            children_[i] = n;

            for (const auto& a : r_bad) {
                for (const auto& b : trs_in[i]) {
                    if (Space::CheckCollision(a, b)) {
                        ans.insert(a.number_);
                        ans.insert(b.number_);
                    }
                }
            }

        }

    }


    for (auto & i : children_) {
        if (i != nullptr) {
            std::set<size_t> ans_i = i->GetIntersecTrNumbers();
            ans.insert(ans_i.begin(), ans_i.end());
        }
    }

    for (int i = 0; i < r_bad_count; i++ ) {

        for (int t = i + 1; t < r_bad_count; t++) {

            if (Space::CheckCollision(r_bad[i], r_bad[t])) {
                ans.insert(r_bad[i].number_);
                ans.insert(r_bad[t].number_);
            }

        }

    }

    return ans;
}


geo::Rectangle Octotree::FindBoundingRect(const std::vector<geo::Triangle> &triangles) noexcept {

    double x_min = 0.0, y_min = 0.0, z_min = 0.0;
    double x_max = 0.0, y_max = 0.0, z_max = 0.0;

    for (const auto& i : triangles) {
        geo::Rectangle r = i.limit_rect;

        if (r.v_min_.x_ < x_min)
            x_min = r.v_min_.x_;
        if (r.v_min_.y_ < y_min)
            y_min = r.v_min_.y_;
        if (r.v_min_.z_ < z_min)
            z_min = r.v_min_.z_;
        if (r.v_max_.x_ > x_max)
            x_max = r.v_max_.x_;
        if (r.v_max_.y_ > y_max)
            y_max = r.v_max_.y_;
        if (r.v_max_.z_ > z_max)
            z_max = r.v_max_.z_;
    }



    return {geo::Vector(x_min,y_min, z_min), geo::Vector(x_max, y_max, z_max)};
}

void Octotree::PrintIntersecTriangles() {

    std::set<size_t> intersec_list = main_->GetIntersecTrNumbers();

    for (const auto& i : intersec_list)
        std::cout<<i<<" ";

    std::cout<<'\n';
}

std::set<size_t> Octotree::GetIntersecTriangles() {

    auto intersec_list = main_->GetIntersecTrNumbers();

    return intersec_list;
}

