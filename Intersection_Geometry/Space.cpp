#include "Space.h"

//....................IsInclude()...................................

bool Space::IsInclude(const geo::Line& l, const geo::Vector& p) noexcept {

    return geo::AreParallel(l.A_, l.D_ - p);
}

bool Space::IsInclude(const geo::Cut& c, const geo::Vector& p) noexcept {

    return geo::AreCoDirected(p - c.A_, c.B_ - p);
}

bool Space::IsInclude(const geo::Plane& t,const geo::Vector& p) noexcept {

    return (t.D0_ * t.N_ - p * t.N_) < geo::eps;
}


bool Space::IsInclude(const geo::Triangle& t,const geo::Vector& p) {

    if (IsInclude(GetPlaneFromTr(t), p)) {
        if (IsInclude(geo::Cut(t.D0_, t.D1_), p) ||
            IsInclude(geo::Cut(t.D1_, t.D2_), p) ||
            IsInclude(geo::Cut(t.D2_, t.D0_), p)) {
            return true;
        }
        else {

            const double x1 = acos(COS_BETWEEN((t.D0_ - p), (t.D1_ - p)));
            const double x2 = acos(COS_BETWEEN((t.D1_ - p), (t.D2_ - p)));
            const double x3 = acos(COS_BETWEEN((t.D2_ - p), (t.D0_ - p)));

            return ((x1 + x2 + x3 > M_PI - 0.001) && (x1 + x2 + x3 < M_PI + 0.001));
        }
    }

    return false;
}

bool Space::IsInclude(const geo::Rectangle& l, const geo::Rectangle& r) noexcept {

    return IsInclude(l, r.v_min_) && IsInclude(l, r.v_max_);
}

bool Space::IsInclude(const geo::Rectangle& r, const geo::Vector& p) noexcept {

    if( p.x_ < (r.v_max_.x_ + geo::eps) && p.x_ > (r.v_min_.x_ - geo::eps)) {
        if( p.y_ < (r.v_max_.y_ + geo::eps) && p.y_ > (r.v_min_.y_ - geo::eps)) {
            if( p.z_ < (r.v_max_.z_ + geo::eps) && p.z_ > (r.v_min_.z_ - geo::eps)) {
                return true;
            }
        }
    }

    return false;
}

//...........................IsInclude()_end.................................

std::set<size_t> Space::GetIntersectTriangles() {
    std::set<size_t> ans;

    for (int i = 0; i < NumOfTr_; i++ ) {

        for (int s = i + 1; s < NumOfTr_; s++) {

            if (CheckCollision(triangles_[i], triangles_[s])) {
                ans.insert(triangles_[i].number_);
                ans.insert(triangles_[s].number_);
            }

        }

    }

    return ans;

}


geo::Plane Space::GetPlaneFromTr(const geo::Triangle &t) noexcept {

    const geo::Vector K01 = t.D1_ - t.D0_;
    const geo::Vector K02 = t.D2_ - t.D0_;

    geo::Vector N = K01^K02;
    geo::Normalization(N);

    return {t.D0_,N};
}

std::pair<geo::Vector, Intersec> Space::IntersecLineLine(const geo::Line& l, const geo::Line& r) {

    if (!l.Valid() || !r.Valid()) {
        return {geo::Vector(NAN), Intersec::Error};
    }

    const geo::Vector pp = r.D_ - l.D_;
    const double a = l.A_ * l.A_;
    const double b = l.A_ * r.A_;
    const double c = pp * l.A_;
    const double d = r.A_ * r.A_;
    const double f = pp * r.A_;
    const double det = -a * d + b * b;

    std::pair<geo::Vector, Intersec> res(0.f, Intersec::Nop);
    if (std::abs(det) < geo::eps)
    {
        //std::cout<<"tut1"<<std::endl;
        if (geo::AreCoDirected((l.D_ - r.D_),l.A_))
        {
            //std::cout<<"tut2"<<std::endl;
            res.second = Intersec::Same;
        }
    }
    else
    {

        const double det1 = -d * c + b * f;
        const double det2 = a * f - b * c;

        const double S1 = det1 / det;
        const double S2 = det2 / det;

        const geo::Vector D1 = l.D_ + l.A_ * S1;
        const geo::Vector D2 = r.D_ + r.A_ * S2;

        if (D1 == D2)
        {
            res.first = D1;
            res.second = Intersec::Dot;
        }
    }

    return res;

}

std::pair<geo::Vector, Intersec> Space::IntersecCutLine(const geo::Cut& c, const geo::Line& l) {

    const auto tmp_res = IntersecLineLine(geo::Line(c.B_ - c.A_, c.A_), l);
    //std::cout<<"Ebat"<<int(tmp_res.second)<<std::endl;

    if (tmp_res.second == Intersec::Dot)
    {
        if ( geo::AreCoDirected((c.A_ -c.B_), (c.A_ - tmp_res.first)) ) {
            return std::make_pair(tmp_res.first, Intersec::Dot);
        }

    }
    else if (tmp_res.second == Intersec::Same)
    {
        return std::make_pair(geo::Vector(0.f), Intersec::Same);
    }

    return std::make_pair(geo::Vector(0.f), Intersec::Nop);;
}

std::pair<geo::Line, Intersec> Space::IntersecPlanePlane(const geo::Plane& p1, const geo::Plane& p2) {

    /*                _____________
     *               /  .D2       /
     *  ____________/____________/_____
     *  \          /            /      \
     *   \        /            /        \
     *    \      /    T       /          \
     *     \    /____._______/      .D1   \
     *      \______________________________\
     *        /            /
     *       /            /
     *      /____________/
     *
     *
     *    __  {  ((T- D1)*N1) = 0;          T(x0,y0,z0), D1(x1,y1,z1),  D2(x2,y2,z2)
     *   |    {  ((T- D2)*N2) = 0;          N1(X1,Y1,Z1),   N2(X2,Y2,Z2)
     *   |
     *   |->    || X1 Y1 Z1 | b1 ||   , b1 = x1*X1 + y1*Y1 + z1*Z1
     *    _____ || X2 Y2 Z2 | b2 ||   , b2 = x2*X2 + y2*Y2 + z2*Z2
     *   |
     *   |     ..............................................................
     *   |--->x0 = (a1 - a13 * z0)/a11 ,    y0 = (a2 - a23 * z0)/a22
     *
     */


    if (geo::AreParallel(p1,p2) ) {
        if (geo::AreCoincide(p1, p2)) {

            return std::make_pair(geo::Line(p1.D0_, p1.D0_), Intersec::Same);
        }
        else
            return std::make_pair(geo::Line(p1.D0_, p1.D0_), Intersec::Nop);

    }

    const double q = p1.N_ * p1.N_;
    const double w = p1.N_ * p2.N_;
    const double l = p2.N_ * p2.N_;
    const double s1 = p1.D0_ * p1.N_;
    const double s2 = p2.D0_ * p2.N_;

    const double det = q * l - w * w;
    if (std::abs(det) < geo::eps / 100000.0)
    {
        throw std::invalid_argument("Something error1");
    }
    else
    {
        const double det1 = s1 * l - w * s2;
        const double det2 = q * s2 - w * s1;

        const double a = det1 / det;
        const double b = det2 / det;

        const geo::Vector D = p1.N_ * a + p2.N_ * b;
        const geo::Vector A = p1.N_ ^ p2.N_;

        return std::make_pair(geo::Line(A, D),Intersec::Line);
    }

    return std::make_pair(geo::Line(p1.N_,p1.N_), Intersec::Error);
}

std::pair<geo::Cut, Intersec> Space::IntersecLineAndTr(const geo::Line& l, const geo::Triangle& t, const geo::Triangle &t1) {

    const geo::Vector a1 = l.projection(t.D0_);
    const geo::Vector b1 = l.projection(t.D1_);
    const geo::Vector c1 = l.projection(t.D2_);


    const geo::Vector  a1_a = t.D0_ - a1;
    const geo::Vector  b1_b = t.D1_ - b1;
    const geo::Vector  c1_c = t.D2_ - c1;


    const geo::Vector  norm_line = GetPlaneFromTr(t).N_ ^ l.A_;

    const double a1_a_dist = (a1_a).Modul() * ((geo::AreCoDirected(norm_line, a1_a) ? 1.0 : -1.0));
    const double b1_b_dist = (b1_b).Modul() * ((geo::AreCoDirected(norm_line, b1_b) ? 1.0 : -1.0));
    const double c1_c_dist = (c1_c).Modul() * ((geo::AreCoDirected(norm_line, c1_c) ? 1.0 : -1.0));


    if (std::abs(a1_a_dist) > geo::eps && std::abs(b1_b_dist) > geo::eps && std::abs(c1_c_dist) > geo::eps
        && a1_a_dist * b1_b_dist > 0.0 && a1_a_dist * c1_c_dist > 0.0)
    {

        return std::make_pair(geo::Cut{l.D_, l.D_}, Intersec::Nop);
    }

    if (std::abs(a1_a_dist) < geo::eps)
    {
        if (std::abs(b1_b_dist) < geo::eps)
        {
            if (std::abs(c1_c_dist) < geo::eps) {
                throw std::invalid_argument("Unreal case");
            }

            return std::make_pair(geo::Cut(t.D0_, t.D1_), Intersec::Cut);
        }

        else if (std::abs(c1_c_dist) < geo::eps)
        {

            return std::make_pair(geo::Cut(t.D0_, t.D2_), Intersec::Cut);
        }
        else if (c1_c_dist * b1_b_dist > 0.f)
        {

            return std::make_pair(geo::Cut(t.D0_, t.D0_), Intersec::Dot);
        }
        else
        {
            const geo::Vector A2 = b1 + (c1 - b1) * ( std::abs(b1_b_dist) / (std::abs(c1_c_dist) + std::abs(b1_b_dist)) );

            return std::make_pair(geo::Cut(t.D0_, A2), Intersec::Cut);
        }
    }
    else if (std::abs(b1_b_dist) < geo::eps)
    {
        if (std::abs(c1_c_dist) < geo::eps)
        {

            return std::make_pair(geo::Cut(t.D1_, t.D2_), Intersec::Cut);
        }
        else if (c1_c_dist * a1_a_dist > 0.f)
        {

            return std::make_pair(geo::Cut(t.D1_, t.D1_), Intersec::Dot);
        }
        else
        {
            const geo::Vector B2 = a1 + (c1 - a1) * (std::abs(a1_a_dist) / (std::abs(c1_c_dist) + std::abs(a1_a_dist)));

            return std::make_pair(geo::Cut(t.D1_, B2), Intersec::Cut);
        }
    }
    else if (std::abs(c1_c_dist) < geo::eps)
    {
        if (a1_a_dist * b1_b_dist > 0.f)
        {
            return std::make_pair(geo::Cut(t.D2_, t.D2_), Intersec::Dot);
        }
        else
        {
            const geo::Vector C2 = b1 + (a1 - b1) * (std::abs(b1_b_dist) / (std::abs(a1_a_dist) + std::abs(b1_b_dist)) );

            return std::make_pair(geo::Cut(t.D2_, C2), Intersec::Cut);
        }
    }
    else
    {

        if (a1_a_dist * b1_b_dist > 0.f)
        {

            const auto res_ca = IntersecCutLine(geo::Cut(t.D2_, t.D0_), l);
            const auto res_cb = IntersecCutLine(geo::Cut(t.D2_, t.D1_), l);

            if (res_ca.second == Intersec::Nop || res_cb.second == Intersec::Nop)
                throw std::invalid_argument("Something error2");


            return std::make_pair(geo::Cut(res_ca.first, res_cb.first), Intersec::Cut);
        }
        else if (b1_b_dist * c1_c_dist > 0.f)
        {

            const auto res_ab = IntersecCutLine(geo::Cut(t.D0_, t.D1_), l);
            const auto res_ac = IntersecCutLine(geo::Cut(t.D0_, t.D2_), l);

            if (res_ab.second == Intersec::Nop || res_ac.second == Intersec::Nop) {
                throw std::invalid_argument("Something error3");
                //return std::make_pair(geo::Cut{l.D_, l.D_}, Intersec::Nop);
            }

            return std::make_pair(geo::Cut(res_ab.first, res_ac.first), Intersec::Cut);
        }
        else if (c1_c_dist * a1_a_dist > 0.f)
        {

            const auto res_bc = IntersecCutLine(geo::Cut(t.D1_, t.D2_), l);
            const auto res_ba = IntersecCutLine(geo::Cut(t.D1_, t.D0_), l);

            if (res_bc.second == Intersec::Nop || res_ba.second == Intersec::Nop) {
               // std::cout<<"Er: "<<a1_a_dist<<" "<<b1_b_dist<<" "<<c1_c_dist<<std::endl;
               throw std::invalid_argument("Something error4");
                //return std::make_pair(geo::Cut{l.D_, l.D_}, Intersec::Nop);
            }

            return std::make_pair(geo::Cut(res_ba.first, res_bc.first), Intersec::Cut);
        }
        else {  throw std::invalid_argument("Something error5"); }
    }

}

//.........................CheckColision............................

bool Space::CheckCollision(const geo::Triangle &t1, const geo::Triangle &t2) {

    geo::Plane p1 = GetPlaneFromTr(t1);
    geo::Normalization(p1.N_);
    geo::Plane p2 = GetPlaneFromTr(t2);
    geo::Normalization(p2.N_);


    auto r = IntersecPlanePlane(p1, p2);

    if (r.second == Intersec::Nop)
        return false;
    else if (r.second == Intersec::Same) {
        return (IsInclude(t1, t2.D0_) || IsInclude(t1, t2.D1_) || IsInclude(t1, t2.D2_));
    }

    auto c1 = IntersecLineAndTr(r.first, t1, t2);

    auto c2 = IntersecLineAndTr(r.first, t2, t1);

    if (c1.second == Intersec:: Error || c2.second == Intersec::Error) {
        throw std::invalid_argument("Something error6");
    }
    else if (c1.second == Intersec:: Nop || c2.second == Intersec::Nop) {
        return false;
    }


    if (c1.second == Intersec::Dot && c2.second == Intersec::Cut) {
        return IsInclude(c2.first, c1.first.A_);
    }
    if (c2.second == Intersec::Dot && c1.second == Intersec::Cut) {
        return IsInclude(c1.first, c2.first.A_);
    }
    if (c1.second == Intersec::Dot && c2.second == Intersec::Dot) {
        return c1.first.A_ == c2.first.B_;
    }
    if (c1.second == Intersec::Cut && c2.second == Intersec::Cut) {
        return (IsInclude(c1.first, c2.first.A_) || IsInclude(c1.first, c2.first.B_) ||
                IsInclude(c2.first, c1.first.A_) || IsInclude(c2.first, c1.first.B_) );
    }

    assert(0);
    return false;
}


bool Space::CheckCollision(const geo::Rectangle& l, const geo::Rectangle& r) noexcept {

    if ( (l.v_min_.x_ - geo::eps < r.v_min_.x_ && r.v_min_.x_ < l.v_max_.x_ + geo::eps) ||
         (l.v_min_.x_ - geo::eps < r.v_max_.x_ && r.v_max_.x_ < l.v_max_.x_ + geo::eps)) {

        if ( (l.v_min_.y_ - geo::eps < r.v_min_.y_ && r.v_min_.y_ < l.v_max_.y_ + geo::eps) ||
             (l.v_min_.y_ - geo::eps < r.v_max_.y_ && r.v_max_.y_ < l.v_max_.y_ + geo::eps)) {

            if ( (l.v_min_.z_ - geo::eps < r.v_min_.z_ && r.v_min_.z_ < l.v_max_.z_ + geo::eps) ||
                 (l.v_min_.z_ - geo::eps < r.v_max_.z_ && r.v_max_.z_ < l.v_max_.z_ + geo::eps)) {

                return true;

            }

        }

    }

    return false;
}

bool Space::CheckCollision(const geo::Rectangle& r, const geo::Triangle& t) noexcept {

    bool b1 = IsInclude(r, t.D0_);
    bool b2 = IsInclude(r, t.D1_);
    bool b3 = IsInclude(r, t.D2_);

    return (int(b1) + int(b2) + int(b3)) % 3;
}


//.............................CheckColision_end..........................