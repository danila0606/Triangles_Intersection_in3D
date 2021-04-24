#include "geometry.h"

namespace geo {
    bool operator==(const Vector &l, const Vector &r) {
        return ((std::abs(l.x_ - r.x_) < eps) &&
                (std::abs(l.y_ - r.y_) < eps) &&
                (std::abs(l.z_ - r.z_) < eps));
    }

    bool operator!=(const Vector &l, const Vector &r) {
        return !(l == r);
    }

    bool AreParallel(const Vector &l, const Vector &r) noexcept {

        double a = 0.f;

        if (std::abs(r.x_) > eps)
            a = l.x_ / r.x_;
        else if (std::abs(r.y_) > eps)
            a = l.y_ / r.y_;
        else if (std::abs(r.z_) > eps)
            a = l.z_ / r.z_;
        else return true;

        auto res = l == Vector(r.x_ * a, r.y_ * a, r.z_ * a);
        return res;
    }

    Vector operator+(const Vector &a, const Vector &b) {
        return Vector(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_);
    }

    Vector operator-(const Vector &that) {
        Vector V(-that.x_, -that.y_, -that.z_);
        return V;
    }

    Vector operator-(const Vector &a, const Vector &b) {
        return Vector(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_);
    }

    double operator*(const Vector &l, const Vector &r) {
        return l.x_ * r.x_ + l.y_ * r.y_ + l.z_ * r.z_;
    }

    Vector operator*(const double a, const Vector &r) {
        return Vector{a * r.x_, a * r.y_, a * r.z_};
    }

    Vector operator*(const Vector &r, const double a) {
        return Vector{a * r.x_, a * r.y_, a * r.z_};
    }

    Vector operator/(const Vector &r, const double a) {
        if (std::abs(a) < eps)
            return Vector(NAN);

        return Vector{r.x_ / a, r.y_ / a, r.z_ / a};
    }

    Vector operator^(const Vector &l, const Vector &r) {
        return Vector{l.y_ * r.z_ - l.z_ * r.y_,
                      l.z_ * r.x_ - l.x_ * r.z_,
                      l.x_ * r.y_ - l.y_ * r.x_};
    }

    void Normalization(Vector &v) noexcept {
        const double modul = v.Modul();
        if (modul < pow(eps, 2))
            return;

        v.x_ /= modul;
        v.y_ /= modul;
        v.z_ /= modul;
    }

    bool AreCoDirected(const Vector &l, const Vector &r) noexcept {

        return AreParallel(l, r) && (r * l > -eps);
    }

    bool operator==(const Point &l, const Point &r) {
        return l.V_ == r.V_;
    }

    bool operator ==(const Triangle& t1, const Triangle& t2) {
        return t1.D0_ == t2.D0_ && t1.D1_ == t2.D1_ && t1.D2_ == t2.D2_;
    }

    std::string Triangle::ToString() const {

        return "[(" + std::to_string(D0_.x_) + ", " +
               std::to_string(D0_.y_) + ", " +
               std::to_string(D0_.z_) + "), (" +
               std::to_string(D1_.x_) + ", " +
               std::to_string(D1_.y_) + ", " +
               std::to_string(D1_.z_) + "), (" +
               std::to_string(D2_.x_) + ", " +
               std::to_string(D2_.y_) + ", " +
               std::to_string(D2_.z_) + ")]";
    }

    Rectangle Triangle::GetLimitRect() const {

        double x_min = 0, y_min = 0, z_min = 0;
        double x_max = 0, y_max = 0, z_max = 0;

        using std::min;
        using std::max;


        x_min = min(D0_.x_, min(D1_.x_, D2_.x_));
        y_min = min(D0_.y_, min(D1_.y_, D2_.y_));
        z_min = min(D0_.z_, min(D1_.z_, D2_.z_));
        x_max = max(D0_.x_, max(D1_.x_, D2_.x_));
        y_max = max(D0_.y_, min(D1_.y_, D2_.y_));
        z_max = max(D0_.z_, min(D1_.z_, D2_.z_));

        return {geo::Vector(x_min,y_min, z_min), geo::Vector(x_max, y_max, z_max)};
    }

    bool Triangle::Valid() const noexcept {
        return  !(D0_ == D1_) && !(D1_ == D2_) && !(D0_ == D2_) && !AreParallel((D0_ - D1_),(D0_ - D2_));
    }

    bool AreParallel (const Line& l, const Line& r) noexcept {
        return AreParallel(l.A_, r.A_);
    }

    Vector Line::projection(const Vector& p) const
    {
        if (Valid())
        {
            const double s = A_ * (p - D_);
            return D_ + A_ * s;
        }
        throw std::invalid_argument("projection error, bad line.");
    }

    bool operator == (const Plane& l, const Plane& r) {
        return (l.D0_ * l.N_) == (r.D0_ * r.N_);
    }

    bool AreParallel (const Plane& l, const Plane& r) noexcept {
        return AreParallel(l.N_,  r.N_);
    }

    bool AreCoincide(const Plane& l, const Plane& r) noexcept {

        return std::abs(std::abs(l.D0_ * l.N_) - std::abs(r.D0_ * r.N_) )< eps;
    }

    Line::Line(const Vector &A, const Vector &D) : A_(A), D_(D) {
        Normalization(A_);
    };
    Line::Line(const Vector &A, const Point &D) : A_(A), D_(D.V_) {
        Normalization(A_);
    };

    void Round(Vector& v, size_t accuracy) {

        int del = std::pow(10, accuracy);

        v.x_ = std::round(v.x_ * del) / del;
        v.y_ = std::round(v.y_ * del) / del;
        v.z_ = std::round(v.z_ * del) / del;
    }

    void R_Triangle::Rotate(double time) {

        geo::Vector d0{},d1{},d2{};
        double dt = 0.01;

        if (time > dt) {
            while (time > dt) {

                size_t del = 10000;
                Vector p0 = tr_.D0_ - line_.projection(tr_.D0_);
                d0 = tr_.D0_ + (omega_ ^ p0) * dt;
                d0.x_ = std::round(d0.x_ * del) / del;
                d0.y_ = std::round(d0.y_ * del) / del;
                d0.z_ = std::round(d0.z_ * del) / del;

                Vector p1 = tr_.D1_ - line_.projection(tr_.D1_);
                d1 = tr_.D1_ + (omega_ ^ p1) * dt;
                d1.x_ = std::round(d1.x_ * del) / del;
                d1.y_ = std::round(d1.y_ * del) / del;
                d1.z_ = std::round(d1.z_ * del) / del;

                Vector p2 = tr_.D2_ - line_.projection(tr_.D2_);
                d2 = tr_.D2_ + (omega_ ^ p2) * dt;
                d2.x_ = std::round(d2.x_ * del) / del;
                d2.y_ = std::round(d2.y_ * del) / del;
                d2.z_ = std::round(d2.z_ * del) / del;

                dt += 0.01;
            }
        }
        else {
            size_t del = 10000;
            Vector p0 = tr_.D0_ - line_.projection(tr_.D0_);
            d0 = tr_.D0_ + (omega_ ^ p0) * time;
            d0.x_ = std::round(d0.x_ * del) / del;
            d0.y_ = std::round(d0.y_ * del) / del;
            d0.z_ = std::round(d0.z_ * del) / del;

            Vector p1 = tr_.D1_ - line_.projection(tr_.D1_);
            d1 = tr_.D1_ + (omega_ ^ p1) * time;
            d1.x_ = std::round(d1.x_ * del) / del;
            d1.y_ = std::round(d1.y_ * del) / del;
            d1.z_ = std::round(d1.z_ * del) / del;

            Vector p2 = tr_.D2_ - line_.projection(tr_.D2_);
            d2 = tr_.D2_ + (omega_ ^ p2) * time;
            d2.x_ = std::round(d2.x_ * del) / del;
            d2.y_ = std::round(d2.y_ * del) / del;
            d2.z_ = std::round(d2.z_ * del) / del;

        }
        std::vector<Vector> t = {d0, d1, d2};

        std::sort(t.begin(), t.end(), [](const geo::Vector l, const geo::Vector r) {
            return l.Modul() < r.Modul();
        } );

        tr_.D0_ = t[0];
        tr_.D1_ = t[1];
        tr_.D2_ = t[2];
    }
}


