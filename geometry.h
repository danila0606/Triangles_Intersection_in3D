#pragma once

#include <iostream>
#include <sstream>
#include <cmath>
#include <set>
#include <vector>
#include <algorithm>
#include <cassert>
#include <string>

namespace geo {

    const double eps = 0.000001;

    //.................Vector....................

    struct Vector {
        explicit Vector(const double &X, const double &Y, const double &Z) :
                x_(X), y_(Y), z_(Z) {};
        explicit Vector(const double& t) : x_(t), y_(t), z_(t) {};
        Vector() = default;

        void PrintVector() const {std::cout << " D0: "<< x_
                                            << " D1: "<< y_
                                            << " D2: "<< z_;};
        double Modul() const {return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);};

        double x_, y_, z_;
    };


    bool operator == (const Vector& l, const Vector& r) {
        return ( (std::abs(l.x_ - r.x_) < eps) &&
                 (std::abs(l.y_ - r.y_) < eps) &&
                 (std::abs(l.z_ - r.z_) < eps)  );
    }

    bool operator != (const Vector& l, const Vector& r) {
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

        return l == Vector (r.x_ * a, r.y_ * a, r.z_ * a);
    }

    Vector operator+(const Vector& a, const Vector& b) {
        return Vector(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_);
    }

    Vector operator-(const Vector& that) {
        Vector V(-that.x_, -that.y_, -that.z_);
        return V;
    }

    Vector operator-(const Vector& a, const Vector& b) {
        return Vector(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_);
    }

    double operator *(const Vector&l, const Vector& r) {
        return l.x_ * r.x_ + l.y_ * r.y_ + l.z_ * r.z_;
    }

    Vector operator *(const double a, const Vector& r) {
        return Vector{a * r.x_, a * r.y_, a * r.z_};
    }

    Vector operator *(const Vector& r, const double a) {
        return Vector{a * r.x_, a * r.y_, a * r.z_};
    }

    Vector operator /(const Vector& r, const double a) {
        if (std::abs(a) < eps)
            return Vector(NAN);

        return Vector{ r.x_ / a, r.y_ / a, r.z_ / a};
    }

    Vector operator ^(const Vector& l, const Vector& r) {
        return Vector{ l.y_ * r.z_ - l.z_ * r.y_,
                       l.z_ * r.x_ - l.x_ * r.z_,
                       l.x_ * r.y_ - l.y_ * r.x_ };
    }

    void Normalization(Vector& v) noexcept {
        const double modul = v.Modul();
        if (modul < pow(eps,2))
            return;

        v.x_ /= modul;
        v.y_ /= modul;
        v.z_ /= modul;
    }

    bool AreCoDirected (const Vector &l, const Vector& r) noexcept {

        return AreParallel(l, r) && (r * l > -eps);
    }

    //...................Vector_end.......................

    //...................Point............................

    struct Point {
        Point(const double &X, const double &Y, const double &Z) :
                V_(Vector(X,Y,Z)) {};

        Vector V_;
    };

    bool operator == (const Point& l, const Point& r) {
        return l.V_ == r.V_;
    }

    //....................Point_end.........................

    //.......................Rectangle.........................

    struct Rectangle {

        Rectangle(const Vector& v_min, const Vector& v_max) noexcept :
                v_min_(v_min), v_max_(v_max),
                x_(v_max.x_ - v_min.x_), y_(v_max.y_ - v_min.y_), z_(v_max.z_ - v_min.z_) {};

        Rectangle() : x_(0), y_(0), z_(0),v_min_(0), v_max_(0) {};



        void Update(const Vector& v_min, const Vector& v_max)
        {
            v_min_ = v_min;
            v_max_ = v_max;
            x_ = v_max.x_ - v_min.x_;
            y_ = v_max.y_ - v_min.y_;
            z_ = v_max.z_ - v_min.z_;
        }

        Vector GetMinCoor() const noexcept {
            return v_min_;
        };
        Vector GetMaxCoor() const noexcept {
            return v_max_;
        };
        std::pair<Vector, Vector> GetCoor() const noexcept {
            return std::make_pair(v_min_, v_max_);
        };


        Vector v_min_, v_max_;
        double x_, y_, z_;
    };

    //.......................Rectangle_end.....................

    //....................Triangle..........................

    struct Triangle {

        Triangle(const Vector &A, const Vector &B, const Vector &C, int N = 0) :
        D0_(A), D1_(B), D2_(C), number_(N) {

            if(!Valid()) {
                throw std::invalid_argument("Bad triangle: " + ToString());
            }

            limit_rect = GetLimitRect();
        };

        Triangle(const Point& A, const Point& B, const Point& C, int N = 0) :
        D0_(A.V_), D1_(B.V_), D2_(C.V_), number_(N) {
            limit_rect = GetLimitRect();
        };

        void PrintTriangle() const  {std::cout<<"{("; D0_.PrintVector();
                                     std::cout<<"),("; D1_.PrintVector();
                                     std::cout<<"),("; D2_.PrintVector();std::cout<<")}";};
        bool Valid() const noexcept
        {
            return  !(D0_ == D1_) && !(D1_ == D2_) && !(D0_ == D2_) && !AreParallel((D0_ - D1_),(D0_ - D2_));
        }

        Rectangle GetLimitRect() const;
        std::string ToString() const ;


        Rectangle limit_rect;
        Vector D0_, D1_, D2_;
        size_t number_;
    };

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

    //......................Triangle_end......................

    //......................Line..............................

    struct Line {
        Line(const Vector &A, const Vector &D) : A_(A), D_(D) {
            Normalization(A_);
        };
        Line(const Vector &A, const Point &D) : A_(A), D_(D.V_) {
            Normalization(A_);
        };

        Vector projection(const Vector& p) const;

        bool Valid() const noexcept { const double mod = A_.Modul(); return (mod > eps && mod < 9999999.0);}

        Vector A_; // the direction vector of the line
        Vector D_; // point on the Line
    };

    Vector Line::projection(const Vector& p) const
    {
        if (Valid())
        {
            const double s = A_ * (p - D_);
            return D_ + A_ * s;
        }
            throw std::invalid_argument("projection error, bad line.");
    }

    bool AreParallel (const Line& l, const Line& r) noexcept {
        return AreParallel(l.A_, r.A_);
    }

    //......................Line_end..........................

    //......................Cut...............................

    struct Cut {
        Cut() = default;
        Cut(const Vector &A, const Vector &B) : A_(A), B_(B) {};
        Cut(const Point &A, const Point &B) : A_(A.V_), B_(B.V_) {};

        Vector A_, B_;
    };

    //.....................Cut_end............................

    //......................Plane.............................

    struct Plane {
        Plane (const Point& D0, const Vector& N) : D0_(D0.V_), N_(N) { /*Normalization(N_)*/;};
        Plane (const Vector& D0, const Vector& N) : D0_(D0), N_(N) { /*Normalization(N_) */;};

        Vector D0_;
        Vector N_;
    };

    bool operator == (const Plane& l, const Plane& r) {
        return (l.D0_ * l.N_) == (r.D0_ * r.N_);
    }

    bool AreParallel (const Plane& l, const Plane& r) noexcept {
        return AreParallel(l.N_,  r.N_);
    }

    bool AreCoincide(const Plane& l, const Plane& r) noexcept {

        return std::abs(std::abs(l.D0_ * l.N_) - std::abs(r.D0_ * r.N_) )< eps;
    }

    //.......................Plane_end.........................

}