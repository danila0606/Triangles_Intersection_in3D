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

    const double eps = 0.000001; // 0.000001

    //.................Vector....................

    struct Vector final {
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

    //...................Vector_end.......................

    //...................Point............................

    struct Point final{
        Point(const double &X, const double &Y, const double &Z) :
                V_(Vector(X,Y,Z)) {};

        Vector V_;
    };

    //....................Point_end.........................

    //.......................Rectangle.........................

    struct Rectangle final {

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

    struct Triangle final {

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
        bool Valid() const noexcept;
        Rectangle GetLimitRect() const;
        std::string ToString() const ;


        Rectangle limit_rect;
        Vector D0_, D1_, D2_;
        size_t number_;
    };

    //......................Triangle_end......................

    //......................Line..............................

    struct Line final {
        Line(const Vector &A, const Vector &D);
        Line(const Vector &A, const Point &D);

        Vector projection(const Vector& p) const;

        bool Valid() const noexcept { const double mod = A_.Modul(); return (mod > eps && mod < 9999999.0);}

        Vector A_; // the direction vector of the line
        Vector D_; // point on the Line
    };

    //......................Line_end..........................

    //......................Cut...............................

    struct Cut final {
        Cut() = default;
        Cut(const Vector &A, const Vector &B) : A_(A), B_(B) {};
        Cut(const Point &A, const Point &B) : A_(A.V_), B_(B.V_) {};

        Vector A_, B_;
    };

    //.....................Cut_end............................

    //......................Plane.............................

    struct Plane final{
        Plane (const Point& D0, const Vector& N) : D0_(D0.V_), N_(N) { /*Normalization(N_)*/;};
        Plane (const Vector& D0, const Vector& N) : D0_(D0), N_(N) { /*Normalization(N_) */;};

        Vector D0_;
        Vector N_;
    };

    //.......................Plane_end.........................

    bool operator==(const Vector &l, const Vector &r);

    bool operator!=(const Vector &l, const Vector &r);

    bool AreParallel(const Vector &l, const Vector &r) noexcept;

    Vector operator+(const Vector &a, const Vector &b);

    Vector operator-(const Vector &that);

    Vector operator-(const Vector &a, const Vector &b);

    double operator*(const Vector &l, const Vector &r);

    Vector operator*(const double a, const Vector &r);

    Vector operator*(const Vector &r, const double a);

    Vector operator/(const Vector &r, const double a);

    Vector operator^(const Vector &l, const Vector &r);

    void Normalization(Vector &v) noexcept;
    bool AreCoDirected(const Vector &l, const Vector &r) noexcept;
    bool operator==(const Point &l, const Point &r);
    bool operator ==(const Triangle& t1, const Triangle& t2);
    bool operator == (const Plane& l, const Plane& r);
    bool AreParallel (const Plane& l, const Plane& r) noexcept;
    bool AreCoincide(const Plane& l, const Plane& r) noexcept;
    bool AreParallel (const Line& l, const Line& r) noexcept;

    struct R_Triangle {

        R_Triangle(Triangle tr, Cut dir, double speed) : tr_(tr) {

            omega_ = dir.B_ - dir.A_;
            Normalization(omega_);
            omega_ = omega_ * (speed/180.0 * 3.1415);
        };

        void Rotate(double time);

        Triangle tr_;
        Vector omega_;
    };

}