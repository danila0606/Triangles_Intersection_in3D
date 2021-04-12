#include "geometry.h"

//.................Space................

#define COS_BETWEEN(x, y) ( ((x) * (y)) / (((x).Modul()) * ((y).Modul()) ) ) // x and y - vectors

enum class Intersec{
    Nop,
    Dot,
    Line,
    Cut,
    Same,
    Error
};

class Space {
public:
    Space() = default;

    static std::pair<geo::Vector, Intersec> IntersecLineLine( const geo::Line& l, const geo::Line& r);
    static std::pair<geo::Vector, Intersec> IntersecCutLine(const geo::Cut& c, const geo::Line& l);
    static std::pair<geo::Cut, Intersec> IntersecLineAndTr(const geo::Line& l, const geo::Triangle& t, const geo::Triangle &t1);
    static std::pair<geo::Line, Intersec> IntersecPlanePlane(const geo::Plane& p1, const geo::Plane& p2);

    static bool IsInclude(const geo::Triangle& t,const geo::Vector& p);
    static bool IsInclude(const geo::Plane& t,const geo::Vector& p) noexcept;
    static bool IsInclude(const geo::Cut& c, const geo::Vector& p) noexcept;
    static bool IsInclude(const geo::Line& l, const geo::Vector& p) noexcept;
    static bool IsInclude(const geo::Rectangle& l, const geo::Rectangle& r) noexcept ;
    static bool IsInclude(const geo::Rectangle& r, const geo::Vector& p) noexcept ;

    static geo::Plane GetPlaneFromTr(const geo::Triangle& t) noexcept ;

    static bool CheckCollision (const geo::Triangle& t1, const geo::Triangle& t2);
    static bool CheckCollision(const geo::Rectangle& l, const geo::Rectangle& r) noexcept;
    static bool CheckCollision(const geo::Rectangle& r, const geo::Triangle& t) noexcept;
    std::set<size_t> GetIntersectTriangles();

    void AddTriangle(const geo::Triangle& t) {triangles_.push_back(t); NumOfTr_++;};
    size_t GetNumOfTr() const noexcept {return NumOfTr_;};
    std::vector<geo::Triangle> GetTriangles() const noexcept {return triangles_;};

private:

    std::vector <geo::Triangle> triangles_;
    size_t NumOfTr_ = 0;

};

//.................Space_end............

