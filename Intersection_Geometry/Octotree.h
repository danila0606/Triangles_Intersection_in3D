#include "Space.h"


/*
 *
 *  y
 *  ^
 *  |   1st half (z = 0)            2nd half (z = 1/2)
 *
 *  |                                             .v_max_
 *  |           |                          |
 *  |       3   |   2                  7   |   6
 *  |           |                          |
 *  |       -----------              -------------
 *  |           |                          |
 *  |       0   |   1                  4   |   5
 *  |      .    |                          |
 *  |   v_min_
 *  .z------------------------->x
 *
 *
 * */


//template <typename T>
class Node {
public:

    //...............Constructor..................

    Node() {
        for (auto i : children_)
            i = nullptr;
    };

    explicit Node(const std::vector<geo::Triangle>& triangles,
            const geo::Rectangle& rect = {geo::Vector(0.0), geo::Vector(0.0)})
            : rect_(rect) {

        for (auto& i : children_)
            i = nullptr;

        for(const auto& i : triangles)
            s_.AddTriangle(i);

    };
    
    //...............Constructor_end................
    
    ~Node() { for (int i = 0; i < 8; i++) {
                    if (children_[i] != nullptr)
                            delete children_[i];
        }
    }   
    
    std::vector<geo::Rectangle> SplitRect() const noexcept;
    size_t GetNumOfTr() const noexcept {return s_.GetNumOfTr();};
    std::set<size_t> GetIntersecTrNumbers();

    geo::Vector GetMaxCorner() const {return rect_.v_max_;};
    geo::Vector GetMinCorner() const {return rect_.v_min_;};

private:

    geo::Rectangle rect_;
    Space s_;
    Node *children_[8] = {};

};

//template <typename T>
class Octotree {
public:

    explicit Octotree(const std::vector<geo::Triangle>& triangles)  {

        Node *m = new Node(triangles, FindBoundingRect(triangles));
        main_ = m;
    };
    
    ~Octotree() {delete main_;};

    static geo::Rectangle FindBoundingRect(const std::vector<geo::Triangle>& triangles) noexcept ;
    void PrintIntersecTriangles();
    std::set<size_t> GetIntersecTriangles();

    geo::Vector GetMaxCorner() const {return main_->GetMaxCorner();};
    geo::Vector GetMinCorner() const {return main_->GetMinCorner();};

private:

    Node* main_;

};