#include "Octotree.h"
#include <fstream>


int main() {

    int N = 0;

    std::ifstream str("../test5.txt");

    if (!str.is_open()) {
        std::cout << "Couldn't open file." << '\n';
        exit(-1);
    }

    str>>N;

    std::vector<geo::Triangle> triangles;

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

    try {

        Octotree tree(triangles);
        tree.PrintIntersecTriangles();

    }catch (std::exception& e) {
        std::cerr<< e.what()<< std::endl;
        exit(-1);
    }

    return  0;
}