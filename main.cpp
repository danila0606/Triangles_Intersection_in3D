#pragma once

#include <fstream>
#include "Loader.h"

std::vector<geo::R_Triangle> GetTrianglesFromCin(double& time);

int main(int argc, char* argv[]) {

    try {

        double time;
        auto trs = GetTrianglesFromCin(time);
        Loader loader(trs, time);
        loader.Start();

    } catch (std::exception& e) {
        std::cerr<< e.what()<< std::endl;
        exit(-1);
    }

    return 0;
}

std::vector<geo::R_Triangle> GetTrianglesFromCin(double& time) {

    std::ifstream str("tests/rotate_test2.txt");

    if (!str.is_open()) {
        std::cerr << "Cant open file! " << std::endl;
        exit(0);
    }


    //std::istream& str = std::cin;

    int N = 0;
    str>>N>>time;
    std::vector<geo::R_Triangle> triangles;
    triangles.reserve(N);

    for (int i = 0; i < N ; i++) {
        double x, y, z;

        std::vector<geo::Vector> t;

        if (!str.good()) {
            std::cerr<<"Problems with cin!"<<std::endl;
            exit(1);
        }
        str >> x >> y >> z;
        t.emplace_back(x,y,z);
        str >> x >> y >> z;
        t.emplace_back(x,y,z);
        str>> x >> y >> z;
        t.emplace_back(x,y,z);

        std::sort(t.begin(), t.end(), [](const geo::Vector l, const geo::Vector r) {
            return l.Modul() < r.Modul();
        } );
        geo::Triangle tr(t[0], t[1], t[2],i);

        str>>x>>y>>z;
        geo::Vector A(x,y,z);
        str>>x>>y>>z;
        geo::Vector B(x,y,z);

        geo::Cut cut(A,B);
        double speed;
        str>>speed;

        triangles.emplace_back(tr,cut,speed);
    }

    return triangles;
}

