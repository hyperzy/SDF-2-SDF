//
// Created by himalaya on 12/6/20.
//

#include "grid3d.h"
#include <fstream>

using namespace std;

// todo: initialization have not set yet
GridBase::GridBase(DimUnit depth, DimUnit height, DimUnit width):
        m_depth(depth), m_height(height), m_width(width){
}

GridBase::GridBase():m_depth(0), m_height(0), m_width(0) {}

DimUnit GridBase::getDepth() const {
    return this->m_depth;
}

DimUnit GridBase::getWidth() const {
    return this->m_width;
}

DimUnit GridBase::getHeight() const {
    return this->m_height;
}

void GridBase::setDepth(int depth) {
    if (depth <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_depth = depth;
}

void GridBase::setHeight(int height) {
    if (height <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_height = height;
}

void GridBase::setWidth(int width) {
    if (width <= 0)
        cerr << "Cannot accept non-positive value." << "\n";
    m_width = width;
}

void Grid3d::savePhi(const std::string &file_path) {
    ofstream fout;
    fout.open(file_path, ios::binary | ios::out);
    if (!fout.is_open()) {
        cerr << "file not exits" << endl;
        exit(EXIT_FAILURE);
    }
    fout.write(reinterpret_cast<const char*>(&this->phi[0]), m_depth * m_height * m_width * sizeof(dtype));
    fout.close();
}

void Grid3d::readPhi(const std::string &file_path) {
    ifstream fin;
    fin.open(file_path, ios::binary | ios::in);
    if (!fin.is_open()) {
        cerr << "file not exits" << endl;
        exit(EXIT_FAILURE);
    }
    fin.read(reinterpret_cast<char *>(&this->phi[0]),m_depth * m_height * m_width * sizeof(dtype));
    fin.close();
}

Grid3d::Grid3d(DimUnit depth, DimUnit height, DimUnit width) : GridBase(depth, height, width) {

}

Grid3d::Grid3d(): GridBase() {

}

dtype Grid3d::getResolution() const {
    if (m_resolution <= 0) {
        cerr << "Resolution is not set yet." << endl;
    }
    return m_resolution;
}

void Grid3d::Init() {
    unsigned long total_num = m_height * m_width * m_depth;
    this->phi.resize(total_num, -INF);
    this->coord.resize(total_num);
}

void Grid3d::initCoord(Vec3 origin, dtype resolution) {
    m_resolution = resolution;
    m_min_coord = origin;
    m_max_coord = origin + Vec3(m_width, m_height, m_depth) * m_resolution;
    for (int i = 0; i < m_depth; i++) {
        for (int j = 0; j < m_height; j++) {
            for (int k = 0; k < m_width; k++) {
                auto idx = this->Index(i, j, k);
                this->coord[idx][0] = origin[0] + k * resolution;       // x
                this->coord[idx][1] = origin[1] + j * resolution;       // y
                this->coord[idx][2] = origin[2] + i * resolution;       // z
                this->coord[idx][3] = 1;                                // 1 for homogeneous coordinate
            }
        }
    }
}