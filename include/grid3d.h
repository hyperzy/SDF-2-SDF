//
// Created by himalaya on 12/6/20.
//

#ifndef SDF_2_SDF_GRID3D_H
#define SDF_2_SDF_GRID3D_H

#include "base.h"
#include <memory>

/****** Note ********/
/** 1-d array will be arranged as z->y->x order instead of intuitive x->y->z order
 *  so i, j, k correspond to z, y, x
 *  (depth, height, width)
 */

struct IndexSet {
    IndexSet(int i, int j, int k): i(i), j(j), k(k) {}
    int i, j, k;
};


class GridBase {
protected:
    DimUnit m_depth, m_height, m_width;
public:
    GridBase(DimUnit depth, DimUnit height, DimUnit width);
    unsigned long Index(IdxType i, IdxType j, IdxType k) const;
    bool isValidRange(IdxType i, IdxType j, IdxType k) const;
    DimUnit getDepth() const;
    DimUnit getWidth() const;
    DimUnit getHeight() const;
    virtual void Init() = 0;
};

class Grid3d: public GridBase {
public:
    // todo: dimension has not set yet
    Grid3d(DimUnit depth, DimUnit height, DimUnit width);
    // signed distance function
    // todo: initialize phi to -INF
    std::vector<dtype> phi;
    std::vector<Vec4> coord;
    std::vector<IndexSet> front;
    void savePhi(const std::string &file_path);
    void readPhi(const std::string &file_path);
    void Init() override;
    // todo: use x_resolution, y_resolution, z_resolution
    void initCoord(Vec3 origin, dtype resolution);
};

inline unsigned long GridBase::Index(IdxType i, IdxType j, IdxType k) const {
    return i * m_width * m_height + j * m_width + k;
}

inline bool GridBase::isValidRange(IdxType i, IdxType j, IdxType k) const {
    return i >= 0 && i < m_depth && j >= 0 && j < m_height
           && k >= 0 && k < m_width;
}

inline unsigned long Index(int i, int j, int k, int depth, int height, int width) {
    return i * width * height + j * width + k;
}
#endif //SDF_2_SDF_GRID3D_H
