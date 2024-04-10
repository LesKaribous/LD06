#pragma once
#include "geom.h"
#include <vector>

struct Sector{
    std::vector<float> distances;
    float averageAngle; //degrees
    float averageDistance = 0; // mm
    float minDist = 0;
    float maxDist = 0;

    void compute();
    void clear();
    void Add(const DataPoint& p);
};



class AbstractGrid{
public:
    enum GridType
    {
        POLAR,
        CARTESIAN,
    };

    AbstractGrid(GridType type) : m_filterType(type){};

    void store()

    inline GridType type() const { return m_filterType; }
private:
    GridType m_filterType;
};


class PolarGrid : public AbstractGrid{
public:
    PolarGrid() : AbstractGrid(POLAR){}

private:
        // Filtering Settings
    //Polar filter
    int _minDist = 0;     // Minimum Distance mm
    int _maxDist = 1000;  // Maximum Distance mm
    int _minAngle = 0;    // Minimum angle °
    int _maxAngle = 360;  // Maximum angle °
    int _threshold = 100; // Minimum point intensity

    //Cartesian filter
    float _cart_min_x = -1000; //abs rectangle filter
    float _cart_min_y = -1000; //abs rectangle filter
    float _cart_max_x = 1000;  //abs rectangle filter
    float _cart_max_y = 1000;  //abs rectangle filter
};

class CartesianGrid : public AbstractGrid{
public:
    CartesianGrid() : AbstractGrid(CARTESIAN){}

private:
    //Cartesian filter
    float _cart_min_x = -1000; //abs rectangle filter
    float _cart_min_y = -1000; //abs rectangle filter
    float _cart_max_x = 1000;  //abs rectangle filter
    float _cart_max_y = 1000;  //abs rectangle filter
};

