#pragma once
#include "geom.h"
#include <vector>

class AbstractFilter{
public:
    enum FilterType{
        POLAR,
        CARTESIAN,
    };

    AbstractFilter(FilterType type) : m_filterType(type){};
    virtual bool pass(DataPoint) = 0;
    inline FilterType type() const { return m_filterType; }
private:
    FilterType m_filterType;
};


class PolarFilter : public AbstractFilter{
public:
    PolarFilter();
    bool pass(DataPoint) override;

    void setMaxDistance(int maxDist);
    void setMinDistance(int minDist);
    void setMaxAngle(int maxAngle);
    void setMinAngle(int minAngle);
    void setDistanceRange(int minDist, int maxDist);
    void setAngleRange(int minAngle, int maxAngle);

private:
    //Polar filter
    int _minDist;   // Minimum Distance mm
    int _maxDist;   // Maximum Distance mm
    int _minAngle;  // Minimum angle °
    int _maxAngle;  // Maximum angle °
    int _threshold; // Minimum point intensity
};


class CartesianFilter : public AbstractFilter{
public:
    CartesianFilter();
    bool pass(DataPoint) override;
    void setGridSize(float w, float h);
private:
    //Cartesian filter
    float _cart_max_x;  //abs rectangle filter
    float _cart_max_y;  //abs rectangle filter
};

