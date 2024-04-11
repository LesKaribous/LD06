#pragma once
#include "geom.h"
#include <vector>
#include <unordered_map>

class AbstractGrid{
public:
    enum GridType{
        POLAR,
        CARTESIAN,
    };

    AbstractGrid(GridType type) : m_gridrType(type){};
    virtual void store(DataPoint) = 0;
    virtual void unstore(DataPoint) = 0;
    virtual void compute() = 0;
    virtual void clear() = 0;

    inline GridType type() const { return m_gridrType; }
private:
    GridType m_gridrType;
};

class PolarGrid : public AbstractGrid{
public:
    PolarGrid();
    void store(DataPoint) override;
    void unstore(DataPoint) override;
    void compute() override;
    void clear() override;

    void setSectorsResolution(int angle);
    float getDistanceAtAngle(int angle); //Faster with sectoring enable
    void printSectors();

    struct Sector{
        float distancesSum;
        int count = 0;

        float averageAngle; //degrees
        float averageDistance = 0; // mm

        void compute();
        void clear();
        void add(const DataPoint& p);
        void remove(const DataPoint& p);
    };

private:
    std::vector<Sector> sectors;
    int _sectorResolution = 10; //° range (each sector is x° wide)
};

class CartesianGrid : public AbstractGrid{
public:

    enum GridMode{
        CORNER //rectangle start in up left corner
    };

    CartesianGrid();
    void store(DataPoint) override;
    void unstore(DataPoint) override;
    void compute() override;
    void clear() override;
    float getDistance(float x, float y, float theta);
    void setGridSize(float w, float h);
    void setResolution(float cellSize);
    void setMode(GridMode mode);
    bool isOccupied(int x, int y);
    
private:
    std::unordered_map<int, bool> cells; //index and count
    float _gridResolution;   //mm
    int _gridRow;
    int _gridCol;
    int _gridHeight;
    int _gridWidth;

    GridMode _mode = CORNER;
};

