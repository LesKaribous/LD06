#pragma once

#include "filter.h"
#include <Arduino.h>

const uint8_t PTS_PER_PACKETS = 12;

class LD06{
public:
    LD06(int pin, HardwareSerial& serial = Serial1);

    void begin();

    // Read data from lidar
    bool readData();
    bool readScan(int count);
    bool readFullScan();

    // Print Data over Serial
    void printScanCSV();      // Print full scan using csv format
    void printScanLidarView();
    void printFOVLidarView();
    void printSectorsLidarView();
    void printScanTeleplot(); // Print full scan using teleplot format (check :https://teleplot.fr/)

    // Settings
    void enableCRC();  // Enable CRC checking
    void disableCRC(); // Disable CRC checking
    void enableFiltering();
    void disableFiltering();

    void setFilter(FilterType type); // Set filter

    //Filtering
    void setIntensityThreshold(int threshold);
    void setMaxDistance(int maxDist);
    void setMinDistance(int minDist);
    void setMaxAngle(int maxAngle);
    void setMinAngle(int minAngle);
    void setDistanceRange(int minDist, int maxDist);
    void setAngleRange(int minAngle, int maxAngle);

    void setCartesianBoundaries(float minx, float miny, float maxx, float maxy);

    //Sectors
    void enableSectoring();
    void disableSectoring();
    void setSectorsResolution(int angle);
    float getDistanceAtAngle(int angle); //Faster with sectoring enable
    std::vector<PolarVector> getAverageDistanceField(); //Faster with sectoring enable

    // Getters
    inline float getSpeed() const { return _speed; }
    inline float getAngleStep() const { return _angleStep; }
    inline float getTimeStamp() const { return _timeStamp; }

private:
    bool readDataCRC();
    bool readDataNoCRC();
    void computeData(uint8_t *values);
    bool filter(const DataPoint &point);

    //Attributes
    const int _pin;
    AbstractFilter* _filter = nullptr;

    // Settings
    bool _useCRC = true;
    bool _useFiltering = true; //If set it will be used

    // Data
    std::vector<DataPoint> scan;

    
    //Sectoring
    std::vector<Sector> sectors;
    int _sectorResolution = 10; //° range (each sector is x° wide)
    bool _useSectoring = false;

    //Grid
    std::vector<Cell> grid;
    int _gridResolution = 50; //mm range (each cell is x mm wide)
    bool _useGrid = false;

    // Temporary variables
    float _speed;
    float _FSA;
    float _LSA;
    float _angleStep;
    int _timeStamp;

    //Serial
    HardwareSerial* _lidarSerial;

    // Reading buffers
    float angles[PTS_PER_PACKETS];
    uint16_t distances[PTS_PER_PACKETS];
    uint8_t confidences[PTS_PER_PACKETS];
};