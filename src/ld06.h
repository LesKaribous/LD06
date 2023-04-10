#pragma once
#include <vector>
#include <cstdint>
#include <Arduino.h>

const uint8_t PTS_PER_PACKETS = 12;

struct DataPoint{
    uint16_t distance; // mm
    float angle;       // degrees
    int16_t x;         // mm
    int16_t y;         // mm
    uint8_t intensity; // 0-255
};

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
    void printScanTeleplot(); // Print full scan using teleplot format (check :https://teleplot.fr/)

    // Settings
    void enableCRC();  // Enable CRC checking
    void disableCRC(); // Disable CRC checking
    void enableFiltering();
    void disableFiltering();
    void setIntensityThreshold(int threshold);
    void setMaxDistance(int maxDist);
    void setMinDistance(int minDist);
    void setMaxAngle(int maxAngle);
    void setMinAngle(int minAngle);
    void setDistanceRange(int minDist, int maxDist);
    void setAngleRange(int minAngle, int maxAngle);

    // Getters
    inline float getSpeed() const { return _speed; }
    inline float getAngleStep() const { return _angleStep; }
    inline float getTimeStamp() const { return _timeStamp; }

private:
    bool readDataCRC();
    bool readDataNoCRC();
    void computeData(uint8_t *values);
    bool filter(const DataPoint &point);

    // Settings
    bool _useCRC = true;
    bool _useFiltering = false;
    const int _pin;

    // Filtering Settings
    int _minDist = 0;     // Minimum Distance mm
    int _maxDist = 1000;  // Maximum Distance mm
    int _minAngle = 0;    // Minimum angle °
    int _maxAngle = 360;  // Maximum angle °
    int _threshold = 100; // Minimum point intensity

    // Data
    std::vector<DataPoint> scan;

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