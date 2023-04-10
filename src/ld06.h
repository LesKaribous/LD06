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

struct Sector{
    std::vector<float> distances;
    int averageDistance = 0; // mm
    float minDist = 0;
    float maxDist = 0;

    void compute(){
        averageDistance = 0;
        minDist = infinityf();
        maxDist = 0;
        for (size_t i = 0; i < distances.size(); i++){
            averageDistance += distances[i];
            if(distances[i] < minDist) minDist = distances[i];
            if(distances[i] > maxDist) maxDist = distances[i];
        }
        averageDistance /= distances.size();
        if(minDist == infinityf()) minDist = 10000;
    }

    void clear(){
        distances.clear();
    }

    void Add(const DataPoint& p){
        distances.push_back(p.distance);
    }
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
    void printFOVLidarView();
    void printSectorsLidarView();
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

    //Sectors
    void enableSectoring();
    void disableSectoring();
    void setSectorsResolution(int angle);
    float getDistanceAtAngle(int angle); //Faster with sectoring enable

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

    //Sectoring
    std::vector<Sector> sectors;
    int _sectorResolution = 10;
    bool _useSectoring = false;

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