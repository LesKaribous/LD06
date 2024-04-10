#include "ld06.h"
#include "Arduino.h"
#include "crc.h"

// Packets size
const uint8_t PACKET_SIZE = 47; // note: 1(Start)+1(Datalen)+2(Speed)+2(SAngle)+36(DataByte)+2(EAngle)+2(TimeStamp)+1(CRC)
// Headers
const uint8_t HEADER = 0x54;
const uint8_t VER_SIZE = 0x2c;

//#define LIDAR_SERIAL Serial1
#define LIDAR_SERIAL (*_lidarSerial)

LD06::LD06(int pin, HardwareSerial& serial) : _pin(pin), _lidarSerial(&serial){}

void LD06::begin(){
    LIDAR_SERIAL.begin(230400, SERIAL_8N1);
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, HIGH);
}

/* Read lidar packet data,
 * return : true if a valid package was received
 */
bool LD06::readData(){
    return _useCRC ? readDataCRC() : readDataNoCRC();
}

/* Read lidar packet data without checking CRC,
 * return : true if a valid package was received
 */
bool LD06::readDataCRC(){
    int result = 0;
    static uint8_t crc = 0;
    static uint8_t n = 0;
    static uint8_t lidarData[PACKET_SIZE];
    while (LIDAR_SERIAL.available())
    {
        uint8_t current = LIDAR_SERIAL.read();
        if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE))
        {
            lidarData[n] = current;
            if (n < PACKET_SIZE - 1)
            {
                crc = CrcTable[crc ^ current];
                n++;
            }
            else
            {
                if (crc == current)
                {
                    computeData(lidarData);
                    result = 1;
                }
                else
                {
                    result = -1;
                }
                n = 0;
                crc = 0;
            }
        }
        else
        {
            n = 0;
            crc = 0;
        }
    }
    return result;
}

/* Read lidar packet data without checking CRC,
 * return : true if a package was received
 */
bool LD06::readDataNoCRC(){
    static uint8_t n = 0;
    static uint8_t lidarData[PACKET_SIZE];
    while (LIDAR_SERIAL.available())
    {
        uint8_t current = LIDAR_SERIAL.read();
        if (n > 1 || (n == 0 && current == HEADER) || (n == 1 && current == VER_SIZE))
        {
            lidarData[n] = current;
            if (n == PACKET_SIZE - 1)
            {
                computeData(lidarData);
                n = 0;
                return true;
            }
        }
        else
            n = 0;
    }
    return false;
}

// Read lidar packets and return true when a new full 360° scan is available
bool LD06::readFullScan(){
    static std::vector<DataPoint> fullScan;
    static bool isInit = false;
    static float lastAngle = 0;
    bool newScan = false;
    DataPoint data;

    if(readData())
    {
        for (int i = 0; i < PTS_PER_PACKETS; i++)
        {
            if (angles[i] < lastAngle)
            {
                if (!isInit)
                {
                    isInit = true;
                }
                else
                {
                    newScan = true;
                    scan.clear();
                    for (uint16_t j = 0; j < fullScan.size(); j++)
                    {
                        if(_useFiltering){
                            if(filter(fullScan[j]))scan.push_back(fullScan[j]);
                        }else scan.push_back(fullScan[j]);
                    }
                    fullScan.clear();

                }
            }
            lastAngle = angles[i];

            data.angle = angles[i];
            data.distance = distances[i];
            data.x = data.distance * cos(data.angle * DEG_TO_RAD);
            data.y = -data.distance * sin(data.angle * DEG_TO_RAD);
            data.intensity = confidences[i];
            fullScan.push_back(data);
        }
    }
    return newScan;
}

// Read lidar packets and return true scan as more than count points
bool LD06::readScan(int count){
    static std::vector<DataPoint> fullScan;
    static bool isInit = false;
    static float lastAngle = 0;
    bool newScan = false;
    DataPoint data;

    if(readData()){
        for (int i = 0; i < PTS_PER_PACKETS; i++){
            if (fullScan.size() > count){
                if (!isInit){ 
                    isInit = true;
                }else{
                    newScan = true;
                    scan.clear();
                    

                    if(_useSectoring){
                        sectors.clear();
                        sectors.resize(360.0/_sectorResolution);
                        for(int i = 0; i < sectors.size() ; i++){
                            sectors[i].averageAngle = float(i) * 360.0/float(_sectorResolution);
                        }
                    }

                    for (uint16_t j = 0; j < fullScan.size(); j++){
                        if(!_useFiltering || filter(fullScan[j])){
                            scan.push_back(fullScan[j]);
                            if(_useSectoring){
                                int index = fullScan[j].angle / float(_sectorResolution);
                                sectors[index].Add(fullScan[j]);
                            }
                        }
                    }

                    if(_useSectoring){
                        for(Sector& sector : sectors ){
                            sector.compute();
                        }
                    }
                    
                    fullScan.clear();
                }
            }

            lastAngle = angles[i];

            data.angle = angles[i];
            data.distance = distances[i];
            data.x = data.distance * cos(data.angle * DEG_TO_RAD);
            data.y = -data.distance * sin(data.angle * DEG_TO_RAD);
            data.intensity = confidences[i];
            fullScan.push_back(data);


        }
    }
    return newScan;
}

void LD06::computeData(uint8_t *values){
    _speed = float(values[3] << 8 | values[2]) / 100;
    _FSA = float(values[5] << 8 | values[4]) / 100;
    _LSA = float(values[PACKET_SIZE - 4] << 8 | values[PACKET_SIZE - 5]) / 100;
    _timeStamp = int(values[PACKET_SIZE - 2] << 8 | values[PACKET_SIZE - 3]);

    _angleStep = ((_LSA - _FSA > 0) ? (_LSA - _FSA) / (PTS_PER_PACKETS - 1) : (_LSA + (360 - _FSA)) / (PTS_PER_PACKETS - 1));

    if (_angleStep > 20)
        return;

    for (int i = 0; i < PTS_PER_PACKETS; i++)
    {
        float raw_deg = _FSA + i * _angleStep;
        angles[i] = (raw_deg <= 360 ? raw_deg : raw_deg - 360);
        confidences[i] = values[8 + i * 3];
        distances[i] = int(values[8 + i * 3 - 1] << 8 | values[8 + i * 3 - 2]);
    }
}

// Return true if point pass the filter
bool LD06::filter(const DataPoint &point){
    //if(_minAngle < 0 && point.angle < 360+_minAngle) return false;

    if ((point.angle >= _minAngle && point.angle <= _maxAngle) || 
        (point.angle + 360 >= _minAngle && point.angle + 360 <= _maxAngle) ||
        (point.angle - 360 >= _minAngle && point.angle - 360 <= _maxAngle)) {
        
        if(point.distance > _maxDist) return false;
        if(point.distance < _minDist) return false;
        if(point.intensity < _threshold) return false;
        return true;
    } else {
        return false;
    }

    //if(_minAngle >= 0 && point.angle < _minAngle) return false;
    //if( point.angle > _maxAngle) return false;
}

//Sectors
void Sector::compute(){
    averageDistance = 0;
    minDist = infinityf();
    maxDist = 0;
    for (size_t i = 0; i < distances.size(); i++){
        averageDistance += distances[i];
        if(distances[i] < minDist) minDist = distances[i];
        if(distances[i] > maxDist) maxDist = distances[i];
    }
    if(distances.size() != 0) averageDistance /= distances.size();
    if(minDist == infinityf()) minDist = 10000;
}

void Sector::clear(){
    distances.clear();
    minDist = 0;
    maxDist = 0;
    averageDistance = 0;
}

void Sector::Add(const DataPoint& p){
    distances.push_back(p.distance);
}

void LD06::setSectorsResolution(int angle){
    _sectorResolution = angle;
    sectors.clear();
    sectors.resize(360/angle); //Reserve sector count
}

float LD06::getDistanceAtAngle(int angle){ //Faster with sectoring enable
    if(_useSectoring){
        int index = angle/_sectorResolution;
        return sectors[index].averageDistance;
    }else{
        if(scan.size() == 0) return 0;
        float averageDistance = 0;
        int count = 0;
        for(auto& point : scan){
            if(point.angle > angle - 1 && point.angle < angle - +1 ){ //map on 360°
                averageDistance += point.distance;
                count++;
            }
        }
        return count == 0 ? 0 : averageDistance/float(count);
    }
}

std::vector<PolarVector> LD06::getAverageDistanceField(){

    std::vector<PolarVector> field;
    
    if(_useSectoring){
        for(auto& sector : sectors) 
            if(sector.averageDistance != 0 ) field.push_back({sector.averageAngle, sector.averageDistance});
        return field;
    }else{ //Give a field on 360 degrees.
        if(scan.size() == 0) return field;
        int count = 1;
        float currentAngle = scan[0].angle;
        float averageDistance = scan[0].distance;

        for(auto& point : scan){
            if(point.angle > currentAngle + 1){ //map on 360°
                field.push_back({currentAngle, averageDistance/count});
                currentAngle = point.angle;
                averageDistance = point.distance;
                count = 1;
            }else{
                averageDistance += point.distance;
                count ++;
            }
        }
        return field;
    }
}


// Print full scan using csv format
void LD06::printScanCSV(){
    Serial.println("Angle(°),Distance(mm),x(mm),y(mm)");
    for (uint16_t i = 0; i < scan.size(); i++)
    {
        Serial.println(String() + scan[i].angle + "," + scan[i].distance + "," + scan[i].x + "," + scan[i].y);
    }
    Serial.println("");
}

// Print full scan using teleplot format (check :https://teleplot.fr/)
void LD06::printScanTeleplot(){
    Serial.print(">lidar:");
    for (uint16_t i = 0; i < scan.size(); i++)
    {
        Serial.print(String() + scan[i].x + ":" + scan[i].y + ";");
    }
    Serial.println("|xy");
}

// Print polar full scan using teleplot format (check :https://teleplot.fr/)
void LD06::printScanLidarView(){
    Serial.print(">lidar:");
    for (uint16_t i = 0; i < scan.size(); i++)
    {
        Serial.print(String() + scan[i].angle + ":" + scan[i].distance + ";");
    }
    Serial.println("|np"); // don't plot
}

// Print sectors using teleplot format (check :https://teleplot.fr/)
void LD06::printSectorsLidarView(){
    Serial.print(">sectors:");
    for (uint16_t i = 0; i < sectors.size(); i++)
    { //Print > point count : average distance : min dist : max dist
        Serial.print(String() + i + ":" + sectors[i].distances.size() + ":" + sectors[i].averageDistance + ":" + sectors[i].minDist + ":" + sectors[i].maxDist +  ";");
    }
    Serial.println("|np"); //don't plot
}


// Print fov using teleplot format (check :https://teleplot.fr/)
void LD06::printFOVLidarView(){
    Serial.print(">fov:");
    
    Serial.print(String() + _minAngle + ":" + _maxAngle + ";");
    Serial.print(String() + _minDist+ ":" + _maxDist + ";");
    
    Serial.println("|np"); //don't plot
}


// Settings
void LD06::enableCRC(){
    _useCRC = true;
}

void LD06::disableCRC(){
    _useCRC = false;
}

void LD06::enableFiltering(){
    _useFiltering = true;
}

void LD06::disableFiltering(){
    _useFiltering = false;
}

void LD06::setFilter(FilterType type){
    _filterType = type;
}

void LD06::enableSectoring(){
    _useSectoring = true;
}

void LD06::disableSectoring(){
    _useSectoring = false;
}

void LD06::setIntensityThreshold(int threshold){
    _threshold = threshold;
}

void LD06::setMaxDistance(int maxDist){
    _maxDist = maxDist;
}

void LD06::setMinDistance(int minDist){
    _minDist = minDist;
}

void LD06::setMaxAngle(int maxAngle){
    _maxAngle = maxAngle;
}

void LD06::setMinAngle(int minAngle){
    _minAngle = minAngle;
}

void LD06::setDistanceRange(int minDist, int maxDist){
    _minDist = minDist;
    _maxDist = maxDist;
}

void LD06::setAngleRange(int minAngle, int maxAngle){
    _minAngle = minAngle;
    _maxAngle = maxAngle;
}

void LD06::setCartesianBoundaries(float minx, float miny, float maxx, float maxy){
    _cart_min_x = minx;
    _cart_min_y = miny;
    _cart_max_x = maxx;
    _cart_max_y = maxy;
}
