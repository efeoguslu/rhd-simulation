#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <future>
#include <sstream>
#include <deque>
#include <string>
#include <algorithm>
#include <cmath>
#include <stack>
#include <unordered_map>
#include <iterator>
#include <numeric>
#include <iomanip>

#include <sys/stat.h>

#include "filters.h"
#include "sequence.h" 

// -----------------------------------------------------------------------------------------------------------------------------------------


const unsigned int lag{ 50 };
const double z_score_threshold{ 10.0 };
const double influence{ 0.25 };

// IIR Filter:
const double filterAlpha{ 0.9 };

// Finding Angles for Rotation:
const double radiansToDegrees{ 57.2957795 };
const double degreesToRadians{ 0.0174532925 };


const double tau{ 0.05 };

// FIR Filter:
const double samplingRate{ 75 };
const int numberOfTaps{ 15 };
const double cutoffFrequency{ 4 };

#define CSV_LINE(variable) #variable

// Struct to hold sensor data
struct SensorData{
    double ax, ay, az;
    double gr, gp, gy;
};


enum class EventType {
    Bump,
    Pothole,
    Stable,
    None
};

// -----------------------------------------------------------------------------------------------------------------------------------------


using deque_iter_double = std::deque<double>::iterator;
using uint = unsigned int;

class VectorStats {
public:
    VectorStats(deque_iter_double start_iterator, deque_iter_double end_iterator) {
        this->start = start_iterator;
        this->end = end_iterator;
        this->compute();
    }

    void compute() {
        double sum = std::accumulate(start, end, 0.0);
        uint slice_size = std::distance(start, end);
        double mean = sum / slice_size;
        std::vector<double> diff(slice_size);
        std::transform(start, end, diff.begin(), [mean](double x) { return x - mean; });
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double std_dev = std::sqrt(sq_sum / slice_size);

        this->m1 = mean;
        this->m2 = std_dev;
    }

    double mean() const {
        return m1;
    }

    double standard_deviation() const {
        return m2;
    }

private:
    deque_iter_double start;
    deque_iter_double end;
    double m1;
    double m2;
};


// -----------------------------------------------------------------------------------------------------------------------------------------

std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence);

//SequenceType getStateChange(const std::deque<double>& states);
EventType getStateChangePothole(const std::deque<double>& states);
EventType getStateChangeBump(const std::deque<double>& states);

inline double compoundVector(double x, double y, double z);
inline void rotateAll(double rollAngle, double pitchAngle, double x, double y, double z, double* x_rotated, double* y_rotated, double* z_rotated);
void complementaryFilter(double ax, double ay, double az, double gr, double gp, double gy, double* rollAngle, double* pitchAngle);
inline void AppendDeque(std::deque<double> &target, std::deque<double> source);
std::string joinPath(const std::string& directory, const std::string& file);
void writeCSVHeader(std::ofstream& outputFile, const std::vector<std::string>& variableNames);
inline bool isFileEmpty(const std::string& fileName);
std::deque<int> readFileIntoDeque(std::string filename);
std::vector<int> getStateChangeIndices(const std::deque<int>& deque);
inline bool isValidDetection(const std::deque<int>& detectionDeque, int index, int range, int whichHazard);
inline int countStateChangesFrom1to0(const std::deque<int>& stateDeque);
inline int countStateChangesFromMinus1to0(const std::deque<int>& stateDeque);
std::deque<int> convertDoubleDequeToIntDeque(const std::deque<double>& doubleDeque);

template<typename T>
void saveDequeIntoFile(const std::deque<T>& deque, std::string name);


template<typename T>
void appendIfNotEmpty(const T& filterConfigured, std::deque<double>& output1, std::deque<double>& output2);

void removeExcessSamples(std::deque<double>& outData, int size);
void applyZScoreThresholding(const std::deque<double>& outData, std::deque<double>& sequenceDeque, int size, unsigned int lag, double z_score_threshold, double influence);
void determineState(const std::deque<double>& sequenceDeque, std::deque<int>& stateDeque);

/*
template<typename T>
void populateVector(std::vector<T>& vec, T start, T end, T step);
*/


void simulation(const std::vector<SensorData>& sensorData, 
                   unsigned int lag, double z_score_threshold, double influence, 
                   const std::vector<int> bumpIndices, const std::vector<int> potholeIndices, 
                   double actFiltThres, double actFiltPosCoef, double actFiltNegCoef,
                   std::vector<std::string>& results);






// Overloads:

void runSimulations(const std::vector<SensorData>& sensorData, 
                    const std::vector<unsigned int>& lags, const std::vector<double>& z_score_thresholds, const std::vector<double>& influences, 
                    const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices,
                    const std::vector<double>& actFiltThres, const std::vector<double>& activeFiltPosCoefs, const std::vector<double>& activeFiltNegCoefs);




void runSimulations(const std::vector<SensorData>& sensorData, 
                    const std::vector<unsigned int>& lags, const std::vector<double>& z_score_thresholds, const std::vector<double>& influences, 
                    const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices,
                    double actFiltThres, double activeFiltPosCoefs, double activeFiltNegCoefs);



void runSimulations(const std::vector<SensorData>& sensorData, 
                    unsigned int lags, double z_score_thresholds, double influences, 
                    const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices,
                    const std::vector<double>& actFiltThres, const std::vector<double>& activeFiltPosCoefs, const std::vector<double>& activeFiltNegCoefs);


void runSimulations(const std::vector<SensorData>& sensorData, 
                    unsigned int lags, double z_score_thresholds, double influences, 
                    const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices,
                    double actFiltThres, double activeFiltPosCoefs, double activeFiltNegCoefs);


