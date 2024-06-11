#include "simulation.h"

/*

// Peak Detection:
typedef unsigned int uint;
typedef std::deque<double>::iterator deque_iter_double;
class VectorStats {
public:
    VectorStats(deque_iter_double start, deque_iter_double end) {
        this->start = start;
        this->end = end;
        this->compute();
    }

    void compute() {
        double sum = std::accumulate(start, end, 0.0);
        uint slice_size = std::distance(start, end);
        double mean = sum / slice_size;
        std::deque<double> diff(slice_size);
        std::transform(start, end, diff.begin(), [mean](double x) { return x - mean; });
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double std_dev = std::sqrt(sq_sum / slice_size);

        this->m1 = mean;
        this->m2 = std_dev;
    }

    double mean() {
        return m1;
    }

    double standard_deviation() {
        return m2;
    }

private:
    deque_iter_double start;
    deque_iter_double end;
    double m1;
    double m2;
};


std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence) {
    std::unordered_map<std::string, std::deque<double>> output;

    uint n = static_cast<uint>(input.size());
    std::deque<double> signals(n);
    std::deque<double> filtered_input(input);
    std::deque<double> filtered_mean(n);
    std::deque<double> filtered_stddev(n);

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    for (int i = lag; i < n; i++) {
        if (std::abs(input[i] - filtered_mean[i - 1]) > threshold * filtered_stddev[i - 1]) {
            signals[i] = (input[i] > filtered_mean[i - 1]) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }
        VectorStats lag_subvector_stats(filtered_input.begin() + (i - lag), filtered_input.begin() + i);
        filtered_mean[i] = lag_subvector_stats.mean();
        filtered_stddev[i] = lag_subvector_stats.standard_deviation();
    }

    output["signals"] = signals;
    output["filtered_mean"] = filtered_mean;
    output["filtered_stddev"] = filtered_stddev;

    return output["signals"];
};
*/

/*
typedef unsigned int uint;
using deque_iter_double = std::deque<double>::iterator;

class VectorStats {
public:
    VectorStats(deque_iter_double start, deque_iter_double end) {
        this->start = start;
        this->end = end;
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

std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence) {
    uint n = static_cast<uint>(input.size());
    std::deque<double> signals(n, 0.0);
    std::deque<double> filtered_input = input;
    std::deque<double> filtered_mean(n);
    std::deque<double> filtered_stddev(n);

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    double sum = std::accumulate(input.begin(), input.begin() + lag, 0.0);
    double sum_sq = std::inner_product(input.begin(), input.begin() + lag, input.begin(), 0.0);

    for (int i = lag; i < n; i++) {
        double mean = sum / lag;
        double variance = (sum_sq / lag) - (mean * mean);
        double std_dev = std::sqrt(variance);

        if (std::abs(input[i] - mean) > threshold * std_dev) {
            signals[i] = (input[i] > mean) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }

        // Update the sum and sum of squares incrementally
        sum += filtered_input[i] - filtered_input[i - lag];
        sum_sq += filtered_input[i] * filtered_input[i] - filtered_input[i - lag] * filtered_input[i - lag];
    }

    return signals;
}


*/


std::deque<double> z_score_thresholding(std::deque<double> input, int lag, double threshold, double influence) {
    uint n = static_cast<uint>(input.size());
    std::deque<double> signals(n, 0.0);
    std::deque<double> filtered_input = input;
    std::deque<double> filtered_mean(n);
    std::deque<double> filtered_stddev(n);

    VectorStats lag_subvector_stats(input.begin(), input.begin() + lag);
    filtered_mean[lag - 1] = lag_subvector_stats.mean();
    filtered_stddev[lag - 1] = lag_subvector_stats.standard_deviation();

    double sum = std::accumulate(input.begin(), input.begin() + lag, 0.0);
    double sum_sq = std::inner_product(input.begin(), input.begin() + lag, input.begin(), 0.0);

    for (int i = lag; i < n; i++) {
        double mean = sum / lag;
        double variance = (sum_sq / lag) - (mean * mean);
        double std_dev = std::sqrt(variance);

        if (std::abs(input[i] - mean) > threshold * std_dev) {
            signals[i] = (input[i] > mean) ? 1.0 : -1.0;
            filtered_input[i] = influence * input[i] + (1 - influence) * filtered_input[i - 1];
        } else {
            signals[i] = 0.0;
            filtered_input[i] = input[i];
        }

        // Update the sum and sum of squares incrementally
        sum += filtered_input[i] - filtered_input[i - lag];
        sum_sq += filtered_input[i] * filtered_input[i] - filtered_input[i - lag] * filtered_input[i - lag];

        // Update the filtered mean and standard deviation
        filtered_mean[i] = mean;
        filtered_stddev[i] = std_dev;
    }

    return signals;
}



/*
SequenceType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        return SequenceType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == 1) {
                return SequenceType::Rising;
            } else if (states[i-1] == -1) {
                return SequenceType::Falling;
            }
        }
    }

    return SequenceType::Stable;
}

*/

EventType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        return EventType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {

        /*
        if (states[i] == 0) {
            if (states[i-1] == 1) {
                return EventType::Bump;
            } else if (states[i-1] == -1) {
                return EventType::Pothole;
            }
        }
        */
        
        if(states[i] == 1){
            if((states[i - 1] == 0) || (states[i - 1] == -1)){
                return EventType::Bump;
            }
        }

        if(states[i] == -1){
            if((states[i - 1] == 0) || (states[i - 1] == 1)){
                return EventType::Pothole;
            }
        }


    }

    return EventType::Stable;
}





EventType getStateChangeBump(const std::deque<double>& states) {
    if (states.size() < 2) {
        return EventType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == 1) {
                return EventType::Bump;
            }
        }
    }

    return EventType::Stable;
}

EventType getStateChangePothole(const std::deque<double>& states) {
    if (states.size() < 2) {
        return EventType::Stable;
    }

    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i] == 0) {
            if (states[i-1] == -1) {
                return EventType::Pothole;
            }
        }
    }

    return EventType::Stable;
}





inline double compoundVector(double x, double y, double z){
    return std::sqrt(x*x + y*y + z*z);
}

inline void rotateAll(double rollAngle, double pitchAngle, double x, double y, double z, double* x_rotated, double* y_rotated, double* z_rotated){
    *x_rotated =  x*std::cos(pitchAngle)                                             + z*std::sin(pitchAngle);
    *y_rotated = -x*std::sin(pitchAngle)*std::sin(rollAngle) + y*std::cos(rollAngle) + z*std::cos(pitchAngle)*std::sin(rollAngle);
    *z_rotated = -x*std::sin(pitchAngle)*std::cos(rollAngle) - y*std::sin(rollAngle) + z*std::cos(rollAngle)*std::cos(pitchAngle);  
}


void complementaryFilter(double ax, double ay, double az, double gr, double gp, double gy, double* rollAngle, double* pitchAngle) {
    static double accel_angle[2] = {0.0, 0.0};
    static double gyro_angle[3] = {0.0, 0.0, 0.0};
    static double angle[3] = {0.0, 0.0, 0.0};
    static bool first_run = true;
    static bool calc_yaw = true;
    double dt{ 0.0 };

    // X (roll) axis
    accel_angle[0] = std::atan2(az, ay) * radiansToDegrees - 90.0; // Calculate the angle with z and y, convert to degrees, and subtract 90 degrees to rotate
    gyro_angle[0] = angle[0] + gr * dt; // Use roll axis (X axis)

    // Y (pitch) axis
    accel_angle[1] = std::atan2(az, ax) * radiansToDegrees - 90.0; // Calculate the angle with z and x, convert to degrees, and subtract 90 degrees to rotate
    gyro_angle[1] = angle[1] + gp * dt; // Use pitch axis (Y axis)

    // Z (yaw) axis
    if (calc_yaw) {
        gyro_angle[2] = angle[2] + gy * dt; // Use yaw axis (Z axis)
    }

    if (first_run) { // Set the gyroscope angle reference point if this is the first function run
        for (int i = 0; i <= 1; i++) {
            gyro_angle[i] = accel_angle[i]; // Start off with angle from accelerometer (absolute angle since gyroscope is relative)
        }
        gyro_angle[2] = 0; // Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
        first_run = false;
    }

    double asum = std::fabs(ax) + std::fabs(ay) + std::fabs(az); // Calculate the sum of the accelerations
    double gsum = std::fabs(gr) + std::fabs(gp) + std::fabs(gy); // Calculate the sum of the gyro readings

    for (int i = 0; i <= 1; i++) { // Loop through roll and pitch axes
        if (std::fabs(gyro_angle[i] - accel_angle[i]) > 5) { // Correct for very large drift (or incorrect measurement of gyroscope by longer loop time)
            gyro_angle[i] = accel_angle[i];
        }
        // Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
        if (asum > 0.1 && asum < 3 && gsum > 0.3) { // Check that the movement is not very high (therefore providing inaccurate angles)
            angle[i] = (1.0 - tau) * (gyro_angle[i]) + (tau) * (accel_angle[i]); // Calculate the angle using a complementary filter
        }
        else if (gsum > 0.3) { // Use the gyroscope angle if the acceleration is high
            angle[i] = gyro_angle[i];
        }
        else if (gsum <= 0.3) { // Use accelerometer angle if not much movement
            angle[i] = accel_angle[i];
        }
    }

    // The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
    if (calc_yaw) { // Only calculate the angle when we want it to prevent large drift
        angle[2] = gyro_angle[2];
    }
    else {
        angle[2] = 0;
        gyro_angle[2] = 0;
    }

    *rollAngle = angle[0];
    *pitchAngle = angle[1];
}


inline void AppendDeque(std::deque<double> &target, std::deque<double> source)
{
    for(long unsigned int i = 0; i < source.size(); i++)
    {
        target.push_back(source.at(i));
    }
}

std::string joinPath(const std::string& directory, const std::string& file) {
    std::string path = directory;
    if (directory.back() != '/' && directory.back() != '\\') {
        path += '\\'; // Add a slash if not already present
    }
    path += file;
    return path;
}

// Function to write CSV header with variable names
void writeCSVHeader(std::ofstream& outputFile, const std::vector<std::string>& variableNames) {
    for (size_t i = 0; i < variableNames.size(); ++i) {
        outputFile << variableNames[i];
        if (i < variableNames.size() - 1) {
            outputFile << ",";
        }
    }
    outputFile << std::endl;
}

// Function to check if file is empty
inline bool isFileEmpty(const std::string& fileName) {
    struct stat fileStat;
    if (stat(fileName.c_str(), &fileStat) != 0) {
        // File does not exist, consider it as empty
        return true;
    }
    return fileStat.st_size == 0;
}

std::deque<int> readFileIntoDeque(std::string filename) {
    std::deque<int> numbers;
    std::ifstream inputFile(filename);

    if (!inputFile.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return numbers; // Return an empty deque if the file cannot be opened
    }

    int number;
    while (inputFile >> number) { // Read integers until the end of the file
        numbers.push_back(number); // Add each number to the deque
    }

    inputFile.close();
    return numbers;
}

template<typename T>
void saveDequeIntoFile(const std::deque<T>& deque, std::string name, const std::string directoryPath){
    
    std::string outputFileName = directoryPath + "/" + name + ".txt";
    std::ofstream outputFile(outputFileName);

    if (!outputFile.is_open()) {
        std::cerr << "Failed to open " << outputFileName << std::endl;
        return;
    }

    for (const auto& signal : deque) {
        outputFile << signal << "\n";
    }

    outputFile.close();
}


std::vector<int> getStateChangeIndices(const std::deque<int>& deque) {
    std::vector<int> indices;
    for (size_t i = 1; i < deque.size(); ++i) {
        if (deque[i] == 1 && deque[i - 1] == 0) {
            indices.push_back(i + 1);
        }
    }
    return indices;
}



inline bool isValidDetection(const std::deque<int>& detectionDeque, int index, int range, int whichHazard) {
    int start = std::max(0, static_cast<int>(index) - range);
    int end = std::min(static_cast<int>(detectionDeque.size()) - 1, static_cast<int>(index) + range);

    int stateChangesInRange{ 0 };
    int stateChangesOutRange{ 0 };

    // Adjusted loop to iterate only within the specified range
    for (int i = start + 1; i <= end; ++i) { // Start from start+1 because we're comparing current element with its predecessor
        // Check for state changes within the specified range
        if (((detectionDeque[i] == whichHazard) && (detectionDeque[i - 1] == 0))) {
            stateChangesInRange++;
        }
        else{
            stateChangesOutRange++;
        }
    }
    return (stateChangesInRange == 1);
}

inline int countStateChangesFrom1to0(const std::deque<int>& stateDeque) {
    int stateChangesCount = 0;

    // Iterate through the deque, starting from the second element
    for (size_t i = 1; i < stateDeque.size(); ++i) {
        // Check if the current element is 0 and the previous element was 1
        if (stateDeque[i] == 0 && stateDeque[i - 1] == 1) {
            stateChangesCount++;
        }
    }

    return stateChangesCount;
}


inline int countStateChangesFromMinus1to0(const std::deque<int>& stateDeque) {
    int stateChangesCount = 0;

    // Iterate through the deque, starting from the second element
    for (size_t i = 1; i < stateDeque.size(); ++i) {
        // Check if the current element is 0 and the previous element was 1
        if (stateDeque[i] == 0 && stateDeque[i - 1] == -1) {
            stateChangesCount++;
        }
    }

    return stateChangesCount;
}

std::deque<int> convertDoubleDequeToIntDeque(const std::deque<double>& doubleDeque) {
    std::deque<int> intDeque;
    for (const auto& value : doubleDeque) {
        intDeque.push_back(static_cast<int>(value));
    }
    return intDeque;
}



template<typename T>
void appendIfNotEmpty(const T& filterConfigured, std::deque<double>& output1, std::deque<double>& output2) {
    if (filterConfigured.getCompletedDataSize() > 0) {
        std::deque<double> completedData = filterConfigured.getCompletedData();
        AppendDeque(output1, completedData);
        AppendDeque(output2, completedData);
    }
}

void removeExcessSamples(std::deque<double>& outData, int size) {
    if (outData.size() > size) {
        unsigned int remove = static_cast<unsigned int>(outData.size() - size);
        outData.erase(outData.begin(), outData.begin() + remove);
    }
}

void applyZScoreThresholding(const std::deque<double>& outData, std::deque<double>& sequenceDeque, int size, unsigned int lag, double z_score_threshold, double influence){
    if (outData.size() == size) {
        sequenceDeque = z_score_thresholding(outData, lag, z_score_threshold, influence);
    }
}


void determineState(const std::deque<double>& sequenceDeque, std::deque<int>& stateDeque){
    if (getStateChange(sequenceDeque) == EventType::Bump) {
        stateDeque.push_back(1);
    }
    if (getStateChange(sequenceDeque) == EventType::Pothole) {
        stateDeque.push_back(-1);
    }
    if (getStateChange(sequenceDeque) == EventType::Stable) {
        stateDeque.push_back(0);
    }
}


void determineStateBump(const std::deque<double>& sequenceDeque, std::deque<int>& stateDeque){
    if (getStateChangeBump(sequenceDeque) == EventType::Bump) {
        stateDeque.push_back(1);
    }
    if (getStateChangeBump(sequenceDeque) == EventType::Stable) {
        stateDeque.push_back(0);
    }
}

void determineStatePothole(const std::deque<double>& sequenceDeque, std::deque<int>& stateDeque){
    if (getStateChangePothole(sequenceDeque) == EventType::Pothole) {
        stateDeque.push_back(-1);
    }
    if (getStateChangePothole(sequenceDeque) == EventType::Stable) {
        stateDeque.push_back(0);
    }
}





// SIMULATION FUNCTION BEGIN
void simulation(const std::vector<SensorData>& sensorData,

                   unsigned int lag, double z_score_threshold, double influence,

                   const std::vector<int> bumpIndices, const std::vector<int> potholeIndices,

                   double actFiltThres, double actFiltPosCoef, double actFiltNegCoef,

                   std::vector<std::string>& results,
                   
                   double iirAlpha,
                   
                   std::string directoryPath) {
    

    auto startInits = std::chrono::high_resolution_clock::now();

    // Initialize Active Filters:
    int activeFilterWindowSize{ 50 };
    int activeFilterOverlapSize{ 35 };

    
    ActiveFilter actFilter;
    double activeFilterThreshold{ actFiltThres };
    double activeFilterPositiveCoef{ actFiltPosCoef };
    double activeFilterNegativeCoef{ actFiltNegCoef };

    actFilter.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilter.setThreshold(activeFilterThreshold);
    actFilter.setCoefficients(activeFilterPositiveCoef, activeFilterNegativeCoef);
    
    
    
    /*
    ActiveFilter actFilterBumpConfigured;
    double bumpActiveFilterThreshold{ 0.18 };
    double bumpActiveFilterPositiveCoef{ 1.3 };
    double bumpActiveFilterNegativeCoef{ 0.9 }; 
    // Parameters for the Bump Active Filter
    actFilterBumpConfigured.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilterBumpConfigured.setThreshold(bumpActiveFilterThreshold);
    actFilterBumpConfigured.setCoefficients(bumpActiveFilterPositiveCoef, bumpActiveFilterNegativeCoef);

    ActiveFilter actFilterPotholeConfigured;
    double potholeActiveFilterThreshold{ 0.13 };
    double potholeActiveFilterPositiveCoef{ 1.8 };
    double potholeActiveFilterNegativeCoef{ 0.1 }; 
    // Parameters for the Pothole Active Filter
    actFilterPotholeConfigured.setWindowParameters(activeFilterWindowSize, activeFilterOverlapSize);
    actFilterPotholeConfigured.setThreshold(potholeActiveFilterThreshold);
    actFilterPotholeConfigured.setCoefficients(potholeActiveFilterPositiveCoef, potholeActiveFilterNegativeCoef);
    */
    
    
    
    

    // ------------------------------------------------------------------------------------------------------------------------

    // Initialize IIR Filter

    ThreeAxisIIR iirFiltAccel;
    ThreeAxisIIR iirFiltGyro;
    ThreeAxisIIR_Init(&iirFiltAccel, iirAlpha);
    ThreeAxisIIR_Init(&iirFiltGyro, iirAlpha);
    
    
    // Initialize FIR Filter Coefficients:
    
    /*
    std::deque<double> filterCoefficients = designFIRFilter(numberOfTaps, cutoffFrequency, samplingRate);
    FIRFilter FIRfilterAccel(filterCoefficients);
    FIRFilter FIRfilterGyro(filterCoefficients);
    */

    // Variables to store the filtered/rotated acceleration values
    double ax_filtered{ 0.0 }, ay_filtered{ 0.0 }, az_filtered{ 0.0 };
    double ax_rotated{ 0.0 }, ay_rotated{ 0.0 }, az_rotated{ 0.0 };

    // Variables to store the filtered/rotated gyroscope values
    double gr_filtered{ 0.0 }, gp_filtered{ 0.0 }, gy_filtered{ 0.0 };
    double gr_rotated{ 0.0 }, gp_rotated{ 0.0 }, gy_rotated{ 0.0 };

    // Variables to store the angles:
    double pitchAngle{ 0.0 };
    double rollAngle{ 0.0 };

    // Variable to store the compound acceleration vector:
    double compoundAccelerationVector{ 0.0 };

    // Deque to store the filtered/unfiltered acceleration vector:

    std::deque<double> filteredVectorDeque;
    filteredVectorDeque.clear();

    std::deque<double> unfilteredVectorDeque;
    unfilteredVectorDeque.clear();
    
    

    int sampleNumber{ 0 };
    const unsigned int wholeDequeSize{ 150 };

    unsigned int removeBumpSamples{ 0 };
    unsigned int removePotholeSamples{ 0 };

    unsigned int removeSamples{ 0 };


    std::deque<double> outData;
    outData.clear();

    
    std::deque<double> outBumpData;
    outBumpData.clear();
    std::deque<double> outPotholeData;
    outPotholeData.clear();
    
    
    std::deque<double> sequenceDeque;
    sequenceDeque.clear();

    
    std::deque<double> sequenceBumpDeque;
    sequenceBumpDeque.clear();
    std::deque<double> sequencePotholeDeque;
    sequencePotholeDeque.clear();
    
    
    std::deque<int> stateDeque;
    stateDeque.clear();

    
    std::deque<int> stateBumpDeque;
    stateBumpDeque.clear();
    std::deque<int> statePotholeDeque;
    statePotholeDeque.clear();
    


    std::string line;

    std::cout << "Starting Simulation for lag=" << lag << ", threshold=" << z_score_threshold << ", influence=" << influence << std::endl;

    int numberOfCorrectBumpDetections{ 0 };
    int numberOfCorrectPotholeDetections{ 0 };

    int numberOfIncorrectBumpDetections{ 0 };
    int numberOfIncorrectPotholeDetections{ 0 };

    std::deque<double> activeFilterOutput;
    activeFilterOutput.clear();

    std::deque<double> activeFilterBumpOutput;
    activeFilterBumpOutput.clear();

    std::deque<double> activeFilterPotholeOutput;
    activeFilterPotholeOutput.clear();

    auto endInits = std::chrono::high_resolution_clock::now();
    auto initsDuration = std::chrono::duration_cast<std::chrono::microseconds>(endInits - startInits).count();
    
    // auto start = std::chrono::high_resolution_clock::now();

    /*
    int64_t durationIIR{0};
    int64_t durationCompFilt{0};
    int64_t durationRotation{0};
    int64_t durationCAV{0};
    int64_t durationFeedData{0};
    int64_t durationAppend{0};
    int64_t durationRemoveSamples{0};
    int64_t durationZScore{0};
    int64_t durationDetermineState{0};
    */

    auto startSim = std::chrono::high_resolution_clock::now();

    for(const auto& data : sensorData){

        //unfilteredVectorDeque.push_back(compoundVector(data.ax, data.ay, data.az));

        //ThreeAxisFIR_Update(FIRfilterAccel, ax, ay, az, ax_filtered, ay_filtered, az_filtered);
        //ThreeAxisFIR_Update(FIRfilterGyro, gr, gp, gy, gr_filtered, gp_filtered, gy_filtered);

        ThreeAxisIIR_Update(&iirFiltAccel, data.ax, data.ay, data.az, &ax_filtered, &ay_filtered, &az_filtered);
        ThreeAxisIIR_Update(&iirFiltGyro, data.gr, data.gp, data.gy, &gr_filtered, &gp_filtered, &gy_filtered);

        //filteredVectorDeque.push_back(compoundVector(ax_filtered, ay_filtered, az_filtered));

        complementaryFilter(ax_filtered, ay_filtered, az_filtered, gr_filtered, gp_filtered, gy_filtered, &rollAngle, &pitchAngle);

        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, ax_filtered, ay_filtered, az_filtered, &ax_rotated, &ay_rotated, &az_rotated);

        compoundAccelerationVector = compoundVector(ax_rotated, ay_rotated, az_rotated);

        filteredVectorDeque.push_back(compoundAccelerationVector);

        actFilter.feedData(compoundAccelerationVector);
        //actFilterBumpConfigured.feedData(compoundAccelerationVector);
        //actFilterPotholeConfigured.feedData(compoundAccelerationVector);

        appendIfNotEmpty(actFilter, outData, activeFilterOutput);
        //appendIfNotEmpty(actFilterBumpConfigured, outBumpData, activeFilterBumpOutput);
        //appendIfNotEmpty(actFilterPotholeConfigured, outPotholeData, activeFilterPotholeOutput);

        removeExcessSamples(outData, wholeDequeSize);
        //removeExcessSamples(outBumpData, wholeDequeSize);
        //removeExcessSamples(outPotholeData, wholeDequeSize);

        applyZScoreThresholding(outData, sequenceDeque, wholeDequeSize, lag, z_score_threshold, influence);
        
        //applyZScoreThresholding(outBumpData, sequenceBumpDeque, wholeDequeSize, lag, 6, influence);
        //applyZScoreThresholding(outPotholeData, sequencePotholeDeque, wholeDequeSize, lag, 15, influence);

        determineState(sequenceDeque, stateDeque);

        //determineStateBump(sequenceBumpDeque, stateBumpDeque);
        //determineStatePothole(sequencePotholeDeque, statePotholeDeque);

        sampleNumber++;
    }

    saveDequeIntoFile(filteredVectorDeque, "compound_acceleration_vector", directoryPath);
    saveDequeIntoFile(stateDeque, "simulation_runtime_state_deque", directoryPath);
    saveDequeIntoFile(sequenceDeque, "simulation_runtime_sequence_deque", directoryPath);
    saveDequeIntoFile(activeFilterOutput, "active_filter_output", directoryPath);


    auto endSim = std::chrono::high_resolution_clock::now();
    auto simDuration = std::chrono::duration_cast<std::chrono::microseconds>(endSim - startSim).count();


    /*
    std::cout << "\ndurationIIR:            " << std::setw(20) << durationIIR
              << "\ndurationCompFilt:       " << std::setw(20) << durationCompFilt
              << "\ndurationRotation:       " << std::setw(20) << durationRotation
              << "\ndurationCAV:            " << std::setw(20) << durationCAV
              << "\ndurationFeedData:       " << std::setw(20) << durationFeedData
              << "\ndurationAppend:         " << std::setw(20) << durationAppend
              << "\ndurationRemoveSamples:  " << std::setw(20) << durationRemoveSamples
              << "\ndurationZScore:         " << std::setw(20) << durationZScore
              << "\ndurationDetermineState: " << std::setw(20) << durationDetermineState
              << std::endl;
    */
    

    // ---------------- Validation: ----------------------------------------------

    unsigned int range{ 100 };

    /*
    // std::cout << "for bumps: " << std::endl;
    for (const auto& index : bumpIndices) {
        bool validBumpDetection = isValidDetection(stateBumpDeque, index, range, 1);
    
        if(validBumpDetection){
            ++numberOfCorrectBumpDetections;
        }
        else{
            ++numberOfIncorrectBumpDetections;
        }
    }


    // std::cout << "for potholes: " << std::endl;
    for (const auto& index : potholeIndices) {
        bool validPotholeDetection = isValidDetection(statePotholeDeque, index, range, -1);
        
        if(validPotholeDetection){
            ++numberOfCorrectPotholeDetections;
        }
        else{
            ++numberOfIncorrectPotholeDetections;
        }
    }
    
    */

    auto startValidation = std::chrono::high_resolution_clock::now();

    //std::cout << "for bumps: " << std::endl;
    for (const auto& index : bumpIndices) {
        bool validBumpDetection = isValidDetection(stateDeque, index, range, 1);
    
        if(validBumpDetection){
            ++numberOfCorrectBumpDetections;
        }
        else{
            ++numberOfIncorrectBumpDetections;
        }
    }

    //std::cout << "for potholes: " << std::endl;
    for (const auto& index : potholeIndices) {
        bool validPotholeDetection = isValidDetection(stateDeque, index, range, -1);
        
        if(validPotholeDetection){
            ++numberOfCorrectPotholeDetections;
        }
        else{
            ++numberOfIncorrectPotholeDetections;
        }
    }

    auto endValidation = std::chrono::high_resolution_clock::now();
    auto validationDuration = std::chrono::duration_cast<std::chrono::microseconds>(endValidation - startValidation).count();
    
    
    // ---------------- Save Variables Used in Runtime: --------------------------

    auto startCSV = std::chrono::high_resolution_clock::now(); 

    // Collect results as a string
    std::ostringstream resultStream;
    resultStream << std::to_string(lag) << ", " << std::to_string(z_score_threshold) << ", " << std::to_string(influence)
                 << ", " << std::to_string(bumpIndices.size()) << ", " << std::to_string(numberOfCorrectBumpDetections) << ", " << std::to_string(numberOfIncorrectBumpDetections)
                 << ", " << std::to_string(potholeIndices.size()) << ", " << std::to_string(numberOfCorrectPotholeDetections) << ", " << std::to_string(numberOfIncorrectPotholeDetections)
                 << ", " << std::to_string((static_cast<double>(numberOfCorrectBumpDetections)/bumpIndices.size())*100)
                 << ", " << std::to_string((static_cast<double>(numberOfCorrectPotholeDetections)/potholeIndices.size())*100)
                 << ", " << std::to_string(countStateChangesFrom1to0(stateDeque) - numberOfCorrectBumpDetections)
                 << ", " << std::to_string(countStateChangesFromMinus1to0(stateDeque) - numberOfCorrectPotholeDetections)
                 //<< ", " << std::to_string(countStateChangesFrom1to0(stateBumpDeque) - numberOfCorrectBumpDetections)
                 //<< ", " << std::to_string(countStateChangesFromMinus1to0(statePotholeDeque) - numberOfCorrectPotholeDetections)

                 << ", " << std::to_string(activeFilterThreshold)
                 << ", " << std::to_string(activeFilterPositiveCoef)
                 << ", " << std::to_string(activeFilterNegativeCoef)

                 //<< ", " << std::to_string(bumpActiveFilterThreshold)
                 //<< ", " << std::to_string(bumpActiveFilterPositiveCoef)
                 //<< ", " << std::to_string(bumpActiveFilterNegativeCoef)

                 //<< ", " << std::to_string(potholeActiveFilterThreshold)
                 //<< ", " << std::to_string(potholeActiveFilterPositiveCoef)
                 //<< ", " << std::to_string(potholeActiveFilterNegativeCoef)

                 << std::endl;

    // Store the result string in the results vector
    results.push_back(resultStream.str());

    auto endCSV = std::chrono::high_resolution_clock::now();
    auto CSVDuration = std::chrono::duration_cast<std::chrono::microseconds>(endCSV - startCSV).count();

    std::cout << "Simulation for lag=" << lag << ", threshold=" << z_score_threshold << ", influence=" << influence << " completed." << std::endl;
    std::cout << "\ninitialization duration:   " << std::setw(20) << initsDuration
              << "\nsimulation duration:       " << std::setw(20) << simDuration
              << "\nvalidation duration:       " << std::setw(20) << validationDuration
              << "\ncsv duration:              " << std::setw(20) << CSVDuration
              << std::endl;
}
// SIMULATION FUNCTION END















void runSimulations(const std::vector<SensorData>& sensorData, 
                    const std::vector<unsigned int>& lags, const std::vector<double>& z_score_thresholds, const std::vector<double>& influences, 
                    const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices, const std::vector<double>& iirFilterAlpha,
                    const std::vector<double>& actFiltThres, const std::vector<double>& activeFiltPosCoefs, const std::vector<double>& activeFiltNegCoefs,
                    const std::string directoryPath){


    // Prepare to collect results
    std::vector<std::string> results;
    results.reserve(lags.size() * z_score_thresholds.size() * influences.size() * actFiltThres.size() * activeFiltPosCoefs.size() * activeFiltNegCoefs.size() * iirFilterAlpha.size());



    // Loop through different configurations
    for (unsigned int lag : lags) {
        for (double z_score_threshold : z_score_thresholds) {
            for (double influence : influences) {

                for(double threshold : actFiltThres){
                    for(double posCoef : activeFiltPosCoefs){
                        for(double negCoef : activeFiltNegCoefs){

                            for(double alpha : iirFilterAlpha)
                                simulation(sensorData, lag, z_score_threshold, influence, bumpIndices, potholeIndices, threshold, posCoef, negCoef, results, alpha, directoryPath);

                        }
                    }
                }
            }
        }
    }

    // Write results to the CSV file once
    std::string testResultsFileName = "test_huseyinabi.csv";
    std::ofstream testResultsOutputFile(testResultsFileName, std::ios::app);

    if (!testResultsOutputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    std::vector<std::string> variableNames = {"lag", "zScoreThreshold", "influence", 
                                              "bumpPressCount", "correctBumpDetections", "incorrectBumpDetections", 
                                              "potholePressCount", "correctPotholeDetections", "incorrectPotholeDetections", 
                                              "bumpDetectionSuccess", "potholeDetectionSuccess", 
                                              "bumpFalsePositives", "potholeFalsePositives",
                                              "activeFilterThreshold", "activeFilterPosCoef", "activeFilterNegCoef", 
                                              "activeFilterThresholdBump", "activeFilterPosCoefBump", "activeFilterNegCoefBump",
                                              "activeFilterThresholdPothole", "activeFilterPosCoefPothole", "activeFilterNegCoefPothole"};

    if (isFileEmpty(testResultsFileName)) {
        writeCSVHeader(testResultsOutputFile, variableNames);
    }

    for (const std::string& result : results) {
        testResultsOutputFile << result;
    }

    testResultsOutputFile.close();
}

