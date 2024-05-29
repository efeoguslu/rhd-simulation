#include <fstream>
#include <sstream>
#include <iostream>
#include <deque>
#include <algorithm>
#include <stack>
#include <mutex>
#include <unordered_map>
#include <iterator>
#include <numeric>
#include <cmath>

#include <sys/stat.h>

#include "filters.h"
#include "sequence.h" 

typedef unsigned int uint;
typedef std::deque<double>::iterator deque_iter_double;

const unsigned int lag{ 50 };
const double threshold{ 10.0 };
const double influence{ 0.25 };

const double filterAlpha{ 0.9 };

const double radiansToDegrees{ 57.2957795 };
const double degreesToRadians{ 0.0174532925 };

double dt{ 0.0 };
const double tau{ 0.05 };

#define CSV_LINE(variable) #variable

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

/*

SequenceType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        // Not enough data to detect state changes.
        return SequenceType::Stable;
    }
    
    double currentState = states[0];
    double nextState = states[1]; // Initialize with the second state to handle transitions starting from the first sample
    
    for(size_t i = 1; i < states.size(); ++i){
        // Update nextState if the current state changes
        if(currentState != states[i]){
            nextState = states[i];
        }
        
        // Check for transitions considering intermediate states
        if((currentState == 0 && nextState == 1) || (currentState == -1 && nextState == 1)){
            //std::cout << "Detected rising sequence starting at: " << i << "\n";
            return SequenceType::Rising;
        }
        if((currentState == 0 && nextState == -1) || (currentState == 1 && nextState == -1)){
            //std::cout << "Detected falling sequence starting at: " << i << "\n";
            return SequenceType::Falling;
        }
        
        // Move to the next state
        currentState = nextState;
    }
    
    // If no clear rising or falling sequence is detected, consider it stable
    return SequenceType::Stable;
}
*/


SequenceType getStateChange(const std::deque<double>& states) {
    if (states.size() < 2) {
        return SequenceType::Stable;
    }

    enum State { STABLE, RISING, FALLING, UNKNOWN } currentState = STABLE;

    for (size_t i = 1; i < states.size(); ++i) {
        if (currentState == STABLE) {
            if (states[i-1] == 0) {
                if (states[i] == 1) {
                    currentState = RISING;
                } else if (states[i] == -1) {
                    currentState = FALLING;
                }
            }
        } else if (currentState == RISING) {
            if (states[i-1] == 1) {
                if (states[i] == 0) {
                    return SequenceType::Rising;
                } else if (states[i] == -1) {
                    currentState = FALLING;
                }
            }
        } else if (currentState == FALLING) {
            if (states[i-1] == -1) {
                if (states[i] == 0) {
                    return SequenceType::Falling;
                } else if (states[i] == 1) {
                    currentState = RISING;
                }
            }
        }
    }

    return SequenceType::Stable;
}



inline double compoundVector(double x, double y, double z){
    return std::sqrt(x*x + y*y + z*z);
}

void rotateAll(double rollAngle, double pitchAngle, double x, double y, double z, double* x_rotated, double* y_rotated, double* z_rotated){
    *x_rotated =  x*std::cos(pitchAngle)                                             + z*std::sin(pitchAngle);
    *y_rotated = -x*std::sin(pitchAngle)*std::sin(rollAngle) + y*std::cos(rollAngle) + z*std::cos(pitchAngle)*std::sin(rollAngle);
    *z_rotated = -x*std::sin(pitchAngle)*std::cos(rollAngle) - y*std::sin(rollAngle) + z*std::cos(rollAngle)*std::cos(pitchAngle);  
}


// dt işini çöz:

void complementaryFilter(double ax, double ay, double az, double gr, double gp, double gy, double* rollAngle, double* pitchAngle) {
    static double accel_angle[2] = {0.0, 0.0};
    static double gyro_angle[3] = {0.0, 0.0, 0.0};
    static double angle[3] = {0.0, 0.0, 0.0};
    static bool first_run = true;
    static bool calc_yaw = true;

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


void AppendDeque(std::deque<double> &target, std::deque<double> source)
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
bool isFileEmpty(const std::string& fileName) {
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


std::vector<int> getStateChangeIndices(const std::deque<int>& deque) {
    std::vector<int> indices;
    for (size_t i = 1; i < deque.size(); ++i) {
        if (deque[i] == 1 && deque[i - 1] == 0) {
            indices.push_back(i + 1);
        }
    }
    return indices;
}


bool isValidDetection(const std::deque<int>& detectionDeque, int index, int range, int whichHazard) {
    int start = std::max(0, static_cast<int>(index) - range);
    int end = std::min(static_cast<int>(detectionDeque.size()) - 1, static_cast<int>(index) + range);

    int stateChangesInRange{ 0 };
    int stateChangesOutsideRange{ 0 };

    // Iterate through the entire detection deque
    for (int i = 1; i < detectionDeque.size(); ++i) {
        // Check for state changes
        if ((detectionDeque[i] == whichHazard && detectionDeque[i - 1] == 0)) {
            // Determine if the state change is within the specified range
            if (i >= start && i <= end) {
                stateChangesInRange++;
            } else {
                stateChangesOutsideRange++;
            }
        }
    }

    // return (stateChanges == 1);
    // std::cout << "state changes in correct range: " << stateChangesInRange << std::endl;
    // std::cout << "state changes in Incorrect range: " << stateChangesOutsideRange << std::endl;

    return ((stateChangesInRange == 1) && (stateChangesOutsideRange < 30));
}





std::deque<int> convertDoubleDequeToIntDeque(const std::deque<double>& doubleDeque) {
    std::deque<int> intDeque;
    for (const auto& value : doubleDeque) {
        intDeque.push_back(static_cast<int>(value));
    }
    return intDeque;
}

/*
bool isValidDetection3(const std::deque<int>& detectionDeque, int index, int range, int whichHazard) {
    int start = std::max(0, static_cast<int>(index) - range);
    int end = std::min(static_cast<int>(detectionDeque.size()) - 1, static_cast<int>(index) + range);
    int stateChangesInRange = 0;
    int stateChangesOutsideRange = 0;

    // Iterate through the entire detection deque
    for (int i = 1; i < detectionDeque.size(); ++i) {
        // Check for state changes
        if ((detectionDeque[i] == whichHazard && detectionDeque[i - 1] == 0)) {

            // Determine if the state change is within the specified range
            if (i >= start && i <= end) {
                stateChangesInRange++;
            } else {
                stateChangesOutsideRange++;
            }
        }
    }   

    std::cout << "state changes in range: " << stateChangesInRange << " state changes outside range: " << stateChangesOutsideRange << std::endl;

    
    *changesInRange = stateChangesInRange;
    *changesOutsideRange = stateChangesOutsideRange;
    
    

    // A valid detection requires exactly one state change within the range and none outside of it
    return (stateChangesInRange > 1 && stateChangesInRange < 5) && (stateChangesOutsideRange < 10);
}

bool isValidDetection(const std::deque<int>& detectionDeque, int index, int range, int whichHazard, int* changesInRange, int* changesOutsideRange){
    int start = std::max(0, static_cast<int>(index) - range);
    int end = std::min(static_cast<int>(detectionDeque.size()) - 1, static_cast<int>(index) + range);
    int stateChangesInRange = 0;
    int stateChangesOutsideRange = 0;

    // Iterate through the entire detection deque
    for (int i = 1; i < detectionDeque.size(); ++i) {
        // Check for state changes
        if ((detectionDeque[i] == whichHazard && detectionDeque[i - 1] == 0)) {

            // Determine if the state change is within the specified range
            if (i >= start && i <= end) {
                stateChangesInRange++;
            } else {
                stateChangesOutsideRange++;
            }
        }
    }   

    std::cout << "state changes in range: " << stateChangesInRange << " state changes outside range: " << stateChangesOutsideRange << std::endl;

    
    *changesInRange = stateChangesInRange;
    *changesOutsideRange = stateChangesOutsideRange;
    
    

    // A valid detection requires exactly one state change within the range and none outside of it
    return (stateChangesInRange > 1 && stateChangesInRange < 5) && (stateChangesOutsideRange < 10);
}

*/


// SIMULATION FUNCTION BEGIN
void runSimulation(unsigned int lag, double threshold, double influence, const std::string& directoryPath) {
    std::string logFileName = "allSensorLogFile.txt";
    std::string logFilePath = directoryPath + "/" + logFileName; // Adjusted for simplicity

    std::ifstream logFile(logFilePath);
    if (!logFile.is_open()) {
        std::cerr << "Error: Unable to open log file." << std::endl;
        return;
    }

    auto bumpDeque{ readFileIntoDeque("bump_buttons.txt") };
    auto potholeDeque{ readFileIntoDeque("pothole_buttons.txt") };

    auto bumpIndices{ getStateChangeIndices(bumpDeque) };
    auto potholeIndices{ getStateChangeIndices(potholeDeque) };
    
    // Initialize Active Filter
    ActiveFilter actFilter;

    // Initialize IIR Filter
    ThreeAxisIIR iirFiltAccel;
    ThreeAxisIIR iirFiltGyro;

    // Parameters for the Active Filter
    actFilter.setWindowParameters(50, 35);
    actFilter.setThreshold(0.2);

    ThreeAxisIIR_Init(&iirFiltAccel, filterAlpha);
    ThreeAxisIIR_Init(&iirFiltGyro, filterAlpha);

    // Variables to store the accel, gyro and angle values
    double ax, ay, az;
    double gr, gp, gy;

    // Variables to store the filtered/rotated acceleration values
    double ax_filtered, ay_filtered, az_filtered;
    double ax_rotated, ay_rotated, az_rotated;

    // Variables to store the filtered/rotated gyroscope values
    double gr_filtered, gp_filtered, gy_filtered;
    double gr_rotated, gp_rotated, gy_rotated;

    // Variables to store the angles:
    double pitchAngle{ 0.0 };
    double rollAngle{ 0.0 };

    // Variable to store the compound acceleration vector:
    double compoundAccelerationVector{ 0.0 };

    int sampleNumber{ 0 };
    const unsigned int wholeDequeSize{ 150 };
    unsigned int removeSamples{ 0 };

    std::deque<double> outData;
    outData.clear();

    std::deque<double> completedData;
    completedData.clear();

    std::deque<double> sequenceDeque;
    sequenceDeque.clear();

    std::deque<int> stateDeque;
    stateDeque.clear();

    std::string line;

    std::cout << "Starting Simulation for lag=" << lag << ", threshold=" << threshold << ", influence=" << influence << std::endl;

    int numberOfCorrectBumpDetections{ 0 };
    int numberOfCorrectPotholeDetections{ 0 };

    int numberOfIncorrectBumpDetections{ 0 };
    int numberOfIncorrectPotholeDetections{ 0 };

    // FILE LOOP BEGIN
    while (std::getline(logFile, line)) {
        std::istringstream ss(line);
        std::string token;

        while (std::getline(ss, token, ',')) {
            std::istringstream pairStream(token);
            std::string key, value;
            if (std::getline(pairStream, key, '=') && std::getline(pairStream, value)) {
                double val = std::stod(value);
                if (key == "ax") ax = val;
                else if (key == "ay") ay = val;
                else if (key == "az") az = val;
                else if (key == "gr") gr = val;
                else if (key == "gp") gp = val;
                else if (key == "gy") gy = val;
            }
        }

        ThreeAxisIIR_Update(&iirFiltAccel, ax, ay, az, &ax_filtered, &ay_filtered, &az_filtered);
        ThreeAxisIIR_Update(&iirFiltGyro, gr, gp, gy, &gr_filtered, &gp_filtered, &gy_filtered);

        complementaryFilter(ax_filtered, ay_filtered, az_filtered, gr_filtered, gp_filtered, gy_filtered, &rollAngle, &pitchAngle);

        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, ax_filtered, ay_filtered, az_filtered, &ax_rotated, &ay_rotated, &az_rotated);
        rotateAll(rollAngle*degreesToRadians, pitchAngle*degreesToRadians, gr_filtered, gp_filtered, gy_filtered, &gr_rotated, &gp_rotated, &gy_rotated);

        compoundAccelerationVector = compoundVector(ax_rotated, ay_rotated, az_rotated);

        actFilter.feedData(compoundAccelerationVector);

        if (actFilter.getCompletedDataSize() > 0) {
            std::deque<double> completedData = actFilter.getCompletedData();
            AppendDeque(outData, completedData);
        }

        if (outData.size() > wholeDequeSize) {
            removeSamples = static_cast<unsigned int>(outData.size() - wholeDequeSize);
            outData.erase(outData.begin(), outData.begin() + removeSamples);
        }

        sampleNumber++;

        if (outData.size() == wholeDequeSize) {
            sequenceDeque = z_score_thresholding(outData, lag, threshold, influence);
        }



        if (getStateChange(sequenceDeque) == SequenceType::Rising) {
            stateDeque.push_back(1);
        }

        if (getStateChange(sequenceDeque) == SequenceType::Falling) {
            stateDeque.push_back(-1);
        }

        if (getStateChange(sequenceDeque) == SequenceType::Stable) {
            stateDeque.push_back(0);
        }
    }
    // FILE LOOP END

    logFile.close();

    // ---------------- Validation: ----------------------------------------------

    unsigned int range{ 100 };

    std::cout << "for bumps: " << std::endl;
    for (const auto& index : bumpIndices) {

        bool validBumpDetection = isValidDetection(stateDeque, index, range, 1);
    
        if(validBumpDetection){
            ++numberOfCorrectBumpDetections;
        }
        else{
            ++numberOfIncorrectBumpDetections;
        }
    }

    std::cout << "for potholes: " << std::endl;
    for (const auto& index : potholeIndices) {

        bool validPotholeDetection = isValidDetection(stateDeque, index, range, -1);
        
        if(validPotholeDetection){
            ++numberOfCorrectPotholeDetections;
        }
        else{
            ++numberOfIncorrectPotholeDetections;
        }
    }


    // ---------------- Save Variables Used in Runtime: --------------------------

    std::string ZTransformVariablesFileName = "z_transform_variables.csv";
    std::ofstream ZTransformVariablesOutputFile(ZTransformVariablesFileName, std::ios::app);

    if (!ZTransformVariablesOutputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    std::vector<std::string> variableNames = {"lag", "threshold", "influence", "numberOfCorrectBumpDetections", "numberOfCorrectPotholeDetections", "numberOfIncorrectBumpDetections", "numberOfIncorrectPotholeDetections"}; //  "changesInRangeBump", "changesOutRangeBump", "changesInRangePot", "changesOutRangePot"};
    if (isFileEmpty(ZTransformVariablesFileName)) {
        writeCSVHeader(ZTransformVariablesOutputFile, variableNames);
    }

    ZTransformVariablesOutputFile << std::to_string(lag) << "," << std::to_string(threshold) << "," << std::to_string(influence)  << "," << std::to_string(numberOfCorrectBumpDetections)  << "," <<  std::to_string(numberOfCorrectPotholeDetections)  << "," << std::to_string(numberOfIncorrectBumpDetections) << "," << std::to_string(numberOfIncorrectPotholeDetections)  << std::endl; // << "," << changes_in_range_bump << "," << changes_out_range_bump << "," << changes_in_range_pot << "," << changes_out_range_pot << std::endl;
    ZTransformVariablesOutputFile.close();

    
    std::string outputFilename = "output_state_signals.txt";
    std::ofstream outputFile(outputFilename);

    if (!outputFile.is_open()) {
        std::cerr << "Failed to open " << outputFilename << std::endl;
        return;
    }

    for (const auto& signal : stateDeque) {
        outputFile << signal << "\n";
    }

    outputFile.close();
    
    

    std::cout << "Simulation for lag=" << lag << ", threshold=" << threshold << ", influence=" << influence << " completed." << std::endl;
}
// SIMULATION FUNCTION END







int main(int argc, char* argv[]){

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];


    // runSimulation(55, 7, influence, directoryPath);

    // return 0;

    // ------------------------------------------------------------

    for (unsigned int lag = 40; lag <= 60; lag += 2) {
        for (double threshold = 7.0; threshold <= 20.0; threshold += 1.0) {
            //for (double influence = 0.0; influence <= 0.5; influence += 0.1) { // influence does not matter
                runSimulation(lag, threshold, influence, directoryPath);
            //}
        }
    }

    std::cout << "All simulations completed." << std::endl;

    return 0;

}
