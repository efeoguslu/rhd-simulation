#include "simulation.h"


template<typename T>
void populateVector(std::vector<T>& vec, T start, T end, T step) {
    for (T i = start; i <= end; i += step) {
        vec.push_back(i);
    }
}



int main(int argc, char* argv[]){

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <directory_path>" << std::endl;
        return -1;
    }

    std::string directoryPath = argv[1];

    //---------------------------------------------------------------------------

    std::string logFileName = "allSensorLogFile.txt";
    std::string logFilePath = directoryPath + "/" + logFileName; // Adjusted for simplicity

    // Read the entire file into memory
    std::ifstream logFile(logFilePath);
    if (!logFile.is_open()) {
        std::cerr << "Error: Unable to open log file." << std::endl;
        return 1;
    }

    std::vector<std::string> logLines;
    std::string line;
    while (std::getline(logFile, line)) {
        logLines.push_back(line);
    }
    logFile.close();


    // Parse the log lines into sensor data
    std::vector<SensorData> sensorData;

    for (const auto& line : logLines) {
        std::istringstream ss(line);
        std::string token;
        SensorData data;
        while (std::getline(ss, token, ',')) {
            std::istringstream pairStream(token);
            std::string key, value;
            if (std::getline(pairStream, key, '=') && std::getline(pairStream, value)) {
                double val = std::stod(value);
                if (key == "ax") data.ax = val;
                else if (key == "ay") data.ay = val;
                else if (key == "az") data.az = val;
                else if (key == "gr") data.gr = val;
                else if (key == "gp") data.gp = val;
                else if (key == "gy") data.gy = val;
            }
        }
        sensorData.push_back(data);
    }

    // ---------------------------------------------------------------------------

    auto bumpIndices{ getStateChangeIndices(readFileIntoDeque("bump_buttons.txt")) };
    auto potholeIndices{ getStateChangeIndices(readFileIntoDeque("pothole_buttons.txt")) };

    // ---------------------------------------------------------------------------

    
    std::vector<unsigned int> lags;
    lags.clear();  
    std::vector<double> z_score_thresholds;
    z_score_thresholds.clear();
    std::vector<double> influences;
    influences.clear();

    // ---------------------------------------------------------------------------

    std::vector<double> activeFilterThresholds;
    activeFilterThresholds.clear();
    std::vector<double> activeFilterPositiveCoefficients;
    activeFilterPositiveCoefficients.clear();
    std::vector<double> activeFilterNegativeCoefficients;
    activeFilterNegativeCoefficients.clear();

    std::vector<double> iirFilterAlphas;
    iirFilterAlphas.clear();

    // ---------------------------------------------------------------------------

    lags = {50};
    
    //populateVector(lags, 0u, 70u, 5u);
    //populateVector(z_score_thresholds, 7.0, 20.0, 1.0);
    //populateVector(influences, 0.0, 0.3, 0.1);


    influences = { 0.25 }; 
    z_score_thresholds = {10.0};


    activeFilterThresholds = {0.2};
    activeFilterPositiveCoefficients = {1.6};
    activeFilterNegativeCoefficients = {0.1};

    /*
    populateVector(activeFilterThresholds, 0.05, 0.5, 0.05);
    populateVector(activeFilterPositiveCoefficients, 1.1, 2.0, 0.1);
    populateVector(activeFilterNegativeCoefficients, 0.1, 0.9, 0.1);
    */





    iirFilterAlphas = { 0.9 };

    //populateVector(iirFilterAlphas, 0.0, 1.0, 0.1);


    // ---------------------------------------------------------------------------

    // Overloads for runSimulations are written. You can pass in either vectors of parameters, or variables.

    auto start = std::chrono::high_resolution_clock::now();
    

    runSimulations(sensorData, 
                  lags, z_score_thresholds, influences, 
                  bumpIndices, potholeIndices, 
                  iirFilterAlphas,
                  activeFilterThresholds, activeFilterPositiveCoefficients, activeFilterNegativeCoefficients);
    
    
                  
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "\nDuration:   " << std::setw(20) << duration;

    std::cout << "\nSimulations completed." << std::endl;

}
