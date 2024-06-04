#include <vector>
#include <deque>
#include <string>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

// Assume the necessary includes and other functions (like ThreeAxisIIR, complementaryFilter, etc.) are already defined.

void runSimulation(const std::vector<SensorData>& sensorData, unsigned int lag, double z_score_threshold, double influence, 
                   const std::vector<int> bumpIndices, const std::vector<int> potholeIndices, double actFiltThres, double actFiltPosCoef, double actFiltNegCoef, std::vector<std::string>& results) {
    
    // ...

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
                 << ", " << std::to_string(activeFilterThreshold)
                 << ", " << std::to_string(activeFilterPositiveCoef)
                 << ", " << std::to_string(activeFilterNegativeCoef)
                 << std::endl;

    // Store the result string in the results vector
    results.push_back(resultStream.str());

    // ... 
}

void runSimulations(const std::vector<SensorData>& sensorData, const std::vector<unsigned int>& lags, const std::vector<double>& z_score_thresholds, const std::vector<double>& influences, const std::vector<int>& bumpIndices, const std::vector<int>& potholeIndices, double actFiltThres, double actFiltPosCoef, double actFiltNegCoef) {
    // Prepare to collect results
    std::vector<std::string> results;
    results.reserve(lags.size() * z_score_thresholds.size() * influences.size());

    // Loop through different configurations
    for (unsigned int lag : lags) {
        for (double z_score_threshold : z_score_thresholds) {
            for (double influence : influences) {
                runSimulation(sensorData, lag, z_score_threshold, influence, bumpIndices, potholeIndices, actFiltThres, actFiltPosCoef, actFiltNegCoef, results);
            }
        }
    }

    // Write results to the CSV file once
    std::string testResultsFileName = "testResults_newZthresholdFixedFakes.csv";
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