#include "simulation.h"


// SIMULATION FUNCTION BEGIN
void runSimulationOld(const std::vector<SensorData>& sensorData, unsigned int lag, double z_score_threshold, double influence, const std::vector<int> bumpIndices, const std::vector<int> potholeIndices, double actFiltThres, double actFiltPosCoef, double actFiltNegCoef) {

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
    ThreeAxisIIR_Init(&iirFiltAccel, filterAlpha);
    ThreeAxisIIR_Init(&iirFiltGyro, filterAlpha);
    
    
    // Initialize FIR Filter Coefficients:
    
    /*
    std::deque<double> filterCoefficients = designFIRFilter(numberOfTaps, cutoffFrequency, samplingRate);
    FIRFilter FIRfilterAccel(filterCoefficients);
    FIRFilter FIRfilterGyro(filterCoefficients);
    */

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
        //determineState(sequenceBumpDeque, stateBumpDeque);
        //determineState(sequencePotholeDeque, statePotholeDeque);

        sampleNumber++;

    }

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
    
    
    
    
    
    
    
    


    //std::cout << "false positive state changes from 1 to 0 in state bump deque: " << countStateChangesFrom1to0(stateBumpDeque) - numberOfCorrectBumpDetections << std::endl;
    //std::cout << "false positive state changes from -1 to 0 in state pothole deque: " << countStateChangesFromMinus1to0(statePotholeDeque) - numberOfCorrectPotholeDetections << std::endl;


    // ---------------- Save Variables Used in Runtime: --------------------------

    std::string testResultsFileName = "testResults_oldProgram.csv";
    std::ofstream testResultsOutputFile(testResultsFileName, std::ios::app);

    if (!testResultsOutputFile.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }

    // std::vector<std::string> variableNames = {"lag", "threshold", "influence", "numberOfCorrectBumpDetections", "numberOfCorrectPotholeDetections", "numberOfIncorrectBumpDetections", "numberOfIncorrectPotholeDetections"}; //  "changesInRangeBump", "changesOutRangeBump", "changesInRangePot", "changesOutRangePot"};
    std::vector<std::string> variableNames = {"lag", "zScoreThreshold", "influence", 
                                             "bumpPressCount", "correctBumpDetections", "incorrectBumpDetections", 
                                             "potholePressCount", "correctPotholeDetections",  "incorrectPotholeDetections", 
                                             "bumpDetectionSuccess", "potholeDetectionSuccess", 
                                             "bumpFalsePositives", "potholeFalsePositives",
                                             "activeFilterThreshold", "activeFilterPosCoef", "activeFilterNegCoef",
                                             "activeFilterThresholdBump", "activeFilterPosCoefBump", "activeFilterNegCoefBump",
                                             "activeFilterThresholdPothole", "activeFilterPosCoefPothole", "activeFilterNegCoefPothole"}; //  "filterThreshold", "positiveCoef", "negativeCoef"}; //  "changesInRangeBump", "changesOutRangeBump", "changesInRangePot", "changesOutRangePot"};

    if (isFileEmpty(testResultsFileName)) {
        writeCSVHeader(testResultsOutputFile, variableNames);
    }

    testResultsOutputFile <<         std::to_string(lag) << ", " << std::to_string(z_score_threshold) << ", " << std::to_string(influence)  
                          << ", " << std::to_string(bumpIndices.size()) << ", " << std::to_string(numberOfCorrectBumpDetections)  << ", " << std::to_string(numberOfIncorrectBumpDetections) 
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

                          //<< ", " << std::to_string(overlapSizeBump)
                          //<< ", " << std::to_string(overlapSizePothole)
                          //<< ", " << std::to_string(cutoffFrequency)

                          //<< ", " << std::to_string(bumpActiveFilterThreshold)
                          //<< ", " << std::to_string(bumpActiveFilterPositiveCoef)
                          //<< ", " << std::to_string(bumpActiveFilterNegativeCoef)

                          //<< ", " << std::to_string(potholeActiveFilterThreshold)
                          //<< ", " << std::to_string(potholeActiveFilterPositiveCoef)
                          //<< ", " << std::to_string(potholeActiveFilterNegativeCoef)
                        
                          << std::endl; 

    testResultsOutputFile.close();

    
    // Save the output state changes:

    
    //saveDequeIntoFile(stateBumpDeque, "bump_state_signal");
    //saveDequeIntoFile(statePotholeDeque, "pothole_state_signal");
    
    //saveDequeIntoFile(activeFilterBumpOutput, "active_filter_bump_output");
    //saveDequeIntoFile(activeFilterPotholeOutput, "active_filter_pothole_output");
    
    
    saveDequeIntoFile(stateDeque, "output_state_signals");
    /*
    saveDequeIntoFile(activeFilterOutput, "active_filter_output");
    */
    

    // Save the filtered/unfiltered vector:
    //saveDequeIntoFile(filteredVectorDeque, "filtered_signal");
    //saveDequeIntoFile(unfilteredVectorDeque, "unfiltered_signal");

    std::cout << "Simulation for lag=" << lag << ", threshold=" << z_score_threshold << ", influence=" << influence << " completed." << std::endl;
}
// SIMULATION FUNCTION END



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



    //---------------------------------------------------------------------------


    auto bumpIndices{ getStateChangeIndices(readFileIntoDeque("bump_buttons.txt")) };
    auto potholeIndices{ getStateChangeIndices(readFileIntoDeque("pothole_buttons.txt")) };

    
    // Start timing
    auto start = std::chrono::high_resolution_clock::now();

    /*
    
    */
    for(double z_score_threshold = 6.0; z_score_threshold <= 15.0; z_score_threshold += 1.0){
        for(double threshold = 0.0; threshold <= 0.2; threshold += 0.02){
            for(double posCoef = 1.0; posCoef <= 1.9; posCoef += 0.2){
                for(double negCoef = 0.0; negCoef <= 0.9; negCoef += 0.2){
                    runSimulationOld(sensorData, lag, z_score_threshold, influence, bumpIndices, potholeIndices, threshold, posCoef, negCoef);
                }
            }
        }
    }
    
    
        

    // End timing
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "Execution time: " << duration << " milliseconds" << std::endl;


    

    /*
    for(double z_score_threshold = 5.0; z_score_threshold <= 15.0; z_score_threshold += 5.0){
        for(double activeFiltThres = 0.01; activeFiltThres <= 0.2; activeFiltThres += 0.02){
            for(double activeFiltPosCoef = 1.1; activeFiltPosCoef <= 2.0; activeFiltPosCoef += 0.1){
                for(double activeFiltNegCoef = 0.1; activeFiltNegCoef <= 0.9; activeFiltNegCoef += 0.1){
                runSimulation(logLines, lag, z_score_threshold, influence, bumpIndices, potholeIndices, activeFiltThres, activeFiltPosCoef, activeFiltNegCoef);
                }
            }
        }    
    }
    */
    
    


    //runSimulation(logLines, lag, z_score_threshold, influence, bumpIndices, potholeIndices, cutoffFrequency, 0.8, 1.2, 0.4);



    std::cout << "Simulations completed." << std::endl;

    return 0;
}