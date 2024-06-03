#ifndef FILTERS_H
#define FILTERS_H

#include <cstdint>
#include <vector>
#include <deque>
#include <iostream>
#include <fstream>
#include <cmath>

typedef struct{
    double alpha;
    double out;
} FirstOrderIIR;

typedef struct {
    FirstOrderIIR x;
    FirstOrderIIR y;
    FirstOrderIIR z;
} ThreeAxisIIR;

const double pi{ 3.14159265359 };


void FirstOrderIIR_Init(FirstOrderIIR *filt, double alpha);
double FirstOrderIIR_Update(FirstOrderIIR *filt, double in);

void ThreeAxisIIR_Init(ThreeAxisIIR *filt, double alpha);
void ThreeAxisIIR_Update(ThreeAxisIIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z);


// --------------------------------------------------------------------------------------------------
/*
#define FIR_FILTER_LENGTH (105)

typedef struct{
    float buf[FIR_FILTER_LENGTH]; // circular buffer
    uint8_t bufIndex;
    float out;
}FIRFilter;

typedef struct{
    FIRFilter x;
    FIRFilter y;
    FIRFilter z;
} ThreeAxisFIR;

void FIRFilter_Init(FIRFilter *fir);
double FIRFilter_Update(FIRFilter *fir, double inp);

void ThreeAxisFIR_Init(ThreeAxisFIR *filt);
void ThreeAxisFIR_Update(ThreeAxisFIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z);


*/

// --------------------------------------------------------------------------------------------------

// FIR Filter class
class FIRFilter {
public:
    FIRFilter(const std::deque<double>& coefficients) : coefficients(coefficients), buffer(coefficients.size(), 0.0) {}

    double update(double newSample) {
        // Add new sample to the buffer
        buffer.pop_front();
        buffer.push_back(newSample);

        // Apply the filter
        double filteredValue = 0.0;
        for (size_t i = 0; i < coefficients.size(); ++i) {
            filteredValue += coefficients[i] * buffer[i];
        }

        return filteredValue;
    }

private:
    std::deque<double> coefficients;
    std::deque<double> buffer;
};


std::deque<double> designFIRFilter(int numTaps, double cutoffFrequency, double samplingRate);
void ThreeAxisFIR_Update(FIRFilter& filter, double ax, double ay, double az, double& ax_filtered, double& ay_filtered, double& az_filtered);



// -------------------------------------------------------------------------------------------------------------------------

class ActiveFilter
{    
private:
    /* data */
    int m_windowSize;
    int m_overlapSize;
    int m_diffSize;

    // https://stackoverflow.com/questions/5563952/why-there-is-no-pop-front-method-in-c-stdvector
    // http://adrinael.net/containerchoice 

    double m_posCoef;   //  = 1.2; // was 1.4 before, but 1.2 was written in Python analysis. UPDATE: 1.2 BEFORE, 1.4 NOW
    double m_negCoef;   //  = 0.8; // UPDATE: 0.4 BEFORE, 0.6 NOW.
    double m_threshold; //   = 0.2; // was 0.15 before, 0.5 is more suitable after analysis


    double m_offset      = 1.0;      //signal offset
    bool  m_dataInit    = false;
    std::deque<double> m_data;  //size: windowSize
    std::deque<double> m_newData; // size: windowSize-overlapSize
    mutable std::deque<double> m_completedData; // completed data , no more operations will be done on there

    // |_______||__overlap___||__newdata__|
    // |---------window-------|
    //          |---------new-window------|
    
public:
    ActiveFilter(/* args */);
    ~ActiveFilter();
    void                setWindowParameters(int windowSize, int overlapSize);
    void                setThreshold(double threshold);
    void                setOffset(double offset);
    void                setCoefficients(double posCoef, double negCoef);
    void                feedData(double data);
    long unsigned int   getCompletedDataSize()const;
    std::deque<double>  getCompletedData()const;     // maybe return can be changed as vector / queue 

private:
    void                initMembers();
    void                doCalculation();
    double              getMaxValue();
    double              getMinValue();
};

#endif // FILTERS_H