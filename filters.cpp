#include "filters.h"

void FirstOrderIIR_Init(FirstOrderIIR *filt, double alpha){

    if(alpha < 0.0f){
        filt->alpha = 0.0f;
    }
    else if(alpha > 1.0f){
        filt->alpha = 1.0f;
    }
    else{
        filt->alpha = alpha;
    }

    filt->out = 0.0f;
}

double FirstOrderIIR_Update(FirstOrderIIR *filt, double in){

    filt->out = (1.0f - filt->alpha) * in + filt->alpha * filt->out;
    return filt->out;
}

void ThreeAxisIIR_Init(ThreeAxisIIR *filt, double alpha){
    FirstOrderIIR_Init(&filt->x, alpha);
    FirstOrderIIR_Init(&filt->y, alpha);
    FirstOrderIIR_Init(&filt->z, alpha);
}

void ThreeAxisIIR_Update(ThreeAxisIIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z){
    *out_x = FirstOrderIIR_Update(&filt->x, in_x);
    *out_y = FirstOrderIIR_Update(&filt->y, in_y);
    *out_z = FirstOrderIIR_Update(&filt->z, in_z);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------

static double FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.000454166748923280,
                                                        -0.005602072298710518,
                                                        -0.015270317254284961,
                                                        -0.012770541519951308,
                                                         0.032706883438698792,
                                                         0.128756425549938464,
                                                         0.233178480001847210,
                                                         0.278910617662771088,
                                                         0.233178480001847210,
                                                         0.128756425549938464,
                                                         0.032706883438698792,
                                                        -0.012770541519951323,
                                                        -0.015270317254284968,
                                                        -0.005602072298710513,
                                                        -0.000454166748923280 };

void FIRFilter_Init(FIRFilter *fir){

    // Clear input buffer
    for(uint8_t n = 0; n < FIR_FILTER_LENGTH; ++n){
        fir->buf[n] = 0.0f;
    }

    // Reset buffer index
    fir->bufIndex = 0;

    // Clear filter output
    fir->out = 0.0f;
}


double FIRFilter_Update(FIRFilter *fir, double inp){

    // Store latest sample in buffer
    fir->buf[fir->bufIndex] = inp;

    // Increment buffer index and wrap around if necessary
    fir->bufIndex++;

    if(fir->bufIndex == FIR_FILTER_LENGTH){
        fir->bufIndex = 0;
    }

    // Compute new output sample (via Convolution)

    fir->out = 0.0f;
    uint8_t sumIndex = fir->bufIndex;

    for(uint8_t n = 0; n < FIR_FILTER_LENGTH; ++n){

        // Decrement index and wrap if necessary 
        if(sumIndex>0){
            sumIndex--;
        }
        else{
            sumIndex = FIR_FILTER_LENGTH - 1;
        }

        // Multiply impulse response with shifted input sample and add to output
        fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];
    }
    // Return filtered output
    return fir->out;
}

void ThreeAxisFIR_Init(ThreeAxisFIR *filt){
    FIRFilter_Init(&filt->x);
    FIRFilter_Init(&filt->y);
    FIRFilter_Init(&filt->z);
}

void ThreeAxisFIR_Update(ThreeAxisFIR *filt, double in_x, double in_y, double in_z, double *out_x, double *out_y, double *out_z){
    *out_x = FIRFilter_Update(&filt->x, in_x);
    *out_y = FIRFilter_Update(&filt->y, in_y);
    *out_z = FIRFilter_Update(&filt->z, in_z);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------------------------------------------------------

ActiveFilter::ActiveFilter(/* args */)
{
    this->initMembers();
}

ActiveFilter::~ActiveFilter()
{

}

void ActiveFilter::setWindowParameters(int windowSize, int overlapSize)
{
    this->m_windowSize  = windowSize;
    this->m_overlapSize = overlapSize;
    this->m_diffSize    = this->m_windowSize - this->m_overlapSize;
}

void ActiveFilter::setThreshold(double threshold)
{
    this->m_threshold = threshold;
}

void ActiveFilter::setOffset(double offset = 1.0)
{
    this->m_offset = offset;
}

void ActiveFilter::setCoefficients(double posCoef, double negCoef)
{
    this->m_posCoef = posCoef;
    this->m_negCoef = negCoef;
}

void ActiveFilter::feedData(double data)
{
    this->m_newData.push_back(data-this->m_offset); //reduce offset value in order to keep signal around 0
    if (this->m_newData.size() >= this->m_diffSize)
    {
        if(this->m_data.size() >= this->m_diffSize)
        {
            //move completed data
            for(int i=0; i< this->m_diffSize; i++)
            {            
                this->m_completedData.push_back(this->m_data.at(0));  
                this->m_data.pop_front();
            }
        }

        //add new data
        for(int i=0; i< this->m_diffSize; i++)
        {
            this->m_data.push_back(this->m_newData.at(0));
            this->m_newData.pop_front();
        }

        this->doCalculation();
    }
}

long unsigned int ActiveFilter::getCompletedDataSize()
{
    return this->m_completedData.size();
}

std::deque<double> ActiveFilter::getCompletedData()
{    
    // temporary copy for return
    std::deque<double> retData(this->m_completedData);
    //clear completed data
    this->m_completedData.clear();
    return retData;
}


void ActiveFilter::initMembers()
{
    this->m_data.clear();
    this->m_completedData.clear();
    this->m_newData.clear();

    this->m_data.resize(0);
    this->m_completedData.resize(0);
    this->m_newData.resize(0);
}

void ActiveFilter::doCalculation()
{
    double minVal = this->getMinValue();
    double maxVal = this->getMaxValue();
    double operationCoef = 1.0;

    /*
    if((maxVal - 0.0) > this->m_threshold){
        operationCoef = this->m_posCoef;
    }
    else if((minVal - 0.0) > this->m_threshold){
        operationCoef = this->m_negCoef;
    }
    */
    

    // original method
    if( (maxVal-minVal) > this->m_threshold)
    {
        operationCoef = this->m_posCoef;
    }else{
        operationCoef = this->m_negCoef;
    }
    

    for (int i=0; i<this->m_data.size(); i++)
    {
        this->m_data.at(i) = this->m_data.at(i) * operationCoef;
    }
}

double ActiveFilter::getMaxValue()
{
    double value=this->m_data.at(0);
    for(int i=0; i< this->m_data.size() ;i++ )
    {
        if(value < this->m_data.at(i))
        {
            value = this->m_data.at(i);
        }
    }
    return value;
}   

double ActiveFilter::getMinValue()
{
    double value=this->m_data.at(0);
    for(int i=0; i< this->m_data.size() ;i++ )
    {
        if(value > this->m_data.at(i))
        {
            value = this->m_data.at(i);
        }
    }
    return value;
}
