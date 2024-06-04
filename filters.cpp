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

/*

static double FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {
    0.000236390274993438,
    0.000125726876208487,
    0.000000000000000000,
    -0.000141697089057423,
    -0.000298878057473519,
    -0.000468388571036301,
    -0.000643338310758618,
    -0.000812442693949838,
    -0.000959945028835860,
    -0.001066234801934345,
    -0.001109204246396959,
    -0.001066299932030298,
    -0.000917137141508291,
    -0.000646461416710867,
    -0.000247173052155022,
    0.000276915230899998,
    0.000908933130337417,
    0.001617748082237211,
    0.002357976504758190,
    0.003071401359931168,
    0.003689874252692277,
    0.004139649357253039,
    0.004346957442181298,
    0.004244493278364228,
    0.003778371311398539,
    0.002915014538129733,
    0.001647390147012687,
    -0.000000000000000002,
    -0.001967921766925677,
    -0.004161458146611896,
    -0.006451621307024914,
    -0.008679668135170225,
    -0.010664061137222311,
    -0.012209808096049466,
    -0.013119712248685788,
    -0.013206886446893412,
    -0.012307745205432763,
    -0.010294600696161740,
    -0.007086961999672602,
    -0.002660676192641165,
    0.002945844984283044,
    0.009628903358347476,
    0.017220671759899442,
    0.025494352835479472,
    0.034173190994070270,
    0.042942920093654392,
    0.051466981566307642,
    0.059403639830720410,
    0.066423970234246829,
    0.072229611644087727,
    0.076569168459950002,
    0.079252216958026867,
    0.080160014429735818,
    0.079252216958026867,
    0.076569168459950002,
    0.072229611644087727,
    0.066423970234246829,
    0.059403639830720410,
    0.051466981566307642,
    0.042942920093654392,
    0.034173190994070277,
    0.025494352835479472,
    0.017220671759899442,
    0.009628903358347476,
    0.002945844984283044,
    -0.002660676192641165,
    -0.007086961999672604,
    -0.010294600696161740,
    -0.012307745205432765,
    -0.013206886446893417,
    -0.013119712248685789,
    -0.012209808096049466,
    -0.010664061137222312,
    -0.008679668135170231,
    -0.006451621307024916,
    -0.004161458146611896,
    -0.001967921766925678,
    -0.000000000000000002,
    0.001647390147012687,
    0.002915014538129732,
    0.003778371311398539,
    0.004244493278364229,
    0.004346957442181301,
    0.004139649357253038,
    0.003689874252692277,
    0.003071401359931170,
    0.002357976504758192,
    0.001617748082237213,
    0.000908933130337418,
    0.000276915230899998,
    -0.000247173052155022,
    -0.000646461416710867,
    -0.000917137141508292,
    -0.001066299932030299,
    -0.001109204246396959,
    -0.001066234801934346,
    -0.000959945028835859,
    -0.000812442693949838,
    -0.000643338310758618,
    -0.000468388571036301,
    -0.000298878057473519,
    -0.000141697089057423,
    0.000000000000000000,
    0.000125726876208487,
    0.000236390274993438,


};

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
*/
// ------------------------------------------------------------------------------------------------------------------------------------------------

// Function to design a low-pass FIR filter using a Hamming window
std::deque<double> designFIRFilter(int numTaps, double cutoffFrequency, double samplingRate) {
    std::deque<double> filterCoefficients(numTaps);
    double M = numTaps - 1;
    double fc = cutoffFrequency / samplingRate;

    for (int n = 0; n < numTaps; ++n) {
        if (n - M / 2 == 0) {
            filterCoefficients[n] = 2 * pi * fc;
        } else {
            filterCoefficients[n] = std::sin(2 * pi * fc * (n - M / 2)) / (n - M / 2);
        }
        filterCoefficients[n] *= (0.54 - 0.46 * std::cos(2 * pi * n / M)); // Hamming window
    }

    // Normalize the filter coefficients
    double sum = 0.0;
    for (double coeff : filterCoefficients) {
        sum += coeff;
    }
    for (double& coeff : filterCoefficients) {
        coeff /= sum;
    }

    return filterCoefficients;
}

// Three-axis FIR filter update function
void ThreeAxisFIR_Update(FIRFilter& filter,
                         double ax, double ay, double az,
                         double& ax_filtered, double& ay_filtered, double& az_filtered) {
    ax_filtered = filter.update(ax);
    ay_filtered = filter.update(ay);
    az_filtered = filter.update(az);
}

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

long unsigned int ActiveFilter::getCompletedDataSize() const
{
    return this->m_completedData.size();
}

std::deque<double> ActiveFilter::getCompletedData() const
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
