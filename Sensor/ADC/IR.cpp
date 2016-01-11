/*
 * IR.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "IR.h"

//HAL_ADC sensor(ADC_IDX1);

IR::IR(HAL_ADC * adc) {
	sensor = adc;
	MaxSamples = 15;
	totalSamples = 0;
	A=C=0;
	B=1;
}

IR::~IR() {}

void IR::init()
{
	sensor->config(ADC_PARAMETER_RESOLUTION, 12);
	sensor->init(ADC_CH_013);
}

float IR::read(){
	float x = readInternal();
	//y = A + Bx + Cx2;
	return A + B*x + C*x*x;
}

float IR::readInternal()
{
	int32_t val = sensor->read(ADC_CH_013);
	val = val & 0xFFFFFFF8; //Removing noise on LSBs
	return ((float)val)/839;
}

void IR::sample(float distance)
{
	dist[totalSamples] = distance;
	volt[totalSamples] = readInternal();
	PRINTF("SAMPLE %d  Value = %f       Distance = %f \r\n", totalSamples +1, volt[totalSamples], dist[totalSamples]);
	totalSamples++;
	if(totalSamples > MaxSamples)
	{
		scale();
		totalSamples = 0;
	}
}

void IR::scale(){
	//TODO: Find scaling function or a polynom to scale the values
	float xmean = 0;
	float x2mean = 0;
	float ymean= 0;

	float Sumxi, Sum_xi2, Sum_xiyi, Sum_xi3,Sum_xi4, Sum_xi2yi;
	Sumxi = Sum_xi2 = Sum_xiyi = Sum_xi3 = Sum_xi4 = Sum_xi2yi = 0;
	for(int i=0; i<MaxSamples; i++)
	{
		float xi = volt[i];
		float yi = dist[i];
		float xi2 = xi*xi;
		float xi3 = xi2*xi;

		Sumxi += xi;
		Sum_xi2 += xi2;
		Sum_xiyi += xi*yi;
		Sum_xi3 += xi3;
		Sum_xi4 += xi3 * xi;
		Sum_xi2yi += xi2 * yi;

		ymean += dist[i];
	}
	xmean = Sumxi/MaxSamples;
	x2mean = Sum_xi2/MaxSamples;
	ymean = ymean/MaxSamples;

	float Sxx, Sxy, Sxx2, Sx2x2, Sx2y;
	Sxx = (Sum_xi2/MaxSamples) - (xmean * xmean);
	Sxy = (Sum_xiyi/MaxSamples) - (xmean * ymean);
	Sxx2 = (Sum_xi3/MaxSamples) - (xmean * x2mean);
	Sx2x2 = (Sum_xi4/MaxSamples) - (x2mean * x2mean);
	Sx2y = (Sum_xi2yi/MaxSamples) - (x2mean * ymean);

	float denom = ((Sxx * Sx2x2) - (Sxx2 * Sxx2));
	B = ((Sxy * Sx2x2) - (Sx2y*Sxx2)) / denom;
	C = ((Sx2y * Sxx) - (Sxy * Sxx2)) / denom;
	A = ymean - B*xmean - C*x2mean;

	PRINTF("Scaling IR... New parameters:\r\n");
	PRINTF("A = %f      B = %f      C = %f \r\n", A, B, C);
}


