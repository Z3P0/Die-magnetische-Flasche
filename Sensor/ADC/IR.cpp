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
	maxSamples = 15;
	totalSamples = 0;
	a = c = 0;
	b = 1;
}

IR::~IR() {}

void IR::init() {
	sensor->config(ADC_PARAMETER_RESOLUTION, 12);
	sensor->init(ADC_CH_013);
}

float IR::read() {
	float x = readInternal();
	// y = A + Bx + Cx2;
	return a + b * x + c * x * x;
}

float IR::readInternal() {
	int32_t val = sensor->read(ADC_CH_013);
	val = val & 0xFFFFFFF8;     //Removing noise on LSBs
	return ((float) val) / 839;
}

void IR::sample(float distance) {
	dist[totalSamples] = distance;
	volt[totalSamples] = readInternal();
	PRINTF("SAMPLE %d  Value = %f       Distance = %f \r\n", totalSamples + 1, volt[totalSamples], dist[totalSamples]);
	totalSamples++;
	if (totalSamples > maxSamples) {
		scale();
		totalSamples = 0;
	}
}

void IR::scale() {
	float xMean = 0;
	float x2mean = 0;
	float yMean = 0;

	float sum_xi, sum_xi2, sum_xiyi, sum_xi3, sum_xi4, sum_xi2yi;
	sum_xi = sum_xi2 = sum_xiyi = sum_xi3 = sum_xi4 = sum_xi2yi = 0;
	for (int i = 0; i < maxSamples; i++) {
		float xi = volt[i];
		float yi = dist[i];
		float xi2 = xi * xi;
		float xi3 = xi2 * xi;

		sum_xi += xi;
		sum_xi2 += xi2;
		sum_xiyi += xi * yi;
		sum_xi3 += xi3;
		sum_xi4 += xi3 * xi;
		sum_xi2yi += xi2 * yi;

		yMean += dist[i];
	}
	xMean = sum_xi / maxSamples;
	x2mean = sum_xi2 / maxSamples;
	yMean = yMean / maxSamples;

	float sxx, sxy, sxx2, sx2x2, sx2y;
	sxx = (sum_xi2 / maxSamples) - (xMean * xMean);
	sxy = (sum_xiyi / maxSamples) - (xMean * yMean);
	sxx2 = (sum_xi3 / maxSamples) - (xMean * x2mean);
	sx2x2 = (sum_xi4 / maxSamples) - (x2mean * x2mean);
	sx2y = (sum_xi2yi / maxSamples) - (x2mean * yMean);

	float denom = ((sxx * sx2x2) - (sxx2 * sxx2));
	b = ((sxy * sx2x2) - (sx2y * sxx2)) / denom;
	c = ((sx2y * sxx) - (sxy * sxx2)) / denom;
	a = yMean - b * xMean - c * x2mean;

	PRINTF("Scaling IR... New parameters:\r\n");
	PRINTF("A = %f      B = %f      C = %f \r\n", a, b, c);
}

