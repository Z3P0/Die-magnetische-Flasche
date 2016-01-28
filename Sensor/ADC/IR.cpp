/*
 * IR.cpp
 *
 *  Created on: 29.12.2015
 *      Author: pinker
 */

#include "IR.h"


IR::IR(HAL_ADC * adc, ADC_CHANNEL cha) {
	channel = cha;
	sensor = adc;
	totalSamples = 0;
	a = 40.526;
	b = -19.181;
	c = 1.033;
}

IR::~IR() {}

void IR::init() {
	// Sensor->config(ADC_PARAMETER_RESOLUTION, 12);
	sensor->init(channel);
}

float IR::read() {
	float x = readInternal();
	// y = A + Bx + Cx2;
	return a + b * x + c * x * x;
}

float IR::readInternal() {
	int32_t val = sensor->read(channel);
	val = val & 0xFFFFFFF8;     //Removing noise on LSBs
	// 12 bit (4096)/ Revvoltage 5 V
	return ((float) val) / 819; // Calculated somehow
}

void IR::sample(float distance) {
	dist[totalSamples] = distance;
	volt[totalSamples] = readInternal();
	PRINTF("SAMPLE %d  Value = %f       Distance = %f \r\n", totalSamples + 1, volt[totalSamples], dist[totalSamples]);
	totalSamples++;
	if (totalSamples > MAXSAMPLES) {
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
	for (int i = 0; i < MAXSAMPLES; i++) {
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
	xMean = sum_xi / MAXSAMPLES;
	x2mean = sum_xi2 / MAXSAMPLES;
	yMean = yMean / MAXSAMPLES;

	float sxx, sxy, sxx2, sx2x2, sx2y;
	sxx = (sum_xi2 / MAXSAMPLES) - (xMean * xMean);
	sxy = (sum_xiyi / MAXSAMPLES) - (xMean * yMean);
	sxx2 = (sum_xi3 / MAXSAMPLES) - (xMean * x2mean);
	sx2x2 = (sum_xi4 / MAXSAMPLES) - (x2mean * x2mean);
	sx2y = (sum_xi2yi / MAXSAMPLES) - (x2mean * yMean);

	float denom = ((sxx * sx2x2) - (sxx2 * sxx2));
	b = ((sxy * sx2x2) - (sx2y * sxx2)) / denom;
	c = ((sx2y * sxx) - (sxy * sxx2)) / denom;
	a = yMean - b * xMean - c * x2mean;

	PRINTF("Scaling IR... New parameters:\r\n");
	PRINTF("A = %f      B = %f      C = %f \r\n", a, b, c);
}

