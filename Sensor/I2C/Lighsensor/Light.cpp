/*
 * Light.cpp
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#include "Light.h"
#include "../../../Define/Define.h"
#include "../../../Extern/Extern.h"

Light::Light() {
}

Light::~Light() {}


//TODO Set integration time
void Light::init() {
	PRINTF("Configuring light sensor\r\n");
	// Powers the light sensors up
	I2C_1.write(lightAdress, lightCtrlReg, 2);
}

/*
 * Error check by the number of the returned bytes
 * @return 0 no error
 * @return -1 error
 */
int16_t Light::read() {
	uint8_t data[2];
	if(I2C_1.writeRead(lightAdress, CHANNEL_0_LOW, 1, data, 2) != 2)
		return -1;

	ch0 = ((int16_t) ((data[1] << 8) | data[0]));

	if(I2C_1.writeRead(lightAdress, CHANNEL_1_LOW, 1, data, 2) != 2)
		return -1;

	ch1 = ((int16_t) ((data[1] << 8) | data[0]));

	return 0;
}

/*
 * Merged and calculates the lux values how its dicribed in the datasheet
 * @return lux value
 */
float Light::getLuxValue() {
	// Read and check for errors.
	if(read() == -1)
		PRINTF("Light sensor read error\r\n");

	float coeff = (ch1/(float)ch0);

	if (coeff <= 0.52){
		return (0.0315 * ch0 - 0.0593 * ch0 * (pow(coeff, 1.4)));
	}else if (coeff <= 0.65){
		return (0.0229 * ch0 - 0.0291 * ch1);
	}else if (coeff <= 0.80){
		return (0.0157 * ch0 - 0.0180 * ch1);
	}else if (coeff <= 1.30){
		return (0.00338 * ch0 - 0.00260 * ch1);
	}
	return 0;
}
