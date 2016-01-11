/*
 * Light.cpp
 *
 *  Created on: 27.12.2015
 *      Author: pinker
 */

#include "Light.h"
#include "../../Define/Define.h"
#include "../../Extern/Extern.h"


Light::Light() {
	init();
}

Light::~Light() {}

void Light::init() {
	PRINTF("Configuring light sensor\r\n");

	//I2C 1 enable and initialize
	I2C_EN.init(true, 1, 1);
	I2C_1.init(i2c_1_speed);

	//Powers the light sensors up
	I2C_1.write(lightAdress, lightCtrlReg, 2);
}

void Light::read() {
	uint8_t data[2];
	I2C_1.writeRead(lightAdress, CHANNEL_0_LOW, 1, data, 2);
	ch0 = ((int16_t) ((data[1] << 8) | data[0]));
	I2C_1.writeRead(lightAdress, CHANNEL_1_LOW, 1, data, 2);
	ch1 = ((int16_t) ((data[1] << 8) | data[0]));

	//PRINTF("LIGHT 1 V: %d IR %d Coeff %f\r\n", ch0, ch1, (ch1/(float)ch0));
}

float Light::getLuxValue() {
	read();
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
