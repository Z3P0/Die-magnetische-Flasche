/*
 * Camera.h
 *
 *  Created on: Jan 9, 2016
 *      Author: Usama Tariq
 */

#ifndef SENSOR_I2C_CAMERA_H_
#define SENSOR_I2C_CAMERA_H_

class Camera {
public:
	Camera();
	virtual ~Camera();

	void init();

private:
	void initRegisters();
	void initTimer();
};

#endif /* SENSOR_I2C_CAMERA_H_ */
