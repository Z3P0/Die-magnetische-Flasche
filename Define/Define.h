/*
 * Define.h
 *
 *  Created on: 25.12.2015
 *      Author: pinker
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define RAD_TO_DEG		57.295779513   	//180 / M_PI
#define DEG_TO_RAD		0.0174532925	// M_PI/180

#define RPM_TO_RADS		0.1047197551	// * (2 * M_PI) / 60
#define RADS_TO_RPM		9.5492965855	// * 60 / (2 * M_PI)

#define ABS(a) 			((a>0)?a:-a)	//Returns the absolute value of a

#define int16Max		32768

#endif /* DEFINE_H_ */
