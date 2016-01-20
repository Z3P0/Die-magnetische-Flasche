/*
 * Registers.h
 *
 *  Created on: 28.11.2015
 *  Author: 	Z3PO
 *
 *  IMU: 	 	LSM9DS0
 *  Datasheet: 	http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00087365.pdf
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

/*--Gyrometer--*/
#define GYRO_ADDRESS			0x6B

#define GYRO_OUT_ALL_AXIS		0xA8
#define GYRO_WHO_AM_I			0x0F

#define CTRL_REG1_G_ADDRESS 	0x20
#define CTRL_REG1_G_VALUE 		0b01111111

// CTRL_REG1_G
// ----------------------------------------
// Data rate: 			190 HZ/Cut off 70		0111
// Power down: 			normal mode				1
// Z: 					enabled					1
// Y: 					enabled					1
// X: 					enabled					1

#define CTRL_REG4_G_ADDRESS 	0x23
#define CTRL_REG4_G_VALUE 		0b10000000

// CTRL_REG4_G
// ----------------------------------------
// Block data update:(countinous update)		1
// endian selection(Data LSb @ lower adress) 	0
// Full scale selection
// +-245 degrees per second:					00
// -empty-										0
// Self test: 				disabled 			00
// Spi interface: 			4-wire				0

//Angular rate FS= +-245 dps (mdps/ digit) Data sheet: 8.75
#define GYR_SCALING_FACTOR			0.00875		 //mdps

#define CTRL_REG5_G_ADDRESS 	0x24
#define CTRL_REG5_G_VALUE 		0b00010000

//CTRL_REG5_G
//----------------------------------------
//Reboot memory contend: 	normal mode			0
//FIFO: 					disabled			0
//-empty-										0
//High pass filter: 		enabled				1
//Int1_Sel1-Int1_Sel0: 	ADC->LPF1			00
//Out_Sel1-Out_Sel0:		ADC->LPF1			00

/*--Magnetometer/ Acceleometer--*/
#define ACC_MAG_ADDRESS			0x1D

#define MAG_OUT_ALL_AXIS		0x88
#define ACC_OUT_ALL_AXIS 		0xA8

#define WHO_AM_I_XM 			0x0F

#define CTRL_REG1_XM_ADDRESS 	0x20
#define CTRL_REG1_XM_VALUE 		0b01101111

// CTRL_REG1_XM
// ---------------------------------
// Acceleration data rate
// selection: 				200 Hz 	  			0110
// Block data update: 		block data			1
// ACC Z 					enabled				1
// ACC Y 					enabled				1
// ACC X 					enabled				1

#define CTRL_REG2_XM_ADDRESS 	0x21
#define CTRL_REG2_XM_VALUE 		0b00000000

// CTRL_REG2_XM
// ---------------------------------
// Accelerometer anti-alias
// filter bandwidth:		773 Hz				00
// Acceleration full-scale
// selection: 				+-2 g				000
// Acceleration self-test:	disabled			00
// SPI interface: 			4-wire				0

//Linear acceleration FS = ±2 g (mg/LSB) Data sheet: 0.061
#define ACC_SCALING_FACTOR			0.061

#define CTRL_REG5_XM_ADDRESS 	0x24
#define CTRL_REG5_XM_VALUE 		0b01101100

// CTRL_REG5_XM
// ---------------------------------
// Temperature sensor:		enabled				0
// Magnetic resolution: 	high				11
// Magnetic data rate: 		50 Hz				011
// Latch up1 interrupt:		not latched			0
// Latch up2 interrupt:		not latched			0

#define CTRL_REG6_XM_ADDRESS 	0x25
#define CTRL_REG6_XM_VALUE 		0b00100000

// CTRL_REG6_XM
// ---------------------------------
// -empty-										0
// Magnetic full-scale selection +-4 g 			01
// -empty-										0
// -empty-										0
// -empty-										0
// -empty-										0
// -empty-										0

//Magnetic FS = ±2 gauss 	(mgauss/ LSB) Data sheet: 0.08
#define MAG_SCALING_FACTOR		0.08

#define CTRL_REG7_XM_ADDRESS 	0x26
#define CTRL_REG7_XM_VALUE 		0b00000000

// CTRL_REG7_XM
// ---------------------------------
// High-pass filter mode (normal)				00
// Filtered data selection: 					0
// -empty-										0
// -empty-										0
// internal filter bypassed						0
// Magnetic data rate controlled by REG6		0
// Magnetic sensor mode selection:	normal mode	0



#endif /* REGISTERS_H_ */
