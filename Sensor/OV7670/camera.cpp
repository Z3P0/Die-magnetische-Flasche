
#include "camera.h"
#include "../../Extern/Include.h"


bool image = false;
long long periodeCAM;
bool statusCAM;
bool camTrigger;

int row;

uint8_t initReg[][2] = {

				{0x12, 0x80}, // COM7 default: 0x00		reset
				{0x11, 0x00}, // CLKRC default: 0x80	30 fps
				{0x3A, 0x04}, // TSLB default: 0x0D
				{0x12, 0x00}, // COM7 default: 0x00		VGA

				/* Set the hardware window */
				{0x17, 0x39}, // HSTART default: 0x11
				{0x18, 0x03}, // HSTOP default: 0x61
				{0x32, 0x92}, // HREF default: 0x80

				{0x19, 0x03}, // VSTRT default: 0x03
				{0x1A, 0x7B}, // VSTOP default: 0x7B
				{0x03, 0x0A}, // VREF default: 0x00

				{0x0C, 0x0C}, // COM3 default: 0x00
				{0x3E, 0x11}, //0b00011001}, // COM14 default: 0x00

				/* Mystery scaling numbers */
				{0x70, 0x3A}, // SCALING_XSC default: 0x3A
				{0x71, 0x35}, // SCALING_YSC default: 0x35
				{0x72, 0x11}, // SCALING_DCWCTR default: 0x11
				{0x73, 0xF1}, // SCALING_PCLK_DIV default: 0x00 F1
				{0xA2, 0x52}, // SCALING_PCLK_DELAY default: 0x02
				{0x15, 0x00}, // COM10 default: 0x00

				/* Gamma curve values */
				{0x7A, 0x20}, // SLOP default: 0x24
				{0x7B, 0x1C}, // GAM1 default: 0x04
				{0x7C, 0x28}, // GAM2 default: 0x07
				{0x7D, 0x3C}, // GAM3 default: 0x10
				{0x7E, 0x5A}, // GAM4 default: 0x28
				{0x7F, 0x68}, // GAM5 default: 0x36
				{0x80, 0x76}, // GAM6 default: 0x44
				{0x81, 0x80}, // GAM7 default: 0x52
				{0x82, 0x88}, // GAM8 default: 0x60
				{0x83, 0x8F}, // GAM9 default: 0x6C
				{0x84, 0x96}, // GAM10 default: 0x78
				{0x85, 0xA3}, // GAM11 default: 0x8C
				{0x86, 0xAF}, // GAM12 default: 0x9E
				{0x87, 0xC4}, // GAM13 default: 0xBB
				{0x88, 0xD7}, // GAM14 default: 0xD2
				{0x89, 0xE8}, // GAM15 default: 0xE5

				/* AGC and AEC parameters. Disable -> Tweak values -> Enable */
				{0x13, 0x80 | 0x40 | 0x20}, // COM8 default: 0x8F
				{0x00, 0x00}, // GAIN default: 0x00
				{0x10, 0x00}, // AECH default: 0x40
				{0x0D, 0x40}, // COM4 default: 0x00
				{0x14, 0x28}, // COM9 default: 0x4A		4x gain
				{0xA5, 0x05}, // BD50MAX default: 0x0F
				{0xAB, 0x07}, // BD60MAX default: 0x0F
				{0x24, 0x95}, // AEW default: 0x75
				{0x25, 0x33}, // AEB default: 0x63
				{0x26, 0xE3}, // VPT default: 0xD4
				{0x9F, 0x78}, // HAECC1 default: 0xC0
				{0xA0, 0x68}, // HAECC2 default: 0x90
				{0xA1, 0x0B}, // RSVD					magic
				{0xA6, 0xD8}, // HAECC3 default: 0xF0
				{0xA7, 0xD8}, // HAECC4 default: 0xC1
				{0xA8, 0xF0}, // HAECC5 default: 0xF0
				{0xA9, 0x90}, // HAECC6 default: 0xC1
				{0xAA, 0x94}, // HAECC7 default: 0x14
				{0x13, 0x80 | 0x40 | 0x20 | 0x04 | 0x01}, // COM8 default: 0x8F

				/* Magic reserved values */
				{0x0E, 0x61}, // COM5 default: 0x01
				{0x0F, 0x4B}, // COM6 default: 0x43
				{0x16, 0x02}, // RSVD
				{0x21, 0x02}, // ADCCTR1 default: 0x02
				{0x22, 0x91}, // ADCCTR2 default: 0x01
				{0x29, 0x07}, // RSVD
				{0x33, 0x03}, // CHLF default: 0x08
				{0x35, 0x0B}, // RSVD
				{0x37, 0x1C}, // ADC default: 0x3F
				{0x38, 0x71}, // ACOM default: 0x01
				{0x3C, 0x78}, // COM12 default: 0x68
				{0x4D, 0x40}, // RSVD
				{0x4E, 0x20}, // RSVD
				{0x69, 0x55}, // GFIX default: 0x00
				{0x6B, 0x4A}, // DBLV default: 0x0A
				{0x74, 0x19}, // REG74 default: 0x00
				{0x8D, 0x4F}, // RSVD
				{0x8E, 0x00}, // RSVD
				{0x8F, 0x00}, // RSVD
				{0x90, 0x00}, // RSVD
				{0x91, 0x00}, // RSVD
				{0x96, 0x00}, // RSVD
				{0x9A, 0x80}, // RSVD
				{0xB0, 0x8C}, // RSVD
				{0xB1, 0x0C}, // ABLC1 default: 0x00
				{0xB2, 0x0E}, // RSVD
				{0xB3, 0x82}, // THL_ST default: 0x80
				{0xB8, 0x0A}, // RSVD

				/* Reserved magic (white balance) */
				{0x43, 0x14}, // AWBC1 default: 0x14
				{0x44, 0xF0}, // AWBC2 default: 0xF0
				{0x45, 0x34}, // AWBC3 default: 0x45
				{0x46, 0x58}, // AWBC4 default: 0x61
				{0x47, 0x28}, // AWBC5 default: 0x51
				{0x48, 0x3A}, // AWBC6 default: 0x79
				{0x59, 0x88}, // RSVD
				{0x5A, 0x88}, // RSVD
				{0x5B, 0x44}, // RSVD
				{0x5C, 0x67}, // RSVD
				{0x5D, 0x49}, // RSVD
				{0x5E, 0x0E}, // RSVD
				{0x6C, 0x0A}, // AWBCTR3 default: 0x02
				{0x6D, 0x55}, // AWBCTR2 default: 0x55
				{0x6E, 0x11}, // AWBCTR1 default: 0xC0
				{0x6F, 0x9F}, // AWBCTR0 default: 0x9A (0x9E for advance AWB)
				{0x6A, 0x40}, // GGAIN default: 0x00
				{0x01, 0x40}, // BLUE default: 0x80
				{0x02, 0x40}, // RED default: 0x80
				{0x13, 0x80 | 0x40 | 0x20 | 0x01}, // COM8 default: 0x8F

				/* Matrix coefficients */
				{0x4F, 0x80}, // MTX1 default: 0x40
				{0x50, 0x80}, // MTX2 default: 0x34
				{0x51, 0x00}, // MTX3 default: 0x0C
				{0x52, 0x22}, // MTX4 default: 0x17
				{0x53, 0x5E}, // MTX5 default: 0x29
				{0x54, 0x80}, // MTX6 default: 0x40
				{0x58, 0x9E}, // MTXS default: 0x1E

				{0x41, 0x08}, // COM16 default: 0x08
				{0x3F, 0x00}, // EDGE default: 0x00
				{0x75, 0x05}, // REG75 default: 0x0F
				{0x76, 0x61}, // REG76 default: 0x01
				{0x4C, 0x00}, // DNSTH default: 0x00
				{0x77, 0x01}, // REG77 default: 0x10
				{0x3D, 0xC2}, // COM13 default: 0x88
				{0x4B, 0x09}, // REG4B default: 0x00
				{0xC9, 0x60}, // SATCTR default: 0xC0
				{0x41, 0x38}, // COM16 default: 0x08
				{0x56, 0x40}, // CONTRAS default: 0x40

				{0x34, 0x11}, // ARBLM default: 0x11
				{0x3B, 0x02}, // COM11 default: 0x00
				{0xA4, 0x89}, // NT_CTRL default: 0x00
				{0x96, 0x00}, // RSVD
				{0x97, 0x30}, // RSVD
				{0x98, 0x20}, // RSVD
				{0x99, 0x20}, // RSVD
				{0x9A, 0x84}, // RSVD
				{0x9B, 0x29}, // RSVD
				{0x9C, 0x03}, // RSVD
				{0x9D, 0x4C}, // BD50ST default: 0x99
				{0x9E, 0x3F}, // BD60ST default: 0x7F
				{0x78, 0x04}, // RSVD

				/* Extra-weird stuff (sort of multiplexor register) */
				{0x79, 0x01}, // RSVD
				{0xC8, 0xF0}, // RSVD
				{0x79, 0x0F}, // RSVD
				{0xC8, 0x20}, // RSVD
				{0x79, 0x10}, // RSVD
				{0xC8, 0x7E}, // RSVD
				{0x79, 0x0B}, // RSVD
				{0xC8, 0x01}, // RSVD
				{0x79, 0x0C}, // RSVD
				{0xC8, 0x07}, // RSVD
				{0x79, 0x0D}, // RSVD
				{0xC8, 0x20}, // RSVD
				{0x79, 0x09}, // RSVD
				{0xC8, 0x80}, // RSVD
				{0x79, 0x02}, // RSVD
				{0xC8, 0xC0}, // RSVD
				{0x79, 0x03}, // RSVD
				{0xC8, 0x40}, // RSVD
				{0x79, 0x05}, // RSVD
				{0xC8, 0x30}, // RSVD
				{0x79, 0x26}, // RSVD

				{0x19, 0x02}, // VSTRT default: 0x03
				{0x03, 0x0C}, // VREF default: 0x00
				{0x1A, 0x7C}, // VSTOP default: 0x7B

		#ifdef YUV422
				{0x12, 0x00}, // COM7			Selects YUV mode
				{0x8C, 0x00}, // RGB444			No RGB444 please
				{0x04, 0x00}, // COM1			CCIR601
				{0x40, 0xC0}, // COM15			[00] to [FF]
				{0x14, 0x48}, // COM9			32x gain ceiling
				{0x4F, 0x80}, // MTX1
				{0x50, 0x80}, // MTX2
				{0x51, 0x00}, // MTX3
				{0x52, 0x22}, // MTX4
				{0x53, 0x5E}, // MTX5
				{0x54, 0x80}, // MTX6
				{0x3D, 0x80 | 0x40}, // COM13
		#endif
				{0xFF, 0xFF}, // END MARKER
};

Camera::Camera(){}

void Camera::init() {
	initGPIO();
	initDCMI();
	initDMA();
	initCAM();
}

/**
 * Init all GPIOs
 */

void Camera::initGPIO() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(
			RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC
					| RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PCLK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // VSYNC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // HREF
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); // D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); // D1
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI); // D2
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI); // D3
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); // D4
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); // D5
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); // D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); // D7

	//Struct for Port-A (HREF, PCLK)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Struct for Port-B (VSYNC, D5)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Struct for Port-C (D0, D1)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//Struct for Port-E (D2,D3,D5,D6,D7)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Struct for PA10 (RESET)
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource10;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//Struct for PC1(PowerDown)
	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource1;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	// EXTCLK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCO1Config(RCC_MCO1Source_HSI, RCC_MCO1Div_1);

}
/**
 * INIT DCMI
 */
void Camera::initDCMI() {
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);
	DCMI_InitTypeDef DCMI_InitStructure;
	DCMI_DeInit();
	DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_Low;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Rising;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;

	DCMI_Init(&DCMI_InitStructure);


	DCMI_CROPInitTypeDef DCMI_CROPInitStructure;

	DCMI_CROPInitStructure.DCMI_HorizontalOffsetCount = 0;//
	DCMI_CROPInitStructure.DCMI_CaptureCount = 2*(WIDTH)-1; //
    DCMI_CROPInitStructure.DCMI_VerticalStartLine = 0; //
	DCMI_CROPInitStructure.DCMI_VerticalLineCount = HEIGHT-1; //

        DCMI_CROPConfig(&DCMI_CROPInitStructure);

        DCMI_CROPCmd(ENABLE);

}
/*
 * Init "Direct Memory Access" and "Nested Vectored Interrupt Controller"
 */
void Camera::initDMA() {
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_BufferSize = IMAGESIZE/ 4;
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &DCMI_Buffer;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = 0x50050028;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;

	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = DCMI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;

	NVIC_Init(&NVIC_InitStructure);
}

/**
 * Setting all registers
 */
int Camera::initCAM() {
	int result;
	AT(NOW() + 10 * MILLISECONDS);

	uint8_t reg_reset[2] = { 0x12, 0x80 };
	//reseting the cam
	result = I2C_1.write(CAMADR, reg_reset, sizeof(reg_reset));
	AT(NOW() + 10*MILLISECONDS);

	if (result < 0) {
		I2C_1.init();
	}

	reg_reset[1] = 0x00;
	result = I2C_1.write(CAMADR, reg_reset, sizeof(reg_reset));
	if (result < 0) {
		I2C_1.init();
	}
	AT(NOW() + 10*MILLISECONDS);

	int totalReg = sizeof(initReg) / 2;
	for (int i = 0; i < totalReg; i++) {
		result = I2C_1.write(CAMADR, initReg[i], sizeof(initReg[i]));
		AT(NOW() + 2*MILLISECONDS);
		if (result < 0) {
#ifdef CAM_DEBUG
			PRINTF("Couldnt write register:%d result: %d\n", i, result);
#endif
		}
#ifdef CAM_DEBUG
		PRINTF("Writing all registers");
#endif
	}
	return result;
}

/**
 * Taking Picture
 */
void Camera::takePicture() {
	DCMI_ITConfig(DCMI_IT_FRAME, ENABLE);
	DCMI_ITConfig(DCMI_IT_OVF, ENABLE);
	DCMI_ITConfig(DCMI_IT_ERR, ENABLE);

    //DCMI_ITConfig(DCMI_IT_VSYNC, ENABLE);
    //DCMI_ITConfig(DCMI_IT_LINE, ENABLE);

	DMA_Cmd(DMA2_Stream1, ENABLE);
	DCMI_Cmd(ENABLE);
	DCMI_CaptureCmd(ENABLE);
	AT(NOW() + 10*MILLISECONDS);
	image = false;
}

/**
 * Interrupt Handler for DCMI
 */
void DCMI_IRQHandler() {

	if (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == SET) {
		DCMI_ClearFlag(DCMI_FLAG_FRAMERI);
	} else if (DCMI_GetFlagStatus(DCMI_FLAG_OVFRI) == SET) {
		DCMI_ClearFlag(DCMI_FLAG_OVFRI);
	} else if (DCMI_GetFlagStatus(DCMI_FLAG_ERRRI) == SET) {
		DCMI_ClearFlag(DCMI_FLAG_ERRRI);
	}
	DCMI_CaptureCmd(DISABLE);
	DCMI_Cmd(DISABLE);
	DMA_Cmd(DMA2_Stream1, DISABLE);
	DCMI_Cmd(DISABLE);
	DCMI_ITConfig(DCMI_IT_FRAME, DISABLE);

	image = true;
}

