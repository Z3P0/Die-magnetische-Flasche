#include "rodos.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "../../Define/Define.h"
#include "../../Extern/Extern.h"

extern "C" void DCMI_IRQHandler();

class Camera {
public:
	Camera();
	void init();
	void takePicture();

private:
	void initGPIO();
	void initDCMI();
	void initDMA();
	int initCAM();

};
