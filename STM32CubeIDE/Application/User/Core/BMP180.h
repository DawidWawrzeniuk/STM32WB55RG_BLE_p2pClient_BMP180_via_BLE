

#ifndef BMPXX80_H_
#define BMPXX80_H_
//
//	Settings
//	Choose sensor
//
#define BMP180

#ifdef BMP180
#define BMP_I2C 1
#endif

//
// I2C address
//
#ifdef BMP180
#define BMP180_I2CADDR	0xEE
#endif


//
//	Mode
//
#ifdef BMP180
#define BMP180_ULTRALOWPOWER	0
#define BMP180_STANDARD			1
#define BMP180_HIGHRES			2
#define BMP180_ULTRAHIGHRES		3
#endif


//
//	Coeffs registers
//
#ifdef BMP180
#define BMP180_CAL_AC1		0xAA  // R   Calibration data (16 bits)
#define BMP180_CAL_AC2		0xAC  // R   Calibration data (16 bits)
#define BMP180_CAL_AC3		0xAE  // R   Calibration data (16 bits)
#define BMP180_CAL_AC4		0xB0  // R   Calibration data (16 bits)
#define BMP180_CAL_AC5		0xB2  // R   Calibration data (16 bits)
#define BMP180_CAL_AC6		0xB4  // R   Calibration data (16 bits)
#define BMP180_CAL_B1		0xB6  // R   Calibration data (16 bits)
#define BMP180_CAL_B2		0xB8  // R   Calibration data (16 bits)
#define BMP180_CAL_MB		0xBA  // R   Calibration data (16 bits)
#define BMP180_CAL_MC		0xBC  // R   Calibration data (16 bits)
#define BMP180_CAL_MD		0xBE  // R   Calibration data (16 bits)
#endif


//
//	Registers
//
#ifdef BMP180
#define	BMP180_CHIPID			0xD0
#define BMP180_VERSION			0xD1
#define BMP180_SOFTRESET		0xE0
#define BMP180_CONTROL			0xF4
#define BMP180_TEMPDATA			0xF6
#define BMP180_PRESSUREDATA		0xF6
#define BMP180_READTEMPCMD		0x2E
#define BMP180_READPRESSURECMD	0x34
#endif



//
//	Control bits
//
#ifdef BMP180
#define	BMP180_SCO			(1<<5) // Start conversion bit (written 0 if conversion done)
#endif


//
// User functions
//
#ifdef BMP180
void BMP180_Init(I2C_HandleTypeDef *i2c_handler, uint8_t mode);

float BMP180_ReadTemperature(void);
int32_t BMP180_ReadPressure(void);

float BMP180_PressureToAltitude(float sea_level, float atmospheric);
float BMP180_SeaLevelForAltitude(float altitude, float atmospheric);
#endif



#endif /* BMPXX80_H_ */
