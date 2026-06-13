#include "main.h"
#include "BMP180.h"
#include "math.h"

#ifdef BMP180
#include "delays.h"
#endif

//
//	 Private variables
//
#if(BMP_I2C == 1)
I2C_HandleTypeDef *i2c_h;
#endif
#if(BMP_SPI == 1)
SPI_HandleTypeDef *spi_h;
#endif

#ifdef BMP180
uint8_t oversampling;
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
#endif


//
//	Functions
//
#ifdef BMP180
uint8_t BMP180_Read8(uint8_t addr)
{
  uint8_t tmp=7;

  HAL_I2C_Mem_Read(i2c_h, BMP180_I2CADDR, addr, 1, &tmp, 1, 10);

  return tmp;
}
#endif



#ifdef BMP180
uint16_t BMP180_Read16(uint8_t addr)
{

	uint8_t tmp[2];

	HAL_I2C_Mem_Read(i2c_h, BMP180_I2CADDR, addr, 1, tmp, 2, 10);

	return ((tmp[0] << 8) | tmp[1]);
}
#endif







#ifdef BMP180
void BMP180_Write8(uint8_t address, uint8_t data)
{
	HAL_I2C_Mem_Write(i2c_h, BMP180_I2CADDR, address, 1, &data, 1, 10);
}
#endif






uint16_t BMP180_readRawTemperature()
{
	  BMP180_Write8(BMP180_CONTROL, BMP180_READTEMPCMD);
	  while((BMP180_Read8(BMP180_CONTROL) & BMP180_SCO));
//	  Delay_us(5000);
	  return BMP180_Read16(BMP180_TEMPDATA);
}



int32_t BMP180_computeB5(int32_t UT)
{
	  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
	  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
	  return X1 + X2;
}



int32_t BMP180_readRawPressure()
{
	  uint32_t raw;

	  BMP180_Write8(BMP180_CONTROL, BMP180_READPRESSURECMD + (oversampling << 6));

	  if (oversampling == BMP180_ULTRALOWPOWER)
		while((BMP180_Read8(BMP180_CONTROL) & BMP180_SCO));
//		Delay_us(5000);
	  else if (oversampling == BMP180_STANDARD)
		while((BMP180_Read8(BMP180_CONTROL) & BMP180_SCO));
//		Delay_us(8000);
	  else if (oversampling == BMP180_HIGHRES)
		while((BMP180_Read8(BMP180_CONTROL) & BMP180_SCO));
//		Delay_us(14000);
	  else
		while((BMP180_Read8(BMP180_CONTROL) & BMP180_SCO));
//		Delay_us(26000);

	  raw = BMP180_Read16(BMP180_PRESSUREDATA);

	  raw <<= 8;
	  raw |= BMP180_Read8(BMP180_PRESSUREDATA+2);
	  raw >>= (8 - oversampling);

	  return raw;
}


void BMP180_Init(I2C_HandleTypeDef *i2c_handler, uint8_t mode)
{
	i2c_h = i2c_handler;

	if (mode > BMP180_ULTRAHIGHRES)
	    mode = BMP180_ULTRAHIGHRES;
	  oversampling = mode;

	  while(BMP180_Read8(BMP180_CHIPID) != 0x55);

	  /* read calibration data */
	  ac1 = BMP180_Read16(BMP180_CAL_AC1);
	  ac2 = BMP180_Read16(BMP180_CAL_AC2);
	  ac3 = BMP180_Read16(BMP180_CAL_AC3);
	  ac4 = BMP180_Read16(BMP180_CAL_AC4);
	  ac5 = BMP180_Read16(BMP180_CAL_AC5);
	  ac6 = BMP180_Read16(BMP180_CAL_AC6);

	  b1 = BMP180_Read16(BMP180_CAL_B1);
	  b2 = BMP180_Read16(BMP180_CAL_B2);

	  mb = BMP180_Read16(BMP180_CAL_MB);
	  mc = BMP180_Read16(BMP180_CAL_MC);
	  md = BMP180_Read16(BMP180_CAL_MD);
}


float BMP180_ReadTemperature(void)
{
	  int32_t UT, B5;     // following ds convention
	  float temp;

	  UT = BMP180_readRawTemperature();

	  B5 = BMP180_computeB5(UT);
	  temp = (B5+8) >> 4;
	  temp /= 10;

	  return temp;
}






int32_t BMP180_ReadPressure(void)
{
	  int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	  uint32_t B4, B7;

	  UT = BMP180_readRawTemperature();
	  UP = BMP180_readRawPressure();

	  B5 = BMP180_computeB5(UT);

	  // do pressure calcs
	  B6 = B5 - 4000;
	  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
	  X2 = ((int32_t)ac2 * B6) >> 11;
	  X3 = X1 + X2;
	  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;



	  X1 = ((int32_t)ac3 * B6) >> 13;
	  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
	  X3 = ((X1 + X2) + 2) >> 2;
	  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
	  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );


	  if (B7 < 0x80000000) {
	    p = (B7 * 2) / B4;
	  } else {
	    p = (B7 / B4) * 2;
	  }
	  X1 = (p >> 8) * (p >> 8);
	  X1 = (X1 * 3038) >> 16;
	  X2 = (-7357 * p) >> 16;


	  p = p + ((X1 + X2 + (int32_t)3791)>>4);

	  return p;
}








float BMP180_PressureToAltitude(float sea_level, float atmospheric)
{
	return 44330.0 * (1.0 - pow(atmospheric / sea_level, 0.1903));
}



float BMP180_SeaLevelForAltitude(float altitude, float atmospheric)
{
	return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}




