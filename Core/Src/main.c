/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "i2c_user.h"
#include "ds18b20.h"
#include "bme280.h"
#include "w25qxx.h"
#include "w25qxxConf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
unsigned int counter = 0;
unsigned int MaxCounter = 10;
unsigned int Valcounter = 0;
unsigned int MaxValCounter = 10;
unsigned int SubMenu = 0;
unsigned int MaxSubMenu = 10;
unsigned int Menu = 0;
unsigned int MaxMenu = 10;
unsigned int MainMenu = 0;
unsigned int MaxMainMenu = 10;

#define MaxCells 4

int EncoderVal =0;
#define true 1
#define false -1
int chouse = false;
#define ButtonMenu 0
#define Buttonincrease 8
#define Buttondecrease 2
#define Buttonchoise 5
int x = 0;
#define EncoderVal x

/* EEPROM function prototypes -----------------------------------------------*/
void    SaveSettinds(int cell);
void    LoadSettinds(int cell);
uint8_t intToByte(int n);
int     byteToInt(uint8_t byte);
int16_t float16ToByte (int16_t argn);
float   byteToFloat16 (unsigned char argb[]);
void    AT24C_WriteBytes (uint16_t addr,uint8_t *buf, uint16_t bytes_count);
uint8_t AT24C_ReadBytes (uint16_t addr, uint8_t *buf, uint16_t bytes_count);
/* Get function prototypes -----------------------------------------------*/
void    GetTemp();
float   GetBME280(int a);
float   GetSelection(int Time);
/* Control function prototypes -----------------------------------------------*/
float   GetSelection();
float   GetVarporization();
void    ControlEHE(int power);
int     ExtraInterapt();
/* Settings function prototypes -----------------------------------------------*/
void    menu();
long    map();
void    Rectification(int a);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t intToByte(int n) {

  unsigned char byte[4];

     byte[0] = n & 0x000000ff;
     byte[1] = n & 0x0000ff00 >> 8;
     byte[2] = n & 0x00ff0000 >> 16;
     byte[3] = n & 0xff000000 >> 24;

     return byte;
}
//-------------------------------------------------------------------------------------------------
int byteToInt(unsigned char byte[4]) {

    int n = 0;

    n = n + (byte[0] & 0x000000ff);
    n = n + ((byte[1] & 0x000000ff) << 8);
    n = n + ((byte[2] & 0x000000ff) << 16);
    n = n + ((byte[3] & 0x000000ff) << 24);

    return n;
}

//-------------------------------------------------------------------------------------------------
/*uint8_t float8Tobyte (int16_t argn)
{
unsigned char bytes[2];
bytes[0] = (argn >> 8) & 0xFF;
bytes[1] = argn & 0xFF;
printf("0x%02x 0x%02x\n", (unsigned char)bytes[0],
                        (unsigned char)bytes[1]);

return bytes
}

//-------------------------------------------------------------------------------------------------
float byteToFloat8 (unsigned char argb[])
{
int16_t integers;
integers = (argb[0] <<8) + argb[1];
//printf("%d\n", (int16_t) integers);
return integers
}
 */
//-------------------------------------------------------------------------------------------------
int16_t float16ToByte (int16_t argn)
{
unsigned char bytes[4];
bytes[0] = (argn >> 24) & 0xFF;
bytes[1] = (argn >> 16) & 0xFF;
bytes[2] = (argn >> 8) & 0xFF;
bytes[3] = argn & 0xFF;
/*printf("0x%x 0x%x 0x%x 0x%x\n", (unsigned char)bytes[0],
                        (unsigned char)bytes[1],
                        (unsigned char)bytes[2],
                        (unsigned char)bytes[3]);
*/
return bytes
}

//-------------------------------------------------------------------------------------------------
float byteToFloat16 (unsigned char argb[])
{
int32_t integers;
integers = (argb[0] <<24) + (argb[1] <<16) + (argb[2] <<8) + argb[3];
/*printf("%d\n", (int32_t) integers);
printf("0x%x\n", (int32_t) integers);
*/
return integers
}

//-------------------------------------------------------------------------------------------------
void SaveSettinds(int cell) {
  AT24C_WriteBytes(800*cell+0 ,float16ToByte (TempTanker),4);
  AT24C_WriteBytes(800*cell+4 ,float16ToByte (TempDrawer),4);
  AT24C_WriteBytes(800*cell+8 ,float16ToByte (TempTakeOff),4);
  AT24C_WriteBytes(800*cell+12,float16ToByte (TempReflux),4);
  AT24C_WriteBytes(800*cell+16,float16ToByte (TempRefrigerator),4);
  AT24C_WriteBytes(800*cell+20,float16ToByte (TempAmbient),4);
  AT24C_WriteBytes(800*cell+24,intToByte (VMainTank),4);
  AT24C_WriteBytes(800*cell+28,intToByte (VRawMaterial),4);
  AT24C_WriteBytes(800*cell+32,intToByte (VlightEndTank),4);
  AT24C_WriteBytes(800*cell+36,intToByte (VLightComTank),4);
  AT24C_WriteBytes(800*cell+40,intToByte (VMainFrTank),4);
  AT24C_WriteBytes(800*cell+44,intToByte (VMainFrResTank),4);
  AT24C_WriteBytes(800*cell+48,intToByte (VHeavyComTank),4);
  AT24C_WriteBytes(800*cell+52,intToByte (VHeavyTank),4);
  AT24C_WriteBytes(800*cell+56,float16ToByte (RawMaterialPersent),4);
  AT24C_WriteBytes(800*cell+60,float16ToByte (PersentSolvent),4);
  AT24C_WriteBytes(800*cell+64,float16ToByte (MolarSolvent),4);
  AT24C_WriteBytes(800*cell+68,float16ToByte (MolarMain),4);
  AT24C_WriteBytes(800*cell+72,float16ToByte (Vaporization),4);
  AT24C_WriteBytes(800*cell+76,float16ToByte (PowerEHE),4);
  for(int i=0,i<6,i++){
      AT24C_WriteBytes(800*cell+80+4*i,float16ToByte (SensorT[i]),4);
      }

  for(int i=0,i<100,i++){
      AT24C_WriteBytes(800*cell+104+4*i,float16ToByte (Selection[i]),4);
      }
}
//-------------------------------------------------------------------------------------------------
void LoadSettinds(int cell) {
    byte buf[4];
  TempTanker         = byteToFloat16(AT24C_ReadBytes (800*cell+0 ,buf,4));
  TempDrawer         = byteToFloat16(AT24C_ReadBytes (800*cell+4 ,buf,4));
  TempTakeOff        = byteToFloat16(AT24C_ReadBytes (800*cell+8 ,buf,4));
  TempReflux         = byteToFloat16(AT24C_ReadBytes (800*cell+12,buf,4));
  TempRefrigerator   = byteToFloat16(AT24C_ReadBytes (800*cell+16,buf,4));
  TempAmbient        = byteToFloat16(AT24C_ReadBytes (800*cell+20,buf,4));
  VMainTank          = byteToInt    (AT24C_ReadBytes (800*cell+24,buf,4));
  VRawMaterial       = byteToInt    (AT24C_ReadBytes (800*cell+28,buf,4));
  VlightEndTank      = byteToInt    (AT24C_ReadBytes (800*cell+32,buf,4));
  VLightComTank      = byteToInt    (AT24C_ReadBytes (800*cell+36,buf,4));
  VMainFrTank        = byteToInt    (AT24C_ReadBytes (800*cell+40,buf,4));
  VMainFrResTank     = byteToInt    (AT24C_ReadBytes (800*cell+44,buf,4));
  VHeavyComTank      = byteToInt    (AT24C_ReadBytes (800*cell+48,buf,4));
  VHeavyTank         = byteToInt    (AT24C_ReadBytes (800*cell+52,buf,4));
  RawMaterialPersent = byteToFloat16(AT24C_ReadBytes (800*cell+56,buf,4));
  PersentSolvent     = byteToFloat16(AT24C_ReadBytes (800*cell+60,buf,4));
  MolarSolvent       = byteToFloat16(AT24C_ReadBytes (800*cell+64,buf,4));
  MolarMain          = byteToFloat16(AT24C_ReadBytes (800*cell+68,buf,4));
  Vaporization       = byteToFloat16(AT24C_ReadBytes (800*cell+72,buf,4));
  PowerEHE           = byteToFloat16(AT24C_ReadBytes (800*cell+76,buf,4));

  for(int i=0,i<6,i++){
    SensorT[i]=      = byteToInt    (AT24C_ReadBytes (800*cell+80+4*i,buf[],4));
}
  for(int i=0,i<100,i++){
    Selection[i]     = byteToFloat16(AT24C_ReadBytes (800*cell+104+4*i,buf[],4));
}
}

//-------------------------------------------------------------------------------------------------
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//-------------------------------------------------------------------------------------------------
void menu(){

MaxMainMenu = 1;
 switch (MainMenu)
{
  case 0 :
LCD_SetPos(1,0);LCD_String("Start Rectification");LCD_SetPos(2,0);LCD_String("Cell = ");LCD_String(Valcounter); MaxValCounter = MaxCells;
if(chouse == true ){Rectification(Valcounter); break;} else break;

if (EncoderVal == ButtonMenu)break;

  case 1:
MaxMenu = 3;
switch (Menu)
{   case 0:
LCD_SetPos(1,0);LCD_String("Settings");
MaxSubMenu = 7;
switch (SubMenu)
{
case 0:LCD_SetPos(1,0);LCD_String("Main Settings");
MaxCounter = 6;
switch(counter)
{

case 0:LCD_SetPos(1,0);LCD_String("Power EHE (kW)");LCD_SetPos(2,0);LCD_String(PowerEHE); MaxValCounter = MaxPowerEHE;
if(chouse == true ){PowerEHE = Valcounter; break;} else break;

case 1:LCD_SetPos(1,0);LCD_String("Raw Material Volume");LCD_SetPos(2,0);LCD_String(VRawMaterial); MaxValCounter = MaxVMainTank;
if(chouse == true ){VRawMaterial = Valcounter; break;} else break;

case 2:LCD_SetPos(1,0);LCD_String("Raw Material Persent");LCD_SetPos(2,0);LCD_String(RawMaterialPersent); MaxValCounter = 100;
if(chouse == true ){RawMaterialPersent = Valcounter; break;} else break;

case 3:LCD_SetPos(1,0);LCD_String("Raw Material Mr");LCD_SetPos(2,0);LCD_String(TempTanker); MaxValCounter = MaxMr;
if(chouse == true ){TempTanker = Valcounter; break;} else break;

case 4:LCD_SetPos(1,0);LCD_String("Solvent Persent");LCD_SetPos(2,0);LCD_String(PersentSolvent); MaxValCounter = 100 - RawMaterialPersent;
if(chouse == true ){PersentSolvent = Valcounter; break;} else break;

case 5:LCD_SetPos(1,0);LCD_String("Solver Mr");LCD_SetPos(2,0);LCD_String(MolarSolvent); MaxValCounter = MaxTempTanker;
if(chouse == true ){MolarSolvent = Valcounter; break;} else break;

case 6:LCD_SetPos(1,0);LCD_String("Energy Vaporization");LCD_SetPos(2,0);LCD_String(Vaporization); MaxValCounter = MaxVaporization;
if(chouse == true ){Vaporization = Valcounter; break;} else break;

}
if (EncoderVal == ButtonMenu)break;


case 1:LCD_SetPos(1,0);LCD_String("Tempeture Parametr");
MaxSubMenu = 5;
switch(counter)
{
case 0:LCD_SetPos(1,0);LCD_String("Tanker Temp");LCD_SetPos(2,0);LCD_String(TempTanker); MaxValCounter = MaxTempTanker;
if(chouse == true ){TempTanker = Valcounter; break;} else break;

case 1:LCD_SetPos(1,0);LCD_String("Drawer Temp");LCD_SetPos(2,0);LCD_String(TempDrawer); MaxValCounter = MaxTempDrawer;
if(chouse == true ){TempDrawer = Valcounter; break;} else break;

case 2:LCD_SetPos(1,0);LCD_String("Take off Temp");LCD_SetPos(2,0);LCD_String(TempTakeOff); MaxValCounter = MaxTempTakeOff;
if(chouse == true ){TempTakeOff = Valcounter; break;} else break;

case 3:LCD_SetPos(1,0);LCD_String("RefluxCondenser Temp");LCD_SetPos(2,0);LCD_String(TempReflux); MaxValCounter = MaxTempReflux;
if(chouse == true ){TempReflux = Valcounter; break;} else break;

case 4:LCD_SetPos(1,0);LCD_String("Refrigerator Temp");LCD_SetPos(2,0);LCD_String(TempRefrigerator); MaxValCounter = MaxTempRefrigerator;
if(chouse == true ){TempRefrigerator = Valcounter; break;} else break;

case 5:LCD_SetPos(1,0);LCD_String("Ambient Temp");LCD_SetPos(2,0);LCD_String(TempAmbient); MaxValCounter = MaxTempAmbient;
if(chouse == true ){TempAmbient = Valcounter; break;} else break;

}

if (EncoderVal == ButtonMenu)break;

case 2:LCD_SetPos(1,0);LCD_String("Selection Parametr");
   MaxSubMenu = 100;
   for(counter;;){
LCD_SetPos(1,0);LCD_String(Selection[counter]);LCD_SetPos(1,3);LCD_String("%  selection");LCD_SetPos(2,0);LCD_String(Selection[counter]); MaxValCounter = MaxSelection;
if(chouse == true ){Selection[counter] = Valcounter; break;} else break;
}

}
if (EncoderVal == ButtonMenu)break;

case 3:LCD_SetPos(1,0);LCD_String("Tanker Volume");
MaxSubMenu = 5;
switch(counter)
{
case 0:LCD_SetPos(1,0);LCD_String("Main tank");LCD_SetPos(2,0);LCD_String(VMainTank); MaxValCounter = MaxVMainTank;
if(chouse == true ){VMainTank = Valcounter; break;} else break;

case 1:LCD_SetPos(1,0);LCD_String("Light Ends Tank");LCD_SetPos(2,0);LCD_String(VlightEndTank); MaxValCounter = MaxVlightEndTank;
if(chouse == true ){VlightEndTank = Valcounter; break;} else break;

case 2:LCD_SetPos(1,0);LCD_String("LightCommercial Tank");LCD_SetPos(2,0);LCD_String(VLightComTank); MaxValCounter = MaxVLightComTank;
if(chouse == true ){VLightComTank = Valcounter; break;} else break;

case 3:LCD_SetPos(1,0);LCD_String("Main Fraction Tank");LCD_SetPos(2,0);LCD_String(VMainFrTank); MaxValCounter = MaxVMainFrTank;
if(chouse == true ){VMainFrTank = Valcounter; break;} else break;

case 4:LCD_SetPos(1,0);LCD_String("2 Main Fraction Tank");LCD_SetPos(2,0);LCD_String(VMainFrResTank); MaxValCounter = MaxVMainFrResTank;
if(chouse == true ){VMainFrResTank = Valcounter; break;} else break;

case 5:LCD_SetPos(1,0);LCD_String("Heavy Commercial Tank");LCD_SetPos(2,0);LCD_String(VHeavyComTank); MaxValCounter = MaxVHeavyComTank;
if(chouse == true ){VHeavyComTank = Valcounter; break;} else break;

case 6:LCD_SetPos(1,0);LCD_String("Heavy Ends Tank");LCD_SetPos(2,0);LCD_String(VHeavyTank); MaxValCounter = MaxVHeavyTank;
if(chouse == true ){VHeavyTank = Valcounter; break;} else break;

}

if (EncoderVal == ButtonMenu)break;

case 6 :
LCD_SetPos(1,0);LCD_String("Time Settings");
MaxCounter = 6;
if (EncoderVal == ButtonMenu){SubMenu = 1;break;}

case 7 :
LCD_SetPos(1,0);LCD_String("Save Settinds");
MaxCounter = 4;
switch(counter)
{
case 0:LCD_SetPos(1,0);LCD_String("Save cell 1");
if(chouse == true ){SaveSettinds(0); LCD_SetPos(1,0);LCD_String("Saving"); chouse !=chouse; break;} else break;

case 1:LCD_SetPos(1,0);LCD_String("Save cell 2");
if(chouse == true ){SaveSettinds(1); LCD_SetPos(1,0);LCD_String("Saving"); chouse !=chouse; break;} else break;

case 2:LCD_SetPos(1,0);LCD_String("Save cell 3");
if(chouse == true ){SaveSettinds(2); LCD_SetPos(1,0);LCD_String("Saving"); chouse !=chouse; break;} else break;

case 3:LCD_SetPos(1,0);LCD_String("Save cell 4");
if(chouse == true ){SaveSettinds(3); LCD_SetPos(1,0);LCD_String("Saving"); chouse !=chouse; break;} else break;

case 4:LCD_SetPos(1,0);LCD_String("Save cell 5");
if(chouse == true ){SaveSettinds(4); LCD_SetPos(1,0);LCD_String("Saving"); chouse !=chouse; break;} else break;

if (EncoderVal == ButtonMenu){SubMenu = 2;break;}


case 8 :
LCD_SetPos(1,0);LCD_String("Load Settinds");
MaxCounter = 4;
switch(counter)
{
case 0:LCD_SetPos(1,0);LCD_String("Load cell 1");
if(chouse == true ){LoadSettings(0); LCD_SetPos(1,0);LCD_String("Loading"); chouse !=chouse; break;} else break;

case 1:LCD_SetPos(1,0);LCD_String("Load cell 2");
if(chouse == true ){LoadSettings(1); LCD_SetPos(1,0);LCD_String("Loading"); chouse !=chouse; break;} else break;

case 2:LCD_SetPos(1,0);LCD_String("Load cell 3");
if(chouse == true ){LoadSettings(2); LCD_SetPos(1,0);LCD_String("Loading"); chouse !=chouse; break;} else break;

case 3:LCD_SetPos(1,0);LCD_String("Load cell 4");
if(chouse == true ){LoadSettings(3); LCD_SetPos(1,0);LCD_String("Loading"); chouse !=chouse; break;} else break;

case 4:LCD_SetPos(1,0);LCD_String("Load cell 5");
if(chouse == true ){LoadSettings(4); LCD_SetPos(1,0);LCD_String("Loading"); chouse !=chouse; break;} else break;

if (EncoderVal == ButtonMenu){SubMenu = 3;break;}
}
if (EncoderVal == ButtonMenu)break;
}
}
//-------------------------------------------------------------------------------------------------
void DS18b20() {
    uint8_t status;
    uint8_t dt[8];
    uint16_t raw_temper;
    float temper[8];
    char c;
    uint8_t i;
  for(i=1;i<=Dev_Cnt;i++)
      {
        ds18b20_MeasureTemperCmd(NO_SKIP_ROM, i);
      }
      HAL_Delay(800);
      for(i=1;i<=Dev_Cnt;i++)
      {
        ds18b20_ReadStratcpad(NO_SKIP_ROM, dt, i);
        /*sprintf(str1,"STRATHPAD %d: %02X %02X %02X %02X %02X %02X %02X %02X; ",
          i, dt[0], dt[1], dt[2], dt[3], dt[4], dt[5], dt[6], dt[7]);*/
        // HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
        raw_temper = ((uint16_t)dt[1]<<8)|dt[0];
        if(ds18b20_GetSign(raw_temper)) c='-';
        else c='+';
        temper[i] = ds18b20_Convert(raw_temper);
        //sprintf(str1,"Raw t: 0x%04X; t: %c%.2f\r\n", raw_temper, c, temper);
        //HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
      }
      HAL_Delay(150);
        TempTanker       = temper[SensorT[0]];
        TempDrawer       = temper[SensorT[1]];
        TempTakeOff      = temper[SensorT[2]];
        TempReflux       = temper[SensorT[3]];
        TempRefrigerator = temper[SensorT[4]];
        TempAmbient      = temper[SensorT[5]];
}

//-------------------------------------------------------------------------------------------------
float BME280(int a) {
    float tf = 0.0f, pf = 0.0f, af = 0.0f, hf = 0.0f;

    switch (a){
    case 1:
              tf = BME280_ReadTemperature();
            //sprintf(str1, "Temperature: %.3f *C\r\n", tf);
            //HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
            //LCD_SetPos(0, 0);
            //sprintf(str1, "%11.3f *C", tf);
            //LCD_String(str1);
              HAL_Delay(1000);
            return tf
    case 2:
              pf = BME280_ReadPressure();
            //sprintf(str1, "Pressure: %.3f Pa; %.3f hPa; %.3f mmHg\r\n", pf, pf/1000.0f, pf * 0.000750061683f);
            //HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
            //LCD_SetPos(0, 1);
            //sprintf(str1, "%11.3f hPa", pf/1000.0f);
            //LCD_String(str1);
            //LCD_SetPos(0, 2);
            //sprintf(str1, "%11.3f mmHg", pf * 0.000750061683f);
            //LCD_String(str1);
              pf /=1000.000f;
              HAL_Delay(1000);
            return pf

}
    /*  af = BME280_ReadAltitude(SEALEVELPRESSURE_PA);
        sprintf(str1, "Altitude: %.3f m\r\n", af);
        HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
        hf = BME280_ReadHumidity();
        sprintf(str1, "Humidity: %.3f %%\r\n", hf);
        HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
        LCD_SetPos(0, 3);
        sprintf(str1, "%7.3f %% %4.1f m", hf, af);
        LCD_String(str1);
        HAL_Delay(1000);
      */
}

//-------------------------------------------------------------------------------------------------
float GetVarporization() {
float Varp = 0;
return Varp
}

//-------------------------------------------------------------------------------------------------
float GetSelection() {
float Selection = 0;
return Selection
}

//-------------------------------------------------------------------------------------------------
void ControlSelection(int i) {
       // rad=1.97/45"/12.5 //101.325
  double degree = 0;
  double Density = GetBME280(2)*(MolarMain*RawMaterialPersent+MolarSolvent*PersentSolvent)/(8.31446261815324*(GetBME280(1)+273.15));
  double k = 472800*50797600000000000/2144923493918082089;

 if (GetSelection() - Selection[i]>0)
 {degree = (2*Selection[i] - GetSelection())/(1.97*PowerEHE*3600/(Density*Vaporization))*k*1000;}
 if (GetSelection() - Selection[i]<0)
    {degree = Selection[i]/(1.97*PowerEHE*3600/(Density*Vaporization));}

    HAL_TIM_Base_Start(&htim2);
  // Start PWM at Port-B pin#6
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, map(degree,0,1000,50,62.5));
  /*if(HAL_GPIO_ReadPin(GPIOA,G180_Pin)==GPIO_PIN_RESET)
    //2ms Pwm - Servo motor arm rotates to 180 degree
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 100);

    if(HAL_GPIO_ReadPin(GPIOA,G90_Pin)==GPIO_PIN_RESET)
    //1.5ms Pwm - Servo motor arm rotates to 90 degree
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 75);

    if(HAL_GPIO_ReadPin(GPIOA,G0_Pin)==GPIO_PIN_RESET)
    //1ms Pwm - Servo motor arm rotates to 0 degree
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 50);
  */

}

//-------------------------------------------------------------------------------------------------
void ControlEHE(int power) {

  double Density = GetBME280(2)*(MolarMain*RawMaterialPersent+MolarSolvent*PersentSolvent)/(8.31446261815324*GetBME280(1));
  double TVaporization = power*3600/(Density*Vaporization)*Stock;
  int FlagEHE = 0;
  if((GetVarporization() - TVaporization && FlagEHE == 0)>0.00) power -= 0.001;
  if(abs((GetVarporization() - TVaporization && FlagEHE == 0)/GetVarporization())<=0.01) FlagEHE = 1 ;
  if((GetVarporization() - TVaporization)<0 && FlagEHE == 0) power += 0.001;
  power = map(power,0,MaxPowerEHE,0,1024);
  

}

//-------------------------------------------------------------------------------------------------

int ExtraInterapt() {

}

//-------------------------------------------------------------------------------------------------


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
