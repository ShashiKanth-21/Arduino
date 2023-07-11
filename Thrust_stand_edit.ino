
/**************************************************************************/
/*!
    @file     Thrust_stand_code.ino
    @author   Nikesh Elango (Okulo Aerospace)

    This is a code for implementing logging and telemetry of sensor data over rf
    ----> https://www.okuloaerospace.com

    @section Sensors
    Voltage & current sensor (WCS1700 w/ voltage divider network)
    ADS115-I2C based Analog to Digital Converter.

    @section clock
    DS3231-I2C-RTC

    @section Sensor_connections
    The circuit for SD card logging:
    analog sensors on analog ins 0, 1, and 2
    SD card attached to SPI bus as follows:
    *MOSI -> pin 11
    *MISO -> pin 12
    *CLK  -> pin 13
    *CS   -> pin 4

    DS3231:  SDA pin   -> Arduino Analog 4 or the dedicated SDA pin - PB7
             SCL pin   -> Arduino Analog 5 or the dedicated SCL pin - PB6

    PWM out -> pin digital 9

    @section References:
    https://github.com/pyserial/pyserial
    https://github.com/adafruit/Adafruit_Python_ADS1x15
    https://github.com/adafruit/RTClib
    https://github.com/bogde/HX711

   @section  HISTORY
    v1.0  - First release
    v2.0 - Motor & solar panel configuration changed dedicated function file ** modified 12/5/2021
    v3.0 - ported to stm32 for multiple GPIO pins
*/
// TBD test 3 sck pins with a single pin - for 3 load cell - done
// add ad1115 interface - done
// calibration and zero correction  -TBD
// add IR Sensor -done

// comment the below line for serial data view
//#define PLOT_CSV
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <HX711.h>

#define K1 210407.064398019f
#define L1 827875.8620f//2255.135105531f
#define L2 2274.554907058f
#define L3 2173.94183138f   

#define t_len 74.249

// pin definitions
#define pin_IR PB4
#define pin_THR PB8

#define LOADCELL_T1_DOUT_PIN PB12
#define LOADCELL_T2_DOUT_PIN PB5 //pins PB13 PB14 not working
#define LOADCELL_T3_DOUT_PIN PA15
#define LOADCELL_T1_SCK_PIN PB15
#define LOADCELL_T2_SCK_PIN PB9
#define LOADCELL_T3_SCK_PIN PB3


#define LOADCELL_Q1_DOUT_PIN PB1 
//#define LOADCELL_Q2_DOUT_PIN PA6
//#define LOADCELL_Q3_DOUT_PIN PB1 //changed from PB2 to PA7
#define LOADCELL_Q1_SCK_PIN PB2
//#define LOADCELL_Q2_SCK_PIN PA7
//#define LOADCELL_Q3_SCK_PIN PB2


// I2C definitions
#ifndef PIN_WIRE_SDA
#define PIN_WIRE_SDA PB7
#endif
#ifndef PIN_WIRE_SCL
#define PIN_WIRE_SCL PB6
#endif
/*=========================================================================
    Sensors Sensitivity
 -----------------------------------------------------------------------*/
#define isense  31.595//22.1500001//33.095->6s      // 33mv/A To be calibrated //for 50v 20mv/a 20-> calibration factor  1.1 scaled to 1.08// 
#define vsense  167.00239//91.82736455//167.002349-6s  // 9.95 k +49.63k = 59.58 (r1/r1+r2)*1000 (for 50v : 10/108.9 *  )
#define isense2 30.095     // TBD
#define vsense2 167.002349 // TBD
#define isense3 30.000     // TBD
/*=========================================================================*/

/*===================================================================002349======
    Initializing  a ads115 object.
    -----------------------------------------------------------------------*/
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
/*=========================================================================*/

/*=========================================================================
    HX711 circuit wiring
    -----------------------------------------------------------------------*/
HX711 scale_T1, scale_T2, scale_T3, scale_Q1, scale_Q2, scale_Q3;
/*=========================================================================*

/*=========================================================================
    Sensor Calibration CONSTANTS
    -----------------------------------------------------------------------*/
float cerr, verr, curr, volt;
/*=========================================================================*/

/*=========================================================================
    Miscellaneous global variables
    -----------------------------------------------------------------------*/
float thrust, torque;
uint32_t rpm;
uint32_t channel, channel_rpm;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0, rolloverCompareCount2 = 0;
volatile uint32_t TimeperiodMeasured, LastCapture = 0, CurrentCapture;
HardwareTimer *MyTim1, *MyTim2;
volatile int32_t pwmVal = 0, pwmVal_start = 0;
//volatile int32_t revVal = 1, revVal_start = 0;

// volatile uint32_t pwmLength = 0;

/*=========================================================================*/

void setup()
{



  // Initialize serial port set baud rate to 115200
  Serial.begin(57600);// holybro telem 57600

  //initialize_loadcells();

  TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_THR), PinMap_PWM);
  TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin_IR), PinMap_PWM);
  channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_THR), PinMap_PWM));
  channel_rpm = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_IR), PinMap_PWM));

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim1 = new HardwareTimer(Instance1);
  MyTim2 = new HardwareTimer(Instance2);

  MyTim1->setMode(channel, TIMER_INPUT_CAPTURE_BOTHEDGE, pin_THR);   // modified
  MyTim2->setMode(channel_rpm, TIMER_INPUT_CAPTURE_FALLING, pin_IR); // modified

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of the interruption and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruption processing is around 4,5 microseconds and thus Max frequency is around 220kHz
  uint32_t PrescalerFactor = 84; // clock frequency of blackpill
  MyTim1->setPrescaleFactor(PrescalerFactor);
  MyTim1->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim1->attachInterrupt(channel, InputCapture_channel_IT_callback);
  MyTim1->attachInterrupt(Rollover_IT_callback);
  MyTim1->resume();

  MyTim2->setPrescaleFactor(PrescalerFactor * 500); // 84Mhz clock -> 10Khz (84 Mhz / 8400)
  MyTim2->setOverflow(0x10000);                     // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim2->attachInterrupt(channel_rpm, InputCapture_channel_rpm_IT_callback);
  MyTim2->attachInterrupt(Rollover_IT_callback2);
  MyTim2->resume();

  // Compute this scale factor only once
  input_freq = MyTim2->getTimerClkFreq() / MyTim2->getPrescaleFactor();

  // Initialze load cells
  initialize_loadcells();
  // Initialize & calibrate the Current/voltage sensors
  ads.begin();
  sens_calibrate();
  // ads_read();

  Serial.println("Initialization done.");
  delay(5000);
  //Serial.println((String) "Frequency = " + input_freq);
}

void loop()
{
   ads_read();
   thrust_read();
   torque_read();
   serial_print();
  // Serial.println();
  // Serial.println((String)"Val  = " +(0b1 & (GPIOB->IDR >> 8)));
  // Serial.print((String) " " + revVal_start + ",");

  // Serial.println(pwmVal);

  //Serial.println(TimeperiodMeasured);
  //Serial.println((String) rpm + "," + pwmVal);
  // Serial.println((String)"Pulse Width Channel  = " + pwmVal);

  delay(200);
}
void initialize_loadcells()
{
  
  // Initialize and calibrate the thrust measurement
  scale_T1.begin(LOADCELL_T1_DOUT_PIN, LOADCELL_T1_SCK_PIN);
  scale_T1.set_scale();
  scale_T1.tare();
  Serial.println( scale_T1.get_units(10));
  scale_T1.set_scale(K1); // Initial calibration of load cell.
  scale_T1.tare();
  delay(100);
  
  scale_T2.begin(LOADCELL_T2_DOUT_PIN, LOADCELL_T2_SCK_PIN);
  scale_T2.set_scale();
  scale_T2.tare();
  Serial.println( scale_T2.get_units(10));
  scale_T2.set_scale(K1); // Initial calibration of load cell.
  scale_T2.tare();
  delay(100);
  
  scale_T3.begin(LOADCELL_T3_DOUT_PIN, LOADCELL_T3_SCK_PIN);
  scale_T3.set_scale();
  scale_T3.tare();
  Serial.println( scale_T3.get_units(10));
  scale_T3.set_scale(K1); // Initial calibration of load cell.
  scale_T3.tare();
  delay(100);
  
  Serial.println("Thrust ready");
  
  // torque
  scale_Q1.begin(LOADCELL_Q1_DOUT_PIN, LOADCELL_Q1_SCK_PIN);
  scale_Q1.set_scale();
  scale_Q1.tare();
  Serial.println(scale_Q1.get_units(10));
  scale_Q1.set_scale(L1); // Initial calibration of load cell.
  scale_Q1.tare();
  delay(100);
  
  Serial.println("Torque ready");
  
  //Initialize and calibrate the thrust measurement
  //scale_Q2.begin(LOADCELL_Q2_DOUT_PIN, LOADCELL_Q2_SCK_PIN);
  //scale_Q2.set_scale();
  //scale_Q2.tare();
  //Serial.println(scale_Q2.get_units(10));

  // Initialize and calibrate the thrust measurement
  //scale_Q3.begin(LOADCELL_Q3_DOUT_PIN, LOADCELL_Q3_SCK_PIN);
  //scale_Q3.set_scale();
  //scale_Q3.tare();
  //Serial.println(scale_Q3.get_units(10));

  //Serial.println("place weight now - torque");
  //delay(5000);
  //Serial.println(scale_Q1.get_units(10));
  
  //delay(1000);
  //Serial.println(scale_Q2.get_units(10));
  //scale_Q2.set_scale(L2); // Initial calibration of load cell.
  //scale_Q2.tare();
  //delay(1000);
  //Serial.println(scale_Q3.get_units(10));
  //scale_Q3.set_scale(L3); // Initial calibration of load cell.
  //scale_Q3.tare();
  //delay(1000);
  
}
void sens_calibrate()
{
#ifndef PLOT_CSV
  Serial.println("Setting initial calibration");
#endif

  cerr = verr = 0;
  for (int i = 0; i < 10; i++)
  {
    verr += ads.readADC_SingleEnded(0); // single ended mode
    // verr2 += ads.readADC_SingleEnded(1);          //single ended mode
    cerr += ads.readADC_SingleEnded(1); // single ended mode
    // cerr2 += ads.readADC_SingleEnded(3);          //single ended mode
  }
  cerr = cerr / 10;
  verr = verr / 10;
  delay(2000);

#ifndef PLOT_CSV
  Serial.println("Sensor calibrated");
#endif
}
void ads_read()
{
  float multiplier = 0.1875F; // multiplier based on gain setting 2/3x
  float temp;
  volt = curr = 0;
  for (int i = 0; i < 3; i++)
  {
    volt += (ads.readADC_SingleEnded(0)) / vsense * multiplier;
    // volt2 += (ads.readADC_SingleEnded(1))/vsense2*multiplier;
    temp = ads.readADC_SingleEnded(1);
    if(temp<=15000)
    {
      curr += (temp - cerr) *  multiplier / isense;
      
    }
    else if(15000 < temp  && temp < 16000)
    {
      curr += (temp - cerr) *  multiplier / isense2;
    }
    else
    {
      curr += (temp - cerr) *  multiplier / isense3;
    }
    // curr2 += (ads.readADC_SingleEnded(3)-cerr2)/isense2*multiplier;
    // curr3 += (ads2.readADC_SingleEnded(2)-cerr3)/isense3*multiplier;
  }
  volt /= 3;
  curr /= 3;
}
void serial_print()
{
  // Serial.print(rtc.getDateStr()); Serial.print(",");
  // Serial.print(rtc.getTimeStr()); Serial.print(",");
  // Serial.print(pwm); Serial.print(",");
  Serial.print(volt);
  Serial.print(",");
  Serial.print(curr);
  Serial.print(",");
  Serial.print(thrust, 3);
  Serial.print(",");
  Serial.print(torque, 3);
  Serial.print(",");
  Serial.print(rpm);
  Serial.print(",");
  Serial.print(ads.readADC_SingleEnded(1));
  Serial.print(",");
  Serial.print(pwmVal);
  Serial.println();
}
void thrust_read()
{
  // Serial.print(scale_T1.get_units(3));Serial.print(",");
  // Serial.print(scale_T2.get_units(3));Serial.print(",");
  // Serial.print(scale_T3.get_units(3));Serial.print(",");
  // Serial.println();
  thrust = scale_T1.get_units(2) + scale_T2.get_units(2) + scale_T3.get_units(2);
}
void torque_read()
{

  //Serial.print(scale_Q1.get_units(3));
  //Serial.print(",");
  //Serial.print(scale_Q2.get_units(3));
  //Serial.print(",");
  //Serial.print(scale_Q3.get_units(3));
  //Serial.print(",");
  //Serial.println();
  
  //Serial.print(scale_Q1.get_units(2));
  torque = (scale_Q1.get_units(2));// *3)*0.07*9.81f/1000;
  //Serial.println(torque);
  
 //Serial.println(scale_Q1.get_units(10)); 
}
void InputCapture_channel_IT_callback(void)
{

  if (0b1 & GPIOB->IDR >> 8)
  {
    pwmVal_start = MyTim1->getCaptureCompare(channel);
  }

  else
  {
    pwmVal = MyTim1->getCaptureCompare(channel) - pwmVal_start;
    // MyTim1->setCapturePolarity(channel_3,TIM_INPUTCHANNELPOLARITY_RISING);
    if (pwmVal < 0)
    {
      /* 0x1000 is max overflow value */
      pwmVal += 0x10000;
    }
    rolloverCompareCount = 0;
    // val=1;
  }
}
void Rollover_IT_callback(void)
{
  rolloverCompareCount++;

  if (rolloverCompareCount > 1)
  {
    pwmVal = 0;
  }
}
void InputCapture_channel_rpm_IT_callback(void)
{

  CurrentCapture = MyTim2->getCaptureCompare(channel_rpm);
  /* frequency computation */
  if (CurrentCapture > LastCapture)
  {
    TimeperiodMeasured = (CurrentCapture - LastCapture) * 1000000 / input_freq;
  }
  else if (CurrentCapture <= LastCapture)
  {
    /* 0x10000 is max overflow value */
    TimeperiodMeasured = (0x10000 + CurrentCapture - LastCapture) * 1000000/ input_freq;
  }
  rpm = 60*1000000/TimeperiodMeasured;
  LastCapture = CurrentCapture;
  rolloverCompareCount2 = 0;
  //Serial.println("triggered");
}
void Rollover_IT_callback2(void)
{
  rolloverCompareCount2++;

  if (rolloverCompareCount2 > 1)
  {
    TimeperiodMeasured = 1;
  }
}
