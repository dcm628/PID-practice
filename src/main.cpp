#include <Arduino.h>
#include <math.h>
#include <iostream>
#include <ADC.h>

#include "ValveStates.h"

bool ISTHISREAL = false;  //flag for test mode vs driving real system. False means test, True means real
bool ISVALVEONLYREAL = true;
ADC* adc = new ADC();
#define BANG1PT_ANALOGINPUT A21  //Pasafire options: A20,A18,A17,A16,A21(fuel line PT),A22(Lox line PT),A15, and maybe A14 that was supposed to be chamber PT?
uint16_t currentRawValue = 0;
float currentConvertedValue = 0;
float fuelLinePT_linConvCoef1_m = 0.0124;
float fuelLinePT_linConvCoef1_b = -123.17;
float LoxLinePT_linConvCoef1_m = 0.0124;
float LoxLinePT_linConvCoef1_b = -126.80;

int8_t testInt = 0;
ValveState bangValveState = ValveState::Closed;
uint8_t bangValvePin = 5; //Pasafire options: 5,6,7,8,9,10, for pyro channels: 31, 32


float loopyboi = 0;
float controllerTargetValue = 300;
int controllerTargetValueInt = static_cast<int>(controllerTargetValue+0.5);
IntervalTimer pressureUpdateInterval;
IntervalTimer banbBangcontrollerInterval;

const int8_t size3 = 125; //max size for signed 8_t index value, I might need to make my function store these as uints and shift them to keep the indexing system at max
int8_t mostRecentIndex8bitInt = 2;
const int size = 8;
const int size2 = 125;
float mostRecentIndex = 2;
//float IntegralArray[size2+1] = {};
int8_t IntegralArray8bitInt[size3+1] = {};
float derivativeArray[size+1] = {};
int16_t loopyboi2 = 0;
bool loopyboi2Flag = false;

int unitConversionCosnt = 6895;
float TankVolume = 0.1; //m^3
float OptimalTankPress = controllerTargetValue* unitConversionCosnt;
float orificeOutDiameter = 0.1*0.0254; //m
float orificeOutArea = ((orificeOutDiameter*orificeOutDiameter)*M_PI)/4;
float orificeInDiameter = 0.1*0.0254; //m
float orificeInArea = ((orificeInDiameter*orificeInDiameter)*M_PI)/4;
float COPVPress = 1000* unitConversionCosnt; //psi
float AirGasConstant = 287; //J/kg-k
float ATPtemp = 288.15; //K
float Gamma = 1.4;
float Cd = 0.973;
float Time = 0;
float TimeDelta = .01;
float TankMass = 0;
float CurrPressure = 0;
float MinimumValveOpenTime = 0.1;
float CurrValveOpenTime = 0;
elapsedMillis valveTimer;
float gasDensity = 0;
float TankPropMass = 0;
float TankPressure = 0;
float massFlow = 0;


float pastnArr[size+1] = {};      // creates arrays of given size plus 1 to leave the first value of the array it's size
float pastNDpArr[size2+1] = {};   // creates arrays of given size plus 1 to leave the first value of the array it's size
float Kp = 1.5;
float Ki = 0;
float Kd = 0;
float Integral = 0;


  int arrayWrapSizeLinReg = 0;
  int arrayMostRecentPositionLinReg = 0;
  int regression_n = 0;
  //float regression_PIDOutput = 0;
  int sizeInputArrayLinReg = 0;

float linearRegressionLeastSquared_PID(float inputArrayLinReg[], int regressionSamples, float timeStep)
{
  // Version of linear regression simplified for finding the recent slope for a PID controller
  // assumes fixed time steps, time is X, controller variable Y
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = size of entries not counting this and the following, 
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  sizeInputArrayLinReg = static_cast<int>(inputArrayLinReg[0]+0.5);
  arrayMostRecentPositionLinReg = static_cast<int>(inputArrayLinReg[1]+0.5);
    // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayLinReg < regressionSamples)
  {
    regression_n = sizeInputArrayLinReg;
  }
  else regression_n = regressionSamples;
  // determine the overwrap value, if any
  arrayWrapSizeLinReg =  (-1) * ((arrayMostRecentPositionLinReg - regression_n) - 1);

  float sumX = 0;
  float sumY = 0;
  float sumXX = 0;
  float sumXY = 0;
  float denLeastSquare = 0;
  float a0LeastSquare = 0;
  float a1LeastSquare = 0;
  // calculate the sum terms;
  // 
    //Serial.print("First for loop: ");
    //Serial.println(arrayWrapSizeLinReg);
    if (arrayWrapSizeLinReg <= 0)    // when there is no wrap required, calculated value will be zero or negative. Set to zero.
    {
      arrayWrapSizeLinReg = 0;
    }
    //Serial.print("overwrap after zero set: ");
    //Serial.println(arrayWrapSizeLinReg);
  if (arrayWrapSizeLinReg > 0)  //only true if there are enough array values to use to wrap the end of the array
  {
    for (int i = arrayMostRecentPositionLinReg; i > (1); i--)
    {
      sumX = sumX + ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep);
      sumXX = sumXX + (((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep)*((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN1: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
 */    
    }  

    for (int i = (sizeInputArrayLinReg + 1); i > (sizeInputArrayLinReg + 1 - arrayWrapSizeLinReg); i--)
    {
      sumX = sumX + ((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 2)*timeStep);
      sumXX = sumXX + (((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 2)*timeStep)*((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 2)*timeStep));
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 2)*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN2: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i + (arrayWrapSizeLinReg) - (sizeInputArrayLinReg) - 2));
 */    
    }  
  }
  else
  {
    for (int i = arrayMostRecentPositionLinReg; i > (arrayMostRecentPositionLinReg - regression_n); i--)
    {
      sumX = sumX + ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep);
      sumXX = sumXX + (((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep)*((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
      sumY = sumY + inputArrayLinReg[i];
      sumXY = sumXY + (inputArrayLinReg[i] * ((i - (arrayMostRecentPositionLinReg - regression_n + 1))*timeStep));
/*       Serial.print("DOES THIS EVER HAPPEN: ");
      Serial.print(i);
      Serial.print(" : ");
      Serial.println((i - (arrayMostRecentPositionLinReg - regression_n + 1)));
 */    
    }
  }
/*   Serial.print("sumX: ");
  Serial.println(sumX,5);
  Serial.print("sumY: ");
  Serial.println(sumY,5);
  Serial.print("sumXX: ");
  Serial.println(sumXX,5);
  Serial.print("sumXY: ");
  Serial.println(sumXY,5); */

  // calculate the denominator term
  denLeastSquare = regression_n*sumXX - (sumX * sumX);
  //Serial.print("den: ");
  //Serial.println(denLeastSquare,5);
  // calculate the a1 term, which is the slope
  a1LeastSquare = ((regression_n*sumXY) - (sumX*sumY))/denLeastSquare;
  // calculate the a0 term, which is the linear offset
  // NOT USED IN PID VERSION
  // a0LeastSquare = ((sumXX*sumY) - (sumXY*sumX))/denLeastSquare;
  return a1LeastSquare;
}

  int reimann_n = 0;
  float sum_PIDOutput = 0;
  int arrayWrapSizeRSum = 0;
  int arrayMostRecentPositionRSum = 0;
  int sizeInputArrayRSum = 0;
  float sumZeroPoint = 0;

float reimannSum_PID_float(float inputArray[], float timeStep, int summationTermNum, float sumZeroPoint = 0)
{
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = size of entries not counting this and the following, 
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  // using centered version of reimann sum
  sizeInputArrayRSum = static_cast<int>(inputArray[0]+0.5);
  arrayMostRecentPositionRSum = static_cast<int>(inputArray[1]+0.5);
  sum_PIDOutput = 0;
  reimann_n = 0;
  // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayRSum < summationTermNum)
  {
    reimann_n = sizeInputArrayRSum;
  }
  else reimann_n = summationTermNum;
  // determine the overwrap value, if any
  arrayWrapSizeRSum = reimann_n - ((sizeInputArrayRSum + 1) - (arrayMostRecentPositionRSum));
  //Serial.print("arrayWrapSizeRSum: ");
  //Serial.println(arrayWrapSizeRSum);
  if (arrayWrapSizeRSum > 0) //only true if there are enough array values to use to wrap the end of the array
    {
      for (int i = arrayMostRecentPositionRSum; i < (sizeInputArrayRSum + 2); i++)
      {
        sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
        //Serial.print("sum_PIDOutput: ");
        //Serial.println(sum_PIDOutput);

      }
      for (int i = 2; i < (arrayWrapSizeRSum + 1); i++)
      {
        sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
        //Serial.print("sum_PIDOutput: ");
        //Serial.println(sum_PIDOutput);
      }
    }
  else
  {
    for (int i = arrayMostRecentPositionRSum; i < (arrayMostRecentPositionRSum + reimann_n); i++)
    {
      sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
    }
  }

  
  return sum_PIDOutput;
}

  int8_t arrayMostRecentPositionRSum8bitInt = 0;
  int8_t sizeInputArrayRSum8bitInt = 0;

float reimannSum_PID_8bitInt(int8_t inputArray[], float timeStep, int summationTermNum, float sumZeroPoint = 0)
{
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = size of entries not counting this and the following, 
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  // using centered version of reimann sum
  
  sizeInputArrayRSum8bitInt = inputArray[0];
  arrayMostRecentPositionRSum8bitInt = inputArray[1];
  sum_PIDOutput = 0;
  // if statement to handle case where the input Array is smaller than the set number of terms to integrate over
  if (sizeInputArrayRSum8bitInt < summationTermNum)
  {
    reimann_n = sizeInputArrayRSum8bitInt;
  }
  else reimann_n = summationTermNum;
  // determine the overwrap value, if any
  arrayWrapSizeRSum = reimann_n - ((sizeInputArrayRSum8bitInt + 1) - (arrayMostRecentPositionRSum8bitInt));
  //Serial.print("arrayWrapSizeRSum: ");
  //Serial.println(arrayWrapSizeRSum);
  if (arrayWrapSizeRSum > 0) //only true if there are enough array values to use to wrap the end of the array
    {
      for (int i = arrayMostRecentPositionRSum8bitInt; i < (sizeInputArrayRSum8bitInt + 2); i++)
      {
        sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
        //Serial.print("sum_PIDOutput: ");
        //Serial.println(sum_PIDOutput);

      }
      for (int i = 2; i < (arrayWrapSizeRSum + 1); i++)
      {
        sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
        //Serial.print("sum_PIDOutput: ");
        //Serial.println(sum_PIDOutput);
      }
    }
  else
  {
    for (int i = arrayMostRecentPositionRSum8bitInt; i < (arrayMostRecentPositionRSum8bitInt + reimann_n); i++)
    {
      sum_PIDOutput = sum_PIDOutput + ((inputArray[i] - sumZeroPoint) * timeStep);
    }
  }

  
  return sum_PIDOutput;
}

float measurementArray [100]= {}; // up to 100 measured values at a time to use in controller

  float funcOutput = 0;
  float P_p = 0;
  float P_i = 0;
  float P_d = 0;
  float e_p = 0;
  float e_i = 0;
  float e_d = 0;
  int arrayMostRecentPositionPID = 0;
  bool PIDmathPrintFlag = false;

float PIDmath(float inputArrayPID[], int8_t inputArrayIntPID[], float controllerSetPoint, float timeStep, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d)
{
  funcOutput = 0;
  P_p = 0;
  P_i = 0;
  P_d = 0;
  e_p = 0;
  e_i = 0;
  e_d = 0;
  arrayMostRecentPositionPID = static_cast<int>(inputArrayPID[1]+0.5);
/*   // PID function calculations - OG float style
  e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
  e_i = reimannSum_PID_float(inputArrayPID, timeStep, integrationSteps, controllerTargetValue);    // integral function calculation
  e_d = linearRegressionLeastSquared_PID(inputArrayPID, 8, timeStep);    // derivative function calculation
 */  
  // PID function calculations - new integral differenced int style
  e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
  e_i = reimannSum_PID_8bitInt(inputArrayIntPID, timeStep, integrationSteps);    // integral function calculation
  e_d = linearRegressionLeastSquared_PID(inputArrayPID, 5, timeStep);    // derivative function calculation

/*     Serial.print(K_i);
    Serial.print(" from 8bitInt: ");
    Serial.print(e_i);
    Serial.print(" : ");
    Serial.println(K_i*(e_i/integrationSteps));
    e_i = reimannSum_PID_float(inputArrayPID, timeStep, integrationSteps, controllerTargetValue);    // integral function calculation
    Serial.print(K_i);
    Serial.print(" from float: ");
    Serial.print(e_i);
    Serial.print(" : ");
    Serial.println(K_i*(e_i/integrationSteps)); */


  // normalizes units to be in PSI
  funcOutput = (K_p*(e_p)) - (K_i*(e_i/integrationSteps)) - (K_d*(e_d * timeStep)); //still not 100% sure on signs, particularly the d
  if (PIDmathPrintFlag)
  {
    Serial.println("insidePID: ");
    Serial.print(K_p);
    Serial.print(" : ");
    Serial.print(e_p);
    Serial.print(" : ");
    Serial.println(K_p*(e_p));
    Serial.print(K_i);
    Serial.print(" : ");
    Serial.print(e_i);
    Serial.print(" : ");
    Serial.println(K_i*(e_i/integrationSteps));
    Serial.print(K_d);  
    Serial.print(" : ");
    Serial.print(e_d);  
    Serial.print(" : ");
    Serial.println(K_d*(e_d * timeStep));  
    Serial.println(funcOutput);
  }

  //P_p just for prints
  P_p = K_p*(e_p);
  P_i = K_i*(e_i/integrationSteps);
  P_d = K_d*(e_d * timeStep);

  return funcOutput;
}
// testing bullshit
float IntegralResult = 0;
float DerivativeResult = 0;
float PIDResult = 0;

int sizeInputArrayInsert = 0;
int arrayMostRecentPositionInsert = 0;
// utility function for running a rolling array
// float array version - !!!!! Not Protected from if you put negative signed floats for the index values !!!!!
void writeToRollingArray(float rollingArray[], float newInputArrayValue)
{
  sizeInputArrayInsert = static_cast<int>(rollingArray[0]+0.5);
  arrayMostRecentPositionInsert = static_cast<int>(rollingArray[1]+0.5);
/*   Serial.print("writeArray vars: ");
  Serial.print(sizeInputArrayInsert);
  Serial.println(arrayMostRecentPositionInsert); */

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + 1)) // special case for being at the end of the array
  {
    rollingArray[1] = 2;
    rollingArray[2] = newInputArrayValue;
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
}
// int array version - !!!!! Not Protected from if you put negative signed ints for the index values !!!!!
/* void writeToRollingArray(int rollingArray[], int newInputArrayValue)
{
  sizeInputArrayInsert = rollingArray[0];
  arrayMostRecentPositionInsert = rollingArray[1];
  if (arrayMostRecentPositionInsert = (sizeInputArrayInsert + 1)) // special case for being at the end of the array
  {
    rollingArray[1] = 2;
    rollingArray[2] = newInputArrayValue;
  }
  else
  {
    rollingArray[1] = arrayMostRecentPositionInsert + 1;
    rollingArray[arrayMostRecentPositionInsert + 1] = newInputArrayValue;
  }
} */

int32_t differencedIntValue = 0;
int arrayBitDepth = 0;
void writeToDifferencedIntArray(int8_t differencedRollingArray[], int newInputArrayValue, int arrayBitDepth, int differencedZeroPoint = 0)
{
  differencedIntValue = newInputArrayValue - differencedZeroPoint;
  //Serial.print("inside diff write");
  //Serial.println(differencedIntValue);
  if(arrayBitDepth <= 8)
  {
    // overflow capping into signed 8 bit int range
    if (differencedIntValue >= 127)
    {
      differencedIntValue = 127;
    }
    if (differencedIntValue <= -128)
    {
      differencedIntValue = -128;
    }
  }
/*   
  else if(arrayBitDepth > 8 && arrayBitDepth <= 16)
  {
    // overflow capping into signed 16 bit int range
    if (differencedIntValue >= 32,767)
    {
      differencedIntValue = 32,767;
    }
    if (differencedIntValue <= -32,768)
    {
      differencedIntValue = -32,768;
    }
  }
  else if(arrayBitDepth > 16 && arrayBitDepth <= 32)
  {
    // overflow capping into signed 32 bit int range
    if (differencedIntValue >= 2,147,483,647)
    {
      differencedIntValue = 2,147,483,647;
    }
    if (differencedIntValue <= -2,147,483,648)
    {
      differencedIntValue = -2,147,483,648;
    }
  }
  else if(arrayBitDepth > 32 && arrayBitDepth <= 64)
  {
    // overflow capping into signed 64 bit int range
    if (differencedIntValue >= 9,223,372,036,854,775,807)
    {
      differencedIntValue = 9,223,372,036,854,775,807;
    }
    if (differencedIntValue <= -9,223,372,036,854,775,808)
    {
      differencedIntValue = -9,223,372,036,854,775,808;
    }
  }
  
 */  
    sizeInputArrayInsert = (differencedRollingArray[0]);
  arrayMostRecentPositionInsert = (differencedRollingArray[1]);
/*   Serial.print("writeArray vars: ");
  Serial.print(sizeInputArrayInsert);
  Serial.println(arrayMostRecentPositionInsert); */

  if (arrayMostRecentPositionInsert == (sizeInputArrayInsert + 1)) // special case for being at the end of the array
  {
    differencedRollingArray[1] = 2;
    differencedRollingArray[2] = static_cast<int8_t>(differencedIntValue);
  }
  else
  {
    differencedRollingArray[1] = arrayMostRecentPositionInsert + 1;
    differencedRollingArray[arrayMostRecentPositionInsert + 1] = static_cast<int8_t>(differencedIntValue);
  }
}

void bangbangController(float bandPIDoutput, float controllerThreshold)
{
//Serial.println("is this thing on?");
  // Update ValveState from the "Banging" ones to Open/Closed where they are allowed to be energized/deenergized again
  // When converting this to run valves on real RocketDriver code, need to use commanded open/closed states.
  
  //simulating the real prop control that will progress valve through opening states - skipping the process one, for this it doesn't matter
  if (bangValveState == ValveState::BangOpenCommanded)
  {
    bangValveState = ValveState::BangingOpen;
  }
  if (bangValveState == ValveState::BangCloseCommanded)
  {
    bangValveState = ValveState::BangingClosed;
  }
  //minimum bang time lockouts, once they are up the valves go to plain Open/Closed states which unlocks them to be commanded again
  if (bangValveState == ValveState::BangingOpen)
  {
    if (valveTimer >= 75)    // X ms opening/closing time
    {
      bangValveState = ValveState::Open;
    }
  }
  if (bangValveState == ValveState::BangingClosed)
  {
    if (valveTimer >= 50)    // X ms opening/closing time
    {
      bangValveState = ValveState::Closed;
    }
  }
  // Update ValveState if Open/Closed based on PID controller output
  if (bandPIDoutput > (controllerThreshold))
  {
    //open valve
    if (bangValveState == ValveState::Closed)
    {
      bangValveState = ValveState::BangOpenCommanded;
      valveTimer = 0;
    }
    
  }
  if (bandPIDoutput < ((-1)*controllerThreshold))
  {
    //close valve
    if (bangValveState == ValveState::Open)
    {
      bangValveState = ValveState::BangCloseCommanded;
      valveTimer = 0;
    }
  }
  //Serial.println(static_cast<uint8_t>(bangValveState));
}

float CurrTankPress(float TankPropMass)
{
  gasDensity = TankPropMass/TankVolume;
  TankPressure = gasDensity*AirGasConstant*ATPtemp;
/*   Serial.print("Pizza TankPropMass: ");
  Serial.println(TankPropMass);
  Serial.print("Pizza TankVolume: ");
  Serial.println(TankVolume);
  Serial.print("Pizza gasDensity: ");
  Serial.println(gasDensity);
  Serial.print("Pizza TankPressure: ");
  Serial.println(TankPressure); */
  return TankPressure;
}

float ChokedMassFlow(float UpstreamPressure, float chockedOrificeArea)
{
  massFlow = Cd*UpstreamPressure*chockedOrificeArea*sqrt((Gamma/(AirGasConstant*ATPtemp))*pow((2/(Gamma+1)), ((Gamma+1)/(Gamma-1))));
  //Serial.print("Pizza massFlow: ");
  //Serial.println(massFlow);
  return massFlow;
}

//for interrupt timers, not sure how to setup to run the other stuff yet. Need to pass everything into this, then into other functions inside?
// it should work fine, anything that needs to be updated to pass into the functions will pass through every time the interrupt runs (I think)
void pressureUpdateFunction()
{  
  if (bangValveState == ValveState::Open || bangValveState == ValveState::BangingOpen)
  {
  //loopyboi = loopyboi + .25;
  //loopyboi2 = static_cast<int>(loopyboi+0.5);

  TankMass += ChokedMassFlow(COPVPress,orificeInArea)*TimeDelta;
  TankMass -= ChokedMassFlow(CurrPressure*unitConversionCosnt,orificeOutArea)*TimeDelta;
  //Serial.println("we are Open");
  }
  if (bangValveState == ValveState::Closed || bangValveState == ValveState::BangingClosed)
  {
  //loopyboi = loopyboi - .15;
  //loopyboi2 = static_cast<int>(loopyboi+0.5);
  //Serial.println("bitch we closed");
  //TankMass += ChokedMassFlow(COPVPress,orificeInArea)*TimeDelta;
  TankMass -= ChokedMassFlow(CurrPressure*unitConversionCosnt,orificeOutArea)*TimeDelta;
  }
  CurrPressure = CurrTankPress(TankMass)/unitConversionCosnt;
  /*   Serial.print("PizzaPress: ");
  Serial.println(CurrTankPress(TankMass)/unitConversionCosnt); */

  if (!ISTHISREAL)
  {
    loopyboi = CurrPressure;
    loopyboi2 = static_cast<int>(CurrPressure+0.5);
  }
  else
  {
    //analog PT read stuff here to use real pressure value
    currentRawValue = adc->analogRead(BANG1PT_ANALOGINPUT);
    currentConvertedValue = fuelLinePT_linConvCoef1_m*currentRawValue + fuelLinePT_linConvCoef1_b;
    loopyboi = currentConvertedValue;
    loopyboi2 = static_cast<int>(currentConvertedValue+0.5);
  }
  Serial.print(loopyboi);
  Serial.println(", ");   // comma delimeter
}

void serialOutputStream()
{
  // Set this up to create .csv formatted outputs
  // header row: 
  // tank1 PSI direct read, tank1 PSI controller array, tank1 bang SV state, controller1 target, controller 1 Kp, controller 1 Ki, controller 1 Kd, controller 1 ep, controller 1 ei, controller 1 ed, controller 1 Pp, controller 1 Pi, controller 1 Pd, controller 1 function output,
  Serial.print(loopyboi);
  Serial.print(", ");   // comma delimeter
  Serial.print(derivativeArray[static_cast<int>(derivativeArray[1]+0.5)]);
  Serial.print(", ");   // comma delimeter
  Serial.print(static_cast<uint8_t>(bangValveState));
  Serial.print(", ");   // comma delimeter
  Serial.print(controllerTargetValueInt);
  Serial.print(", ");   // comma delimeter
  Serial.print(Kp);
  Serial.print(", ");   // comma delimeter
  Serial.print(Ki);
  Serial.print(", ");   // comma delimeter
  Serial.print(Kd);
  Serial.print(", ");   // comma delimeter
  Serial.print(e_p);
  Serial.print(", ");   // comma delimeter
  Serial.print(e_i);
  Serial.print(", ");   // comma delimeter
  Serial.print(e_d);
  Serial.print(", ");   // comma delimeter
  Serial.print(P_p);
  Serial.print(", ");   // comma delimeter
  Serial.print(P_i);
  Serial.print(", ");   // comma delimeter
  Serial.print(P_d);
  Serial.print(", ");   // comma delimeter
  Serial.print(PIDResult);
  Serial.print(", ");   // comma delimeter


  Serial.println();  //end line change
}


ValveState bangValveStateIn = ValveState::Closed;
uint8_t ValveOutputPin = 0;
void bangValveDrive(ValveState bangValveStateIn, uint8_t ValveOutputPin)
{
// function for simple output drive of the valves
// hold valve enables high, don't fuck with them in the loop, assign in setup

  switch (bangValveStateIn)
  {
  case ValveState::Closed:
    digitalWriteFast(ValveOutputPin, LOW);
    break;
  case ValveState::BangingClosed:
    digitalWriteFast(ValveOutputPin, LOW);
    break;
  case ValveState::Open:
    digitalWriteFast(ValveOutputPin, HIGH);
    break;
  case ValveState::BangingOpen:
    digitalWriteFast(ValveOutputPin, HIGH);
    break;
  default:
    break;
  }
}



void controllerUpdateFunction()
{
  PIDResult = PIDmath(derivativeArray, IntegralArray8bitInt, controllerTargetValue,TimeDelta, 100, 0.1 , Kp, Ki, Kd);
  bangbangController(PIDResult, 1);
  writeToRollingArray(derivativeArray, loopyboi);   //array for use in derivative
  //writeToRollingArray(IngegralArray, loopyboi); //float integral array, not in use
  writeToDifferencedIntArray(IntegralArray8bitInt, loopyboi2, 8, controllerTargetValueInt);

//physical outputs
if (ISTHISREAL || ISVALVEONLYREAL)
{
  bangValveDrive(bangValveState, bangValvePin);
}

serialOutputStream();

/* Serial.print(", Array readout most recent: float: ");
Serial.println(IngegralArray[static_cast<int>(IngegralArray[1])]);
Serial.print(", Array readout most recent: int: ");
Serial.println(IngegralArray8bitInt[IngegralArray8bitInt[1]]); */

}


//////////////////////////////////////////////////////////////////////
void setup() {
//valve output pin
pinMode(bangValvePin, OUTPUT);
//valveEnable Pasafire output pins
pinMode(11, OUTPUT);
pinMode(12, OUTPUT);
pinMode(24, OUTPUT);
pinMode(25, OUTPUT);

///// ADC0 /////
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 or ADC_REFERENCE::REF_EXT.
  //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
  adc->adc0->setAveraging(8);                                    // set number of averages
  adc->adc0->setResolution(16);                                   // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);     // change the sampling speed
  adc->adc0->recalibrate();

///// ADC1 /////
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
  adc->adc1->setAveraging(8);                                    // set number of averages
  adc->adc1->setResolution(16);                                   // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);     // change the sampling speed
  adc->adc1->recalibrate();





pastnArr [0] = size;
pastNDpArr[0] = {size2};

pressureUpdateInterval.begin(pressureUpdateFunction, 5000);
pressureUpdateInterval.priority(126);
banbBangcontrollerInterval.begin(controllerUpdateFunction, 10000);
banbBangcontrollerInterval.priority(124);

// testing bullshit
//IntegralArray[0] = {size2};
//IntegralArray[1] = {mostRecentIndex};
IntegralArray8bitInt[0] = {size3};
IntegralArray8bitInt[1] = {2};
derivativeArray[0] = {size};
derivativeArray[1] = {2};

}

void loop() 
{

// testing bullshit
PIDmathPrintFlag = false;   //to toggle the PID prints

//PIDResult = PIDmath(IntegralArray,300,TimeDelta,8, 1, 1, .5, .1);
//Serial.print("PID function output: ");
//Serial.println(PIDResult);


/* // fuck around and find out with array functions
if (loopyboi2Flag)
{
  loopyboi2 = loopyboi2 +1;
  if (loopyboi2 >= 150)
  {
    loopyboi2Flag = false;
  }
}
if (!loopyboi2Flag)
{
  loopyboi2 = loopyboi2 -1;
  if (loopyboi2 <= -150)
  {
    loopyboi2Flag = true;
  }
} */

//writeToDifferencedIntArray(IntegralArray8bitInt, loopyboi2, 8);

/* Serial.print("Array entry: ");
Serial.print(loopyboi2);
Serial.print(", Array readout most recent: ");
Serial.println(IntegralArray8bitInt[IntegralArray8bitInt[1]]); */


/* delay(100);
Serial.print(", Array position most recent: ");
Serial.println(derivativeArray[1]);
Serial.print(", most recent value from: ");
Serial.println(derivativeArray[static_cast<int>(derivativeArray[1])]);
Serial.println("Array readout loop: ");
for (size_t i = 2; i < size + 1; i++)
{
//Serial.print(" Int: ");
//Serial.print(IntegralArray8bitInt[i]);
Serial.print(" derivative: ");
Serial.println(derivativeArray[i]);
}
Serial.println(); */


/* testInt = 300;
Serial.print("testInt 8 bit overflow: ");
Serial.println(PIDResult); */
}