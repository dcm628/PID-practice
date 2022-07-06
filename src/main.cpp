#include <Arduino.h>
#include <math.h>
#include <iostream>

#include "ValveStates.h"

ValveState bangValveState = ValveState::Closed;
float loopyboi = 295;
float controllerTargetValue = 300;
IntervalTimer pressureUpdateInterval;
IntervalTimer banbBangcontrollerInterval;

int unitConversionCosnt = 6895;
float TankVolume = 0.1; //m^3
float OptimalTankPress = 300* unitConversionCosnt;
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
float DeadBand = unitConversionCosnt* 10;
float CurrPressure = 0;
float MinimumValveOpenTime = 0.1;
float CurrValveOpenTime = 0;
elapsedMillis valveTimer;
float gasDensity = 0;
float TankPropMass = 0;
float TankPressure = 0;
float massFlow = 0;


/* float TankPressArray[];
float PossibleTankPressArray[];
float optimalArray[];
float derivativeArray[];
float ErrorArray[];
float IngegralArray[]; */

float prevPressure = CurrPressure;
bool hasbeenopened = false;
bool ValveStillForceOpen = false;
const int size = 8;
const int size2 = 30;
float mostRecentIndex = 5;
float IngegralArray[size2] = {};
float pastnArr[size+1] = {};      // creates arrays of given size plus 1 to leave the first value of the array it's size
float pastNDpArr[size2+1] = {};   // creates arrays of given size plus 1 to leave the first value of the array it's size
float Kp = 2;
float Ki = 15;
float Kd = .1;
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
  sizeInputArrayLinReg = static_cast<int>(inputArrayLinReg[0]);
  arrayMostRecentPositionLinReg = static_cast<int>(inputArrayLinReg[1]);
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

float reimannSum_PID(float inputArray[], float timeStep, float sumZeroPoint, int summationTermNum)
{
  // !!!!! - Function is built to expect arrays in format of:
  // !!!!! - index[0] = size of entries not counting this and the following, 
  // !!!!! - index[1] = array index for the starting point which is most recent entry, with next most recent the next highest index and so on
  // using centered version of reimann sum
  
  sizeInputArrayRSum = static_cast<int>(inputArray[0]);
  arrayMostRecentPositionRSum = static_cast<int>(inputArray[1]);
  sum_PIDOutput = 0;
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


float measurementArray [100]= {}; // up to 100 measured values at a time to use in controller

  float funcOutput = 0;
  float e_p = 0;
  float e_i = 0;
  float e_d = 0;
  int arrayMostRecentPositionPID = 0;

float PIDmath(float inputArrayPID[], float controllerSetPoint, float timeStep, float integrationSteps, float errorThreshold, float K_p, float K_i, float K_d)
{
  funcOutput = 0;
  e_p = 0;
  e_i = 0;
  e_d = 0;
  arrayMostRecentPositionPID = static_cast<int>(inputArrayPID[1]);
  // PID function calculations
  e_p = controllerSetPoint - inputArrayPID[arrayMostRecentPositionPID];    // proportional offset calculation
  e_i = reimannSum_PID(inputArrayPID, timeStep, controllerTargetValue, integrationSteps);    // integral function calculation
  e_d = linearRegressionLeastSquared_PID(inputArrayPID, 8, timeStep);    // derivative function calculation


  // normalizes units to be in PSI
  funcOutput = (K_p*(e_p)) - (K_i*(e_i/integrationSteps)) - (K_d*(e_d * timeStep));
/*   Serial.println("insidePID: ");
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
  Serial.println(funcOutput); */
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
  sizeInputArrayInsert = static_cast<int>(rollingArray[0]);
  arrayMostRecentPositionInsert = static_cast<int>(rollingArray[1]);
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



void bangbangController(float bandPIDoutput, float controllerThreshold)
{
//Serial.println("is this thing on?");
  // Update ValveState from the "Banging" ones to Open/Closed where they are allowed to be energized/deenergized again
  // When converting this to run valves on real RocketDriver code, need to use commanded open/closed states.
  if (bangValveState == ValveState::BangingOpen)
  {
    if (valveTimer >= 150)    // X ms opening/closing time
    {
      bangValveState = ValveState::Open;
    }
  }
  if (bangValveState == ValveState::BangingClosed)
  {
    if (valveTimer >= 100)    // X ms opening/closing time
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
      bangValveState = ValveState::BangingOpen;
      valveTimer = 0;
    }
    
  }
  if (bandPIDoutput < ((-1)*controllerThreshold))
  {
    //close valve
    if (bangValveState == ValveState::Open)
    {
      bangValveState = ValveState::BangingClosed;
      valveTimer = 0;
    }
  }
  //Serial.println(static_cast<uint8_t>(bangValveState));
}

float CurrTankPress(float TankPropMass)
{
  gasDensity = TankPropMass/TankVolume;
  TankPressure = gasDensity*AirGasConstant*ATPtemp;
  return TankPressure;
}

float ChokedMassFlow(float UpstreamPressure, float chockedOrificeArea)
{
  massFlow = Cd*UpstreamPressure*chockedOrificeArea*(Gamma/AirGasConstant/ATPtemp*(2/(Gamma+1))*(Gamma/AirGasConstant/ATPtemp*(2/(Gamma+1))*((Gamma+1)/(Gamma-1)))*((Gamma+1)/(Gamma-1)))*(1/2);
  return massFlow;
}

//for interrupt timers, not sure how to setup to run the other stuff yet. Need to pass everything into this, then into other functions inside?
// it should work fine, anything that needs to be updated to pass into the functions will pass through every time the interrupt runs (I think)
void pressureUpdateFunction()
{
  if (bangValveState == ValveState::Open || bangValveState == ValveState::BangingOpen)
  {
  loopyboi = loopyboi + .2;
  //Serial.println("we are Open");
  }
  if (bangValveState == ValveState::Closed || bangValveState == ValveState::BangingClosed)
  {
  loopyboi = loopyboi - .1;
  //Serial.println("bitch we closed");
  }
  Serial.println(loopyboi);
}

void controllerUpdateFunction()
{
  PIDResult = PIDmath(IngegralArray,controllerTargetValue,TimeDelta, 4, 0.1 , Kp, Ki, Kd);
  bangbangController(PIDResult, 1);
  writeToRollingArray(IngegralArray, loopyboi);

}


void setup() {
  // put your setup code here, to run once:
pastnArr [0] = size;
pastNDpArr[0] = {size2};

pressureUpdateInterval.begin(pressureUpdateFunction, 5000);
pressureUpdateInterval.priority(126);
banbBangcontrollerInterval.begin(controllerUpdateFunction, 10000);
banbBangcontrollerInterval.priority(124);

// testing bullshit
IngegralArray[0] = {size};
IngegralArray[1] = {mostRecentIndex};
/* IngegralArray[2] = {285};
IngegralArray[3] = {290};
IngegralArray[4] = {295};
IngegralArray[5] = {300};
IngegralArray[6] = {305};
IngegralArray[7] = {310};
IngegralArray[8] = {315};
IngegralArray[9] = {320}; */
// tweaked version to have most recent value at index 5
IngegralArray[5] = {250};
IngegralArray[6] = {250};
IngegralArray[7] = {250};
IngegralArray[8] = {250};
IngegralArray[9] = {250};
IngegralArray[2] = {250};
IngegralArray[3] = {250};
IngegralArray[4] = {250};
}

void loop() 
{

// testing bullshit


//PIDResult = PIDmath(IngegralArray,300,TimeDelta,8, 1, 1, .5, .1);
//Serial.print("PID function output: ");
//Serial.println(PIDResult);


// fuck around and find out with array functions
//loopyboi = loopyboi +0.15;
//writeToRollingArray(IngegralArray, loopyboi);
//Serial.println(loopyboi);
/* Serial.print("Array readout: ");
for (size_t i = 2; i < 10; i++)
{
Serial.print(" : ");
Serial.print(IngegralArray[i]);
}
Serial.println(); */

}