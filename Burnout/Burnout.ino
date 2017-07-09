
#include <TimerOne.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>

// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   4
#define MAXCS   2
#define MAXCLK  3

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

int ledA = 0;
int ledB = 4;
bool showLedA = true;

#define LED0 13
#define LED1 10
#define LED2 11
#define LED3 12
#define LED4 7
#define LED5 9
#define LED6 8

void ledShow()
{
  showLedA = !showLedA;
  int digit;
  if(showLedA)
  {
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
    digit = ledA;
    
  } else {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
    digit = ledB;
  }

  switch(digit)
  {
    case 0:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;
    case 1:
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, HIGH);

      break;
    case 2:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, LOW);

      break;
    case 3:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;
    case 4:
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, HIGH);

      break;
    case 5:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;
    case 6:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;
    case 7:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, HIGH);

      break;
    case 8:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;
    case 9:
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, LOW);

      break;

    case 10://F
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, HIGH);

      break;

    case 11: //H
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, HIGH);

      break;
      
    case 12: // -
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, HIGH);

      break;
    case 13: //n
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, LOW);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, LOW);
      digitalWrite(LED6, HIGH);

      break;
    case 14: //C
      digitalWrite(LED0, LOW);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, LOW);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, LOW);

      break;
      
    default:
      digitalWrite(LED0, HIGH);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED4, HIGH);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, HIGH);
      
        
  } 
  
}

double heatValue = 0.0;
int period = 3000;
void updateHeater()
{
  int cycle = millis() % period;
  if (cycle < (heatValue * period))
    digitalWrite(A5, HIGH);
  else
    digitalWrite(A5, LOW);
  
}

void timerUpdate()
{
  ledShow();
  updateHeater();

  
}


void ledSetup()
{
  for (int i = 5; i <= 13; i++)
    pinMode(i, OUTPUT);

    
}

double PIDSet, PIDIn, PIDOut;

//Specify the links and initial tuning parameters
#define KP 0.01
#define KI 0.000001
#define KD 0




PID heatPID(&PIDIn, &PIDOut, &PIDSet,KP,KI,KD, DIRECT);

void setup() {
  heatPID.SetOutputLimits(0.0, 1.0);
  
  ledSetup();

  //Button Inputs
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  //SSR Output
  pinMode(A5, OUTPUT);
  

  Timer1.initialize(5000);
  Timer1.attachInterrupt( timerUpdate ); // attach the service routine here

  Serial.begin(9200);
  
}



bool buttonAEdge = false;
bool buttonAEdgeMem = false;
unsigned long buttonAStart = 0;
int buttonACount = 0;

bool buttonBEdge = false;
bool buttonBEdgeMem = false;
unsigned long buttonBStart = 0;
int buttonBCount = 0;

int state = 0;
int mode = 1;
int substate = 0;

int knob = 0;
double setTemp = 0;
double startTemp = 0;
double actTemp = 0;

#define CANCELTIME 5
#define STARTTIME 3
#define MAXTEMP 2000

#define TEMPREACHEDDELAY 300000
#define COOLTEMP 350
unsigned long delayMillis;
unsigned long tempReachedMillis;
unsigned long modeStartMillis;

bool tempReached;

#define NUMMODES 6

void loop() {

  

  //Button Detection
  bool buttonA = digitalRead(A1);
  bool buttonB = digitalRead(A2);

  //Rising Edge Detection
  buttonAEdge = false;
  buttonBEdge = false;
  
  if(buttonA && !buttonAEdgeMem)
  {
      buttonAEdge = true;
      buttonAEdgeMem = true;
  }

  if(buttonB && !buttonBEdgeMem)
  {
      buttonBEdge = true;
      buttonBEdgeMem = true;
  }
  
  if(!buttonA)
    buttonAEdgeMem = false;
    
  if(!buttonB)
    buttonBEdgeMem = false;

  //Button Hold Count
  if(buttonAEdge)
    buttonAStart = millis();
  if(buttonBEdge)
    buttonBStart = millis();
    
  if(buttonA)
    buttonACount = (int) ((millis() - buttonAStart) / 1000L);
  else
  {
    buttonACount = 0;
    buttonAStart = 0;
  }

  if(buttonB)
    buttonBCount = (int) ((millis() - buttonBStart) / 1000L);
  else
  { 
    buttonBCount = 0;
    buttonBStart = 0;
  }


  
  
  switch(state)
  {
    case 0: //Waiting for Input / Start Signal
      ledA = 13; //n
      //Check Button B Press & Select Mode
      if(buttonBEdge)
        mode = mode + 1;

      if (mode > NUMMODES)
        mode = 1; 
        
      ledB = mode;
      

     //Check Button A hold to start mode
      
      
      if(buttonA)
      {
  
        if(buttonACount >= STARTTIME)
        {
          state = 1;
          ledA = 12; //-
          ledB = 12; //-
        }
        else
        {
          ledA = -1;
          ledB = STARTTIME - buttonACount;
        }
      }
      
      

      break;
    
    case 1: //Release Start Button
      //ModeStartTime
      modeStartMillis = millis();

      //Reset startTemp
      startTemp = thermocouple.readFarenheit();

      //Reset Remp Reached Indicator
      tempReached = false;
      tempReachedMillis = 0; 
      //Turn on PID
      heatPID.SetMode(AUTOMATIC);
      //Reset PID Integral
      heatPID.SetOutputLimits(0.0, 1.0);   // Forces minimum up to 0.0
      heatPID.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
      heatPID.SetOutputLimits(0.0, 1.0);   // Set the limits back to normal    


      ledA = 12; //-
      ledB = 12; //-

      if(buttonACount == 0)
        state = 2;
      break;
      
    case 2:
      switch(mode)
      {
        case 1: //Mode 1. Full Manual PWM Control
            //get Knob value:
            knob = analogRead(A0);

            //Set PWM
            if (knob > 100)
              heatValue = (knob - 100.0f) / (1023.0f - 100.0f);
            else
              heatValue = 0.0f;
            

            //On Button A Press show Mode
            if(buttonA)
            {
              ledA = 13; //n
              ledB = mode;
              
            } 
            else  //Show PWM Value
            {
              if (heatValue >= 1.0f)
              {
                ledA = 11; //H
                ledB = 1;
              }
              else if (heatValue == 0.0f)
              {
                ledA = 0;
                ledB = 10; //F
              }
              else 
              {
                ledA = (int) (heatValue * 99) / 10;
                ledB = (int) (heatValue * 99) % 10;
              }
            }

            //Check for Cancelation

            if(buttonB)
            {
              if(buttonBCount >= CANCELTIME)
              {
                heatValue = 0.0f;
                state = 0;
              }
              else
              {
                ledA = -1;
                ledB = CANCELTIME - buttonBCount;
              }
              
            }

             if((millis() - delayMillis) > 5000)
            {
              delayMillis = millis();
              
              //Debug output
              Serial.print("Actual Temp: ");
              Serial.print(thermocouple.readFarenheit());
              Serial.print("; Heat Val: ");
              Serial.println(heatValue);
            }

            break;
        case 2: //Temperature Control
            //get Knob value:
            knob = analogRead(A0);
            //Get Temp Value
            actTemp = thermocouple.readFarenheit();

            
            
            //Set Temp Set
            if (knob > 100)
              setTemp = (double) ((knob - 100.0f) / (1023.0f - 100.0f) * MAXTEMP);
            else
              setTemp = 0;


 

            if(!isnan(actTemp)) 
              PIDIn = actTemp;
            PIDSet = setTemp;
   
            heatPID.Compute();

            if (knob > 100)
              heatValue = PIDOut;
            else
              heatValue = 0.0;
            
              
            //On Button A Press show Mode
            if(buttonA)
            {
              ledA = 13; //n
              ledB = mode;
              
            } 
            else  //Show Temp Set Value
            {
              
              if (setTemp == 0)
              {
                ledA = 0;
                ledB = 10; //F
              }
              else 
              {
                ledA = (int) setTemp / 1000;
                ledB = (int) (setTemp / 100) % 10;
              }
            }

            //Check for Cancelation

            if(buttonB)
            {
              if(buttonBCount >= CANCELTIME)
              {
                heatValue = 0.0f;
                state = 0;
                heatPID.SetMode(MANUAL);
              }
              else
              {
                ledA = -1;
                ledB = CANCELTIME - buttonBCount;
              }
              
            }

            //Debug
            if((millis() - delayMillis) > 5000)
            {
              delayMillis = millis();
              Serial.print("Act Temp, ");
              Serial.print(actTemp);
              Serial.print(",Set Temp, ");
              Serial.print(setTemp);
              Serial.print(",Heat Val, ");
              Serial.print(heatValue);
              Serial.print(", Internal,");
              Serial.print(thermocouple.readInternal()*1.8+32);
              Serial.print(",Min, ");
              Serial.println((float) (millis()-modeStartMillis) / 60.0f / 1000.0f);
              
              
            }
            break;
            
        case 3: //Ramp Slow 108F/hour
        case 4: //Ramp Med  189F/hour
        case 5: //Ramp Fast 270F/hour
            //get Knob value:
            knob = analogRead(A0);
            //Get Temp Value
            actTemp = thermocouple.readFarenheit();

            double maxTemp;
            
            //Max Temp Set
            if (knob > 100)
              maxTemp = (double) ((knob - 100.0f) / (1023.0f - 100.0f) * MAXTEMP);
            else
              maxTemp = 0;
              
            //Set Temp Set
            if(mode == 3)
              setTemp = startTemp + 108.0 / 60 / 60 / 1000 * (millis() - modeStartMillis);
            if(mode == 4)
              setTemp = startTemp + 189.0 / 60 / 60 / 1000 * (millis() - modeStartMillis);
            if(mode == 5)
              setTemp = startTemp + 270.0 / 60 / 60 / 1000 * (millis() - modeStartMillis);

            if(!isnan(actTemp)) 
              PIDIn = actTemp;
            
            PIDSet = setTemp;
   
            heatPID.Compute();

            

            //Reset Millis Timer if still in ramp up and temp is below set
            if((actTemp < maxTemp) && !tempReached)
               tempReachedMillis = 0;

            //Log when temp exceeds maxTemp
            if ((actTemp > maxTemp) && (tempReachedMillis == 0))
              tempReachedMillis = millis();

            if ((tempReachedMillis != 0) && ((millis() - tempReachedMillis) > TEMPREACHEDDELAY))
              tempReached = true;

            //Run Heater
            if (!tempReached)
              heatValue = (float) PIDOut;
            else
              heatValue = 0.0f;
                
                          
            //On Button A Press show Mode
            if(buttonA)
            {
              ledA = 13; //n
              ledB = mode;
              
            } 
            else  //Show Temp Set Value or CL for cool
            {
              
              if (maxTemp == 0)
              {
                ledA = 0;
                ledB = 10; //F
              }
              else 
              {
                if(!tempReached)
                {
                  ledA = (int) maxTemp / 1000;
                  ledB = (int) (maxTemp / 100) % 10;
                }
                else
                {
                  ledA = 14; //C
                  ledB = 1;
                }
              }
            }

            //Check for Cancelation

            if(buttonB)
            {
              if(buttonBCount >= CANCELTIME)
              {
                heatValue = 0.0f;
                state = 0;
              }
              else
              {
                ledA = -1;
                ledB = CANCELTIME - buttonBCount;
              }
              
            }

            //Exit
            if(tempReached && (actTemp < COOLTEMP))
            {
               heatValue = 0.0f;
               state = 0;
            }

            //Debug
            if((millis() - delayMillis) > 5000)
            {
              delayMillis = millis();
              Serial.print("Act Temp, ");
              Serial.print(actTemp);
              Serial.print(",Set Temp, ");
              Serial.print(setTemp);
              Serial.print(",Heat Val, ");
              Serial.print(heatValue);
              Serial.print(",Min, ");
              Serial.println((float) (millis()-modeStartMillis) / 60.0f / 1000.0f);
              
            }
              
          break;
        case 6: //Burnout
          //Generate Setpoint based on Form1 Burnout Cycle
          unsigned long runtime =  millis() - modeStartMillis;

          if( runtime < 5400000) {
            if (startTemp < 350)
              setTemp = map(runtime, 0, 5400000, startTemp, 350);
            else
              setTemp = 350;
          } else if( runtime < 7200000 ) {
            setTemp = 350;
          } else if(runtime < 23400000) {
            setTemp = map(runtime, 7200000, 23400000, 350, 1350); 
          } else if( runtime < 34200000 ) {
            setTemp = 1350;
          } else if( runtime < 43200000 ) {
            setTemp = map(runtime, 34200000, 43200000, 1350, 900); 
          } else if( runtime < 46800000 ) {
             setTemp = 900;
          } else {
             setTemp = 0;
          }


          

          //Get Temp Value
          actTemp = thermocouple.readFarenheit();

          //Run PID
          
          if(!isnan(actTemp)) 
            PIDIn = actTemp;  
          else
            Serial.println("Temp Fail!");
          PIDSet = setTemp;

          
          
          heatPID.Compute();
          if(runtime < 46800000)
            heatValue = PIDOut;
          else
            heatValue = 0.0;

         if(buttonB)
          {
            if(buttonBCount >= CANCELTIME)
            {
              heatValue = 0.0f;
              state = 0;
            }
            else
            {
              ledA = -1;
              ledB = CANCELTIME - buttonBCount;
            }
            
          } 
          else if(buttonA)
            {
              ledA = 13; //n
              ledB = mode;
              
          } 
          else {
            //Cycle CountDown Timer EachDigit is 10 mins
            if(runtime < 46800000)
            {
              ledA = ((int) map(runtime/100000, 0, 468, 78, 0)) / 10; //Divide to avoid map overflow
              ledB = ((int) map(runtime/100000, 0, 468, 78, 0)) % 10; //Divide to avoid map overflow
            } else {
              ledA = 14; //C
              ledB = 1;
            }
          }

          //Exit
          if((runtime > 46800000) && (actTemp < COOLTEMP))
          {
             heatValue = 0.0f;
             state = 0;
          }

          //Debug
          if((millis() - delayMillis) > 60000)
          {
            delayMillis = millis();
            Serial.print("Act Temp, ");
            Serial.print(actTemp);
            Serial.print(",Set Temp, ");
            Serial.print(setTemp);
            Serial.print(",Heat Val, ");
            Serial.print(heatValue);
            Serial.print(",Min, ");
            Serial.println((float) (millis()-modeStartMillis) / 60.0f / 1000.0f);
            
          }
          break;

        

      }  
      break;
  }

  delay(100);
  return;


  
  

 



    
  /*
    Serial.println(knob);
  // basic readout test, just print the current temp
  
   
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = "); 
     Serial.println(c);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFarenheit());
 
   delay(1000);
   */
}



