/*
 * 
 * Program: testing robot encoder counts
 * Program purpose: utilizing interupts count the encoder iteration and organize that data then transmit it serially 
 *                  Test the encoder count at different speeds and transmit the voltage of the battery. 
 * 
 * 
 */








#include <Arduino.h>
#include "pins.h"
#include "motor.h"
#include <util/atomic.h>


// Global Constants
const unsigned long ENCODER_SAMPLE_INTERVAL = 500UL;    // units, milliseconds

// Global Variables
volatile unsigned long leftEncoderCount = 0;
volatile unsigned long rightEncoderCount = 0;



/* Interrupt Service Routine
 *  updates left encoder count
 */
void leftEncoderISR(void)
{
  leftEncoderCount++;
}
/*
 * initializes the voltage to be taken 
 */
void voltageInit()
{
  analogReference(INTERNAL);
}

/* Interrupt Service Routine
 *  updates left encoder count
 */

ISR(PCINT2_vect)
{
  rightEncoderCount++;
}

ISR(PCINT0_vect)
  {
    rightEncoderCount++;
  }

  
void mysetup(void)
{
  pinMode(LEFT_ENCODER_A_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN), leftEncoderISR, CHANGE);
  voltageInit();
  // setup for a pin change interupt 
  cli();//turn off interupts 
  PCICR |= 0b00000100; //enables port D pin change interrupts 
  PCMSK2 |= 0b00010000; //PCINT20
  sei();//enable interupts 
  
}

int main(void){

  unsigned char motorSpeed = 64; // This is 25% duty cycle or 25% of 255

  unsigned long startTime, dTEncoder;
  int count = 0;
  int i = 0;
  unsigned long encoder_count_left[32] = {0};
  unsigned long encoder_count_right[32] = {0};
  unsigned long elapsed_time[32] = {0};
  float battery_voltage[32] = {0};
  int increment_count[32] = {0};
 
  
  init();
  mysetup(); 
  
  Serial.begin(115200);
  initMotors();


  
  while(1)
  {
    if(count == 0){
      driveForward(motorSpeed); 
      delay(2000); //ensure that motor gets up to speed 
      startTime = millis();
    }

    
     

    while(count < 32){
      if( (millis() - startTime) >=  ENCODER_SAMPLE_INTERVAL)
      {
        //keeps the time and stores it in the array
        dTEncoder = millis() - startTime;
        elapsed_time[count] = {dTEncoder};        
        startTime = millis();
      
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          // code with interrupts blocked (consecutive atomic operations will not get interrupted)

          //set the counts and voltage into the arrays created 
          increment_count[count] = count; 
          encoder_count_left[count] = leftEncoderCount;
          encoder_count_right[count] = rightEncoderCount;
          battery_voltage[count] = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
            
          
        }
          if(count == 30){
            for(int i = 0; i < 31; i++){
              Serial.print(increment_count[i]);
              Serial.print(",");
              Serial.print(elapsed_time[i]);
              Serial.print(",");
              Serial.print(encoder_count_left[i]);
              Serial.print(",");
              Serial.print(encoder_count_right[i]);
              Serial.print(",");
              Serial.print(battery_voltage[i]);
              Serial.print(",");
              Serial.println(" ");
            }  
          }
        

        if(count == 10){
            motorSpeed = 128;//This is 50% duty cycle
            if(i==0){
              driveForward(motorSpeed); 
              delay(2000);//to ensure motor gets up to speed 
              startTime = millis();                
            }

            if(i==1){
              driveBackward(motorSpeed);
              delay(2000);//to ensure motor gets up to speed 
              startTime = millis();
            }
        }
        
          if(count == 20){
            motorSpeed = 191;//This is 75% duty cycle
            if(i==0){
              driveForward(motorSpeed); 
              delay(2000);//to ensure motor gets up to speed 
              startTime = millis();
            }

            if(i==1){
              driveBackward(motorSpeed);
              delay(2000);//to ensure motor gets up to speed 
              startTime = millis();
            }
          }

           if(count == 30 & i == 0){
            stopMotors();
            delay(1000);//keeps the motor from switching directions to fast 
            motorSpeed = 64;//this is 50% duty cycle 
            driveBackward(motorSpeed);
            delay(2000);//to ensure motor gets up to speed 
            startTime = millis();
            count = 0;
            i = 1;
           }
            
           if(count == 30 & i == 1){
            stopMotors();
           }
          count++;
          leftEncoderCount = 0UL;       // 0 is type int, 0UL is type unsigned long
          rightEncoderCount = 0UL;

  
      }
      
    }


    

   
    

   }
  
  return 0;
}
