// AC Voltage Sensor with LCD By Solarduino (Revision 2)

// Note Summary
// Note :  Safety is very important when dealing with electricity. We take no responsibilities while you do it at your own risk.
// Note :  This AC Votlage Sensor Code is for Single Phase AC Voltage transformer ZMPT101B module use.
// Note :  The value shown in Serial Monitor / LCD Display is refreshed every second and is the average value of 4000 sample readings.
// Note :  The voltage measured is the Root Mean Square (RMS) value.
// Note :  The analog value per sample is squared and accumulated for every 4000 samples before being averaged. The averaged value is then getting square-rooted.
// Note :  The auto calibration (voltageOffset1) is using averaged analogRead value of 4000 samples.
// Note :  The auto calibration (currentOffset2) is using calculated RMS current value including currentOffset1 value for calibration.  
// Note :  The unit provides reasonable accuracy and may not be comparable with other expensive branded and commercial product.
// Note :  All credit shall be given to Solarduino.

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/////////////*/


/* 0- General */

        int decimalPrecision = 2;                   // decimal places for all values shown in LED Display & Serial Monitor

        /* 1- AC Voltage Measurement */
        
        int VoltageAnalogInputPin = A2;             // Which pin to measure voltage Value (Pin A0 is reserved for button function)
        float voltageSampleRead  = 0;               /* to read the value of a sample in analog including voltageOffset1 */
        float voltageLastSample  = 0;               /* to count time for each sample. Technically 1 milli second 1 sample is taken */
        float voltageSampleSum   = 0;               /* accumulation of sample readings */
        float voltageSampleCount = 0;               /* to count number of sample. */
        float voltageMean ;                         /* to calculate the average value from all samples, in analog values*/ 
        float RMSVoltageMean ;                      /* square roof of voltageMean without offset value, in analog value*/
        float adjustRMSVoltageMean;
        float FinalRMSVoltage;                      /* final voltage value with offset value*/
        int beginCount = 0;                         /* to initiate counting*/
        
        /* 1.1- AC Voltage Offset */
        
              int OffsetRead = 0;                   /* To switch between functions for auto callibation purpose */   
              float offsetSampleReadV;              /* sample for offset purpose */
              float offsetSampleSumV = 0;           /* accumulation of sample readings for offset */
              float voltageOffset1 =0.00 ;          // to Offset deviation and accuracy. Offset any fake current when no current operates. 
                                                    // Offset will automatically callibrate when SELECT Button on the LCD Display Shield is pressed.
                                                    // If you do not have LCD Display Shield, look into serial monitor to add or minus the value manually and key in here.
                                                    // 26 means add 26 to all analog value measured.
              float voltageOffset2 = 0.00;          // too offset value due to calculation error from squared and square root 
              float offsetVoltageMean;
              float offsetLastSample = 0;           /* to count time for each sample. Technically 1 milli second 1 sample is taken */
              float offsetSampleCount = 0;          /* to count number of sample. */
              

        /* 2 - LCD Display  */
    
        #include<LiquidCrystal.h>                   /*Load the liquid Crystal Library (by default already built-it with arduino solftware)*/
        LiquidCrystal LCD(8,9,4,5,6,7);             /*Creating the LiquidCrystal object named LCD */
        unsigned long startMicrosLCD;               /* start counting time for LCD Display */
        unsigned long currentMicrosLCD;             /* current counting time for LCD Display */
        const unsigned long periodLCD = 1000000;    // refresh every X seconds (in seconds) in LED Display. Default 1000 = 1 second 


void setup() {
 
/* 0- General */

    Serial.begin(9600);                             /* In order to see value in serial monitor */
    
/* 2 - LCD Display  */

    LCD.begin(16,2);                                /*Tell Arduino that our LCD has 16 columns and 2 rows*/
    LCD.setCursor(0,0);                             /*Set LCD to upper left corner of display*/  
    startMicrosLCD = micros();

}
     
void loop() 

{
   
    /* 0- General */


              /* 0.1- Button Function */
        
              int buttonRead;
              buttonRead = analogRead (0);                                    // Read analog pin A0. Pin A0 automatically assigned for LCD Display Button function (cannot be changed)

              /*Right button is pressed */
              if (buttonRead < 60) 
              {   LCD.setCursor(0,0); LCD.print ("PRESS <SELECT>   "); }       
     
              /* Up button is pressed */
              else if (buttonRead < 200) 
              {   LCD.setCursor(0,0); LCD.print ("PRESS <SELECT>   "); }    
                 
              /* Down button is pressed */
              else if (buttonRead < 400)
              {   LCD.setCursor(0,0); LCD.print ("PRESS <SELECT>  ");  }      
     
              /* Left button is pressed */
              else if (buttonRead < 600)
              {   LCD.setCursor(0,0); LCD.print ("PRESS <SELECT>   "); } 
     
              /* Select button is pressed */
              else if (buttonRead < 800)
              {   
              OffsetRead = 1;                                                 // to activate offset 
              LCD.setCursor(0,0);
              LCD.print ("INITIALIZING..... ");
              LCD.setCursor(0,1);
              LCD.print ("WAIT 5 SEC ..... ");
              }
   
   /* 1- AC Voltage Measurement */
        if(micros() >= (voltageLastSample + 1000))                                                                      /* every 0.5 milli second taking 1 reading */
          {
            offsetSampleReadV = (analogRead(VoltageAnalogInputPin)- 512);                                             /* read the sample value for offset purpose*/
            offsetSampleSumV = offsetSampleSumV + offsetSampleReadV;                                                  /* Accumulate analog values from each sample for offset purpose*/
            
            voltageSampleRead = (analogRead(VoltageAnalogInputPin)- 512)+ voltageOffset1;                             /* read the sample value including offset value*/
            voltageSampleSum = voltageSampleSum + sq(voltageSampleRead) ;                                             /* accumulate total analog values for each sample readings*/
            
            voltageSampleCount = voltageSampleCount + 1;                                                              /* to move on to the next following count */
            voltageLastSample = micros() ;                                                                            /* to reset the time again so that next cycle can start again*/ 
          }
        
        if(voltageSampleCount == 1000)                                                                                /* after 4000 count or 800 milli seconds (0.8 second), do the calculation and display value*/
          {
            offsetVoltageMean = offsetSampleSumV / voltageSampleCount;                                                /* average accumulated analog values for offset purpose */
            voltageMean = voltageSampleSum/voltageSampleCount;                                                        /* calculate average value of all sample readings taken*/
            RMSVoltageMean = (sqrt(voltageMean))*1.5;                                                                 // The value X 1.5 means the ratio towards the module amplification.      
            adjustRMSVoltageMean = RMSVoltageMean + voltageOffset2;                                                   /* square root of the average value including offset value */                                                                                                                                                       /* square root of the average value*/                                                                                                             
            FinalRMSVoltage = RMSVoltageMean + voltageOffset2;                                                        /* this is the final RMS voltage*/
            if(FinalRMSVoltage <= 2.5)                                                                                /* to eliminate any ghost values*/
            {FinalRMSVoltage =0;}
            Serial.print(" The Voltage RMS value is: ");
            Serial.print(FinalRMSVoltage,decimalPrecision);
            Serial.println(" V ");
            offsetSampleSumV =0;                                                                                      /* to reset accumulate offset sample values for the next cycle */
            voltageSampleSum =0;                                                                                      /* to reset accumulate sample values for the next cycle */
            voltageSampleCount=0;                                                                                     /* to reset number of sample for the next cycle */
            beginCount = 0;
          }


         /* 1.1 - Offset AC Voltage */
          
          if(OffsetRead == 1)                                                                                         /* if the SELECT button is pressed */
            {
             voltageOffset1 = 0;                                                                                      /* set back voltageOffset as default*/
               if(micros() >= offsetLastSample + 1000)                                                                   
                {  
                  offsetSampleCount = offsetSampleCount + 1;                                                                          
                  offsetLastSample = micros();                                                                          
                }                                                                             
                  if(offsetSampleCount == 1500)                                                                       /* after 1.5 seconds, run this codes.  */       
                {                                                                                                     
                  voltageOffset1 = - offsetVoltageMean;                                                               /* to offset values */
                  OffsetRead = 2;                                                                                     /* go for the next offset setting*/                      
                  offsetSampleCount = 0;                                                                              /* to reset the time again so that next cycle can start again */ 
                }                                                                             
            } 

          if(OffsetRead == 2)                                                                                         /* second offset is continued */
            {
             voltageOffset2 = 0;                                                                                      /* set back voltageOffset2 as default*/
               if(micros() >= offsetLastSample + 1000)                                                                   
                {                                                                            
                  offsetSampleCount = offsetSampleCount + 1;                                                                          
                  offsetLastSample = micros();                                                                          
                }                                                                             
                  if(offsetSampleCount == 2500)                                                                       /* after 2.5 seconds, run this codes.  */
                {                                                                                                     
                  voltageOffset2 = - RMSVoltageMean;                                                                  /* to offset values */
                  OffsetRead = 0;                                                                                     /* until next offset button is pressed*/                      
                  offsetSampleCount = 0;                                                                              /* to reset the time again so that next cycle can start again */ 
                }                                                                             
            } 

           
          /* 2 - LCD Display  */

          currentMicrosLCD = micros();
          if (currentMicrosLCD - startMicrosLCD >= periodLCD)
          {
            LCD.setCursor(0,0);                                                                                   /* Set cursor to first colum 0 and second row 1  */
            LCD.print(FinalRMSVoltage,decimalPrecision);                                                          /* display current value in LCD in first row */
            LCD.print("v                        ");  
            LCD.setCursor(0,1);
            LCD.print("                           "); 
            startMicrosLCD = currentMicrosLCD ;                                                                   /* Set the starting point again for next counting time */
          }

}