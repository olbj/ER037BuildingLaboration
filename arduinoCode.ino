/************************************************************
* File: _jobb3m.ino
* Author: Jörgen Stegeby
* Date: 2014-10-03
*
* Det här Arduino programmet kontrollerar temperaturen i ett hus
* med två rum i två våningar. Bottenvåningen värms med ett 200W
* element och av/på-reglering. Det övre rummet får sin värme 
* från det undre rummet genom en fläkt som drar upp den varma 
* luften. Fläkten kan kontrolleras antingen vis PID reglering
* (förvald) eller av/på-reglering som kan väljas från PC GUI.
* Temperatuen mäts med två temperatursensorer som är kopplade 
* Arduinos analogingångar A0 för det under rummet och A1 för det
* övre rummet.
* PC GUI (Grafiskt användar gränssnitt - Graphical User Interface
* är skrivet för Processing och körs på PC. Man måste ställa in 
* rätt COM-post (den COM-port som Arduino är kopplat till) i 
* Processing.
* 
* This Arduino program controls temperature on a house with two
* stories. The bottom floor has a heater with on/off regulation.
* The topfloor has a fan moving warm air from the bottom floor
* to the top floor. The fan can be controlled by PID regulation 
* (default) or on/off regulation selectable from the PC GUI.
* The temperature is monitiered with two heat sensors connected 
* to analog inputs AO for the lower room and A1 for the top room.
* The PC GUI (Graphical User Insterface) is written for Processing
* and is run on the PC. It is important to set the corret COM-
* port in Processing. It should be the same port as the Arduino is
* connected to.
* 
*************************************************************/
#include "Wire.h"
#include "SparkFunTMP102.h" // Used to send and recieve specific information from our sensor

// Definierar of Arduino pinnar och delaytime
#define FANCONTROL 10
#define LED5 12
#define delaytime 1000

// initierar variabler - these are expained when initilized below
TMP102 sensor0(0x48); // Initialize sensor at I2C address 0x48
TMP102 sensor1(0x49); // Second sensor initialization
TMP102 sensor2(0x4A); // third sensor initialization
  int  val, state1, state2, state3, ctrl, pwmvalue, comp;
  float temp1, temp2, t1b, t2b, diff1, diff2, t1min, t2min, tempext;
  int temp1p, temp2p, t1bp, t2bp, diff1p, diff2p, t1minp, t2minp;
  
// PID variabler
double error, lasterror = 0.0, sumerror, derror, output, PIDvalue;
//double kp = 7;
////double ki = 0;
////double kd = 0;
//double ki = 0.0001;
//double kd = 0.2;


double kp = 7;
//double ki = 0;
//double kd = 0;
double ki = 0.0001;
double kd = 0.2;

// Setup körs en gång vid start och när man trycker RESET
void setup() {
  // initierar seriekommunikation till 9600 bits per sekund
  Serial.begin(115200);
  pinMode(LED5, OUTPUT); // sätter pinne 12 till OUTPUT

  t1b = 30;   // börvärdestemperatur för under rummet 35C
  t2b = 29;   // börvärdestemperatur för övre rummet 32C

  diff1 = 2;    // skillnad mellan börvärtdet och undre 
                // värdet för att starta elementet 4C
  diff2 = 2;    // Fläkt AV/PÅ reglering - nedre gräns
  state2 = 1;   // 0 = Seial.print    1 = Serial.write
  state1 = 1;   // 0 = element av     1 = element på
  state3 = 1;   // 0 = har varit över T2bör 1 = är på väg upp
  ctrl   = 0;   // 0 = PID reglering  1 = AV/PÅ reglering
  //temp1 = 165;  // simulerade värden för Tempsensor 1 nedre rummet
  //temp2 = 160;  // simulerade värden för Tempsensor 1 övre rummet
  comp = 30;    // Min värde då PWM blir för lågt för att 
                // fläkten ska fungera

///////////////////////////////////////
////// TEMPERATURE SENSORS ////////////
///////////////////////////////////////

  sensor0.begin();  // Join I2C bus
  
  // Initialize sensor0 settings
  // These settings are saved in the sensor, even if it loses power
  
  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor0.setFault(0);  // Trigger alarm immediately
  
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor0.setAlertPolarity(1); // Active HIGH
  
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor0.setAlertMode(0); // Comparator Mode.
  
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor0.setConversionRate(2);
  
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor0.setExtendedMode(0);

  //set T_HIGH, the upper limit to trigger the alert on 
  //sensor0.setHighTempF(85.0);  // set T_HIGH in F
  sensor0.setHighTempC(150); // set T_HIGH in C
  
  //set T_LOW, the lower limit to shut turn off the alert
  //sensor0.setLowTempF(84.0);  // set T_LOW in F
  sensor0.setLowTempC(0); // set T_LOW in C


  ///////////////////
  sensor1.begin();  // Join I2C bus
  
  // Initialize sensor1 settings
  // These settings are saved in the sensor, even if it loses power
  
  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor1.setFault(0);  // Trigger alarm immediately
  
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor1.setAlertPolarity(1); // Active HIGH
  
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor1.setAlertMode(0); // Comparator Mode.
  
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor1.setConversionRate(2);
  
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor1.setExtendedMode(0);

  //set T_HIGH, the upper limit to trigger the alert on 
  //sensor0.setHighTempF(85.0);  // set T_HIGH in F
  sensor1.setHighTempC(150); // set T_HIGH in C
  
  //set T_LOW, the lower limit to shut turn off the alert
  //sensor0.setLowTempF(84.0);  // set T_LOW in F
  sensor1.setLowTempC(0); // set T_LOW in C

  ///////////////////
  sensor1.begin();  // Join I2C bus
  
  // Initialize sensor2 settings
  // These settings are saved in the sensor, even if it loses power
  
  // set the number of consecutive faults before triggering alarm.
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  sensor2.setFault(0);  // Trigger alarm immediately
  
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  sensor2.setAlertPolarity(1); // Active HIGH
  
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  sensor2.setAlertMode(0); // Comparator Mode.
  
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  sensor2.setConversionRate(2);
  
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  sensor2.setExtendedMode(0);

  //set T_HIGH, the upper limit to trigger the alert on 
  //sensor0.setHighTempF(85.0);  // set T_HIGH in F
  sensor2.setHighTempC(150); // set T_HIGH in C
  
  //set T_LOW, the lower limit to shut turn off the alert
  //sensor0.setLowTempF(84.0);  // set T_LOW in F
  sensor2.setLowTempC(0); // set T_LOW in C
}

// Programmlopen som går i evighet:
void loop() {
  
// Kontrollerar om det finns data på serieporten
   if (Serial.available() > 0) { // Om data kommit in på serieporten,
      val = Serial.read(); // läs värdet och spara i variabel val
 //       Serial.println(val);  // debug kod
 
 // Utför ändringar på variabler beroende på vilken kod som 
 // mottogs från Processing
      switch(val){
        case 1:
          t1bp++;  // ökar T1börvärde med 1
          t1b = t1b + 0.2; // for float precision
        break;
        case 2:
          t1bp--;  // minskar T1börvärde med 1
          t1b = t1b - 0.2; // for float precision
        break;
        case 3:
          diff1p--;  // ökar min värdet genom att minska skillnaden
          diff1 = diff1 - 0.2;
        break;
        case 4: 
          diff1p++;  // minskar min värdet genom att öka skillnaden
          diff1 = diff1 + 0.2;
        break;
        case 5:
          t2bp++;  // ökar T2börvärde med 1
          t2b = t2b + 0.2;
        break;
        case 6:
          t2bp--;  // minskar T1börvärde med 1
          t2b = t2b - 0.2;
        break;
        case 7:
          ctrl = 0;  // Fläkt PID reglering
        break;
        case 8:
          ctrl = 1;  // Fläkt AV/PÅ reglering
        break;
        case 9:
          diff2p++;  // ökar min värdet för fläkt AV/PÅ reglering
          diff2 = diff2 + 0.2;
        break;
        case 0:
          diff2p--;  // minskar min värdet för fläkt AV/PÅ reglering
          diff2 = diff2 - 0.2;
        break;       
        } // end switch(val)
    } // end if Serial.available()
  
t1min = t1b-diff1; // beräknar minvärde för under rummet
t2min = t2b-diff2; // beräknar minvärde för övre rummet

//temp1 = analogRead(A0);  // läser tempmätaren för undre rummet
//temp2 = analogRead(A1);  // läser tempmätaren för undre rummet
sensor0.wakeup();
//delay(10);
temp1 = sensor0.readTempC(); // läser tempmätaren för undre rummet
sensor0.sleep(); 
sensor1.wakeup();
//delay(10);
temp2 = sensor1.readTempC();  // läser temperaturen för övre rummet
sensor1.sleep();
sensor2.wakeup();
//delay(10);
tempext = sensor2.readTempC();  // läser temperaturen för övre rummet
sensor2.sleep();

//// Skriver värden till serieporten för Serial monitor
//  if(!state2) {
//    Serial.print("255 ");  // Talar om för Processing att T1bör kommer
//    Serial.println(t1b);  // skickar T1börvärde
//    Serial.print("254 ");  // Talar om för Processing att T2bör kommer
//    Serial.println(t2b);  // skickar T2börvärde
//    Serial.print("253 ");  // Talar om för Processing att Temp 1 nedre rummet kommer
//    Serial.print(temp1);// Skickar Temp 1 värde för nedre rummet
//    Serial.print("252 ");  // Talar om för Processing att Temp 2 nedre rummet kommer
//    Serial.print(temp2);// Skickar Temp 2 värde för nedre rummet
//    Serial.print("251 ");  // Talar om för Processing att T1min kommer
//    Serial.print(t1min);//Skickar Temp 1 min värde för nedre rummet
//    Serial.print("250 ");  // Talar om för Processing att elementstatus kommer
//    Serial.print(state1);//Skickar elementstatus
//    Serial.print("248 ");  // Talar om för Processing att inkrementera tidvariabeln i Processing
//    Serial.print("247 ");  // Talar om fläktreglering mode
//    Serial.print(ctrl); // Skickar fläktreglerings mode
//    Serial.print("246 ");  // Talar om för Processing att T2min kommer
//    Serial.print(t2min);// Skickar T2min
//    Serial.print("245 ");  // Talar om för Processing om temperaturen för Fläkt AV/PÅ är på väg upp eller ner
//    Serial.print(state3);// Skickar status på temperaturriktning
//    Serial.print("244 ");  // Ritar ut vita sträck för att radera tidigare graf   
//
//    }
// Skriver värden till serieporten för Processing
// Första värdet 255-244 talar om för Processing 
// vilket värde som följer och är skrivet som ett nummer
// Nästa värde är en variabel med ett värde som Processing 
// kommer att använda

// The values processing will read and interpreat, to increase precision.
t1bp    = round(t1b*5); // in Processing we can display the correct values by
t2bp    = round(t2b*5); // dividing by 5.
temp1p  = round(temp1*5);
temp2p  = round(temp2*5);
t1minp  = round(t1min*5);
t2minp  = round(t2min*5);
//Serial.println(t1b);
//Serial.println(t1bp);
//  if(state2) {
    Serial.write(255);  // Talar om för Processing att T1bör kommer
    Serial.write(t1bp);  // skickar T1börvärde
    Serial.write(254);  // Talar om för Processing att T2bör kommer
    Serial.write(t2bp);  // skickar T2börvärde
    Serial.write(253);  // Talar om för Processing att Temp 1 nedre rummet kommer
    Serial.write(temp1p);// Skickar Temp 1 värde för nedre rummet
    Serial.write(252);  // Talar om för Processing att Temp 2 nedre rummet kommer
    Serial.write(temp2p);// Skickar Temp 2 värde för nedre rummet
    Serial.write(251);  // Talar om för Processing att T1min kommer
    Serial.write(t1minp);//Skickar Temp 1 min värde för nedre rummet
    Serial.write(250);  // Talar om för Processing att elementstatus kommer
    Serial.write(state1);//Skickar elementstatus
    Serial.write(248);  // Talar om för Processing att inkrementera tidvariabeln i Processing
    Serial.write(247);  // Talar om fläktreglering mode
    Serial.write(ctrl); // Skickar fläktreglerings mode
    Serial.write(246);  // Talar om för Processing att T2min kommer
    Serial.write(t2minp);// Skickar T2min
    Serial.write(245);  // Talar om för Processing om temperaturen för Fläkt AV/PÅ är på väg upp eller ner
    Serial.write(state3);// Skickar status på temperaturriktning
    Serial.write(244);  // Ritar ut vita sträck för att radera tidigare graf   

//    }
    



/*****************************************************/
// AV/PÅ reglering av element i nedre rummet
/*****************************************************/

 // kontroll om den avlästa temperaturen är mindre än den satta
 // temperaturen för nedre rummet och temperaturen är på väg upp.
  if (temp1p < t1bp && state1 == 1){
     digitalWrite(LED5 , HIGH);   
  }

 // kontroll om den avlästa temperaturen är mindre än den satta
 // temperaturen för nedre rummet och temperaturen är på väg ner.
  if (temp1p < t1bp && state1 == 0){
     digitalWrite(LED5 , LOW); 
  }
 
 // kontroll om den avlästa temperaturen är högre än den satta
 // temperaturen för nedre rummet. 
  if (temp1p > t1bp && state1 == 1){
    state1 = 0;
     digitalWrite(LED5 , LOW); 
  }

 // kontroll om den avlästa temperaturen är lägre än 
 // mintemperaturen 
  if (temp1p < t1minp){
    state1 = 1;
     digitalWrite(LED5 , HIGH); 
  }
  

/*****************************************************/
// PID reglering då ctrl=0
/*****************************************************/

if (ctrl == 0)
{  
// Beräknar error, sum of error over time och derror.
  error = t2bp - temp2p;
  sumerror += (error * delaytime);
  derror = (error - lasterror) / delaytime;
  lasterror = error;
  PIDvalue = kp * error + ki * sumerror + kd * derror ;
  output = PIDvalue;
  
  // begränsar output till 0-255 området då output är större
  // än 250 och mindre än 0 
    if (output > 250)
    {
      output = 250;
    }
    
    if (output < comp)
    {
      output = 0;
    }

 // skickar output värdet till fläktstyrpinnen
  analogWrite(FANCONTROL, output);
  
       if(state2) 
       {
           Serial.write(249);
           Serial.write(int(output)/10);
       }
       if(!state2) 
       {
           Serial.print("249");
           Serial.println(PIDvalue); 
       }  
} // end if(ctrl == 0)


/*****************************************************/
// AV/PÅ reglering för fläkten då ctrl=1
/*****************************************************/

  if (ctrl == 1){
    lasterror = 0;
   // kontroll om den avlästa temperaturen är mindre än den satta
   // temperaturen för övre rummet och temperaturen är på väg upp.
      if (temp2p < t2bp && state3 == 1){
           analogWrite(FANCONTROL, 255); 
      }     
      
   // kontroll om den avlästa temperaturen är mindre än den satta
   // temperaturen för övre rummet och temperaturen är på väg ner.
      if (temp2p < t2bp && state3 == 0){
           analogWrite(FANCONTROL, 0); 
      }     
      
   // kontroll om den avlästa temperaturen är högre än den satta
   // temperaturen för övre rummet. 
      if (temp2p < t2minp){
           analogWrite(FANCONTROL, 255);
           state3 = 1;
      }     

 // kontroll om den avlästa temperaturen är lägre än 
 // mintemperaturen        
      if (temp2p > t2bp){
           analogWrite(FANCONTROL, 0); 
           state3 = 0;
      }     
       
} // end if (ctrl == 1)

// slår av fläkten om börtemperaturen är uppnådd
  if (temp2p > t2bp && ctrl == 1) { 
     analogWrite(FANCONTROL, 0); 
  }
delay(delaytime); // another one-second delay
  Serial.print("R"); // Flag Read || 0
  Serial.print(",");
  Serial.print(temp1); // Values sensor 1 || 1
  Serial.print(",");
  Serial.print(temp2); // Values sensor 2 || 2
  Serial.print(",");
  Serial.print(t1b);          // Setpoint t1 || 3
  Serial.print(",");
  Serial.print(t1min);       // Hysteresis value setpoint1 || 4
  Serial.print(",");
  Serial.print(t2b);          // Setpoint t2 || 5
  Serial.print(",");
  Serial.print(t2min);       // Hysteresis point setpoint2 || 6
  Serial.print(",");
  Serial.print(state1);       // Check the status of On/off sensor 1. If 1= temp is rising, if 0 is lowering down
  Serial.print(",");          // || 7
  Serial.print(state3);       // Check the status of On/off sensor 2. If 1= temp is rising, if 0 is lowering down
  Serial.print(",");          // || 8
  Serial.print(ctrl);       // On/off or PID. "0" is PID, "1" On/Off
  Serial.print(",");        // || 9
  Serial.print(PIDvalue);       // Value PID
  //Serial.print(float(output));       // Value PID
  Serial.print(",");        // || 10
  Serial.println(tempext);       // External Temperature || 11

  delay(delaytime/10); // another 10/one-second delay
  
} // end loop()
