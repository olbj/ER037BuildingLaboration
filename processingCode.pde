/*************************************************************** 
* Denna kod skapar ett UI och läser in seriedata från
* Arduino. Dessa värden är T1, T2, T1min, T2min, T1bör, T2bör
* och visar dessa på skärmen som en plot
* Rader markerade med '////' (fyra kommentartecken) är debugkod
* 
* Jörgen Stegeby (C) 2014
****************************************************************/

import processing.serial.*;

Serial port; // Create object from Serial class
int x,y,val,val1,mouseclick,tid,fan, valold;
float val2;
int count1,count2,count,counting, energytab;

Table dataTable; // Store values from Arduino
int numReadings = 100; //keeps track of how many readings you'd like to take before writing the file. 
int readingCounter = 0; //counts each reading to compare to numReadings. 
String fileName, valSave;
String[] sensValTemp, sensVal;

color myred = color(255,0,0);
color mygreen = color(0,255,0);
color myblue   = color(0,0,255);
color mydarkgreen = color(0,128,0);
color mydarkblue  = color(0,144,128);


void setup() {
  size(900, 620); // size of the user interface window
  println(Serial.list()); // prints the active COM-port list
  //String arduinoPort = Serial.list()[0]; //COM3
  //String arduinoPort = Serial.list()[1]; //COM5
  String arduinoPort = Serial.list()[2]; //COM6
  //String arduinoPort = Serial.list()[3];
  println(arduinoPort);
  
  port = new Serial(this, arduinoPort, 115200);
  
  //fill(0);
  //size(900, 610); // size of the user interface window
  rect(80, 40, 790, 540);  // Draw the graph window
    // Ritar upp boxar
  rect(120, 15, 20, 20);    // Draw the T1+ box
  rect(150, 15, 20, 20);    // Draw the T1- box
  rect(240, 15, 20, 20);    // Draw the T1min+ box
  rect(270, 15, 20, 20);    // Draw the T1min- box
  rect(360, 15, 20, 20);    // Draw the T2+ box
  rect(390, 15, 20, 20);    // Draw the T2- box
  rect(490, 15, 35, 20);    // Draw the PID box
  rect(535, 15, 50, 20);    // Draw the AV/PÅ box
  rect(595, 15, 20, 20);    // Draw the AV/PÅ+ box
  rect(625, 15, 20, 20);    // Draw the AV/PÅ- box
  
  smooth();
  PFont font;
  font = createFont("Arial",8,true); // Arial, 16 point, anti-aliasing on
  fill(0);
  textFont(font);
 
  // Draw the grading lines on left side with every 5 line longer
    for(y=580 ; y>40 ; y=y-10) {
      if (y==580||y==530||y==480||y==430||y==380||y==330||y==280||y==230||y==180||y==130||y==80) { x=10; }
      else { x=0; }
    line(75-x, y, 80, y); // Draw the line
    }
    for(y=0 ; y<800 ; y=y+10) {
      if (y==0||y==50||y==100||y==150||y==200||y==250||y==300||y==350||y==400||y==450||y==500||y==550||y==600||y==650||y==700||y==750||y==800) { x=4; }
      else { x=0; }
    line(80+y, 581, 80+y, 584+x); // Draw the line
    }
    
  fill(0);
  textSize(14);
  for(y=580 ; y>40 ; y=y-50) {
    text((580-y)/10, 45, y+5);// Draw the 0 at the bottom under the line
  }
  text("Temp (°C)", 5, 45);// Draw the Start text
  text("Tid 1 sek/pixel", 250, 605);// Draw the Tid text  
  text("5 pixlar/sträck", 450, 605);// Draw the Tid text
  text("50 pixlar/längre sträck", 650, 605);// Draw the Tid text
    
    text("T1:", 95, 30);// Draw the T1 text
    text("+", 126, 30);// Draw the T1 + text
    text("-", 159, 30);// Draw the T1 - text
    text("T1min:", 195, 30);// Draw the T1min text
    text("+", 246, 30);// Draw the T1min + text
    text("-", 279, 30);// Draw the T1min - text
    text("T2:", 335, 30);// Draw the T2 text
    text("+", 366, 30);// Draw the T2 + text
    text("-", 399, 30);// Draw the T2 - text
    text("Fläkt:", 450, 30);// Draw the Fläkt text
    text("PID", 496, 30);// Draw the PID text
    text("AV/PÅ", 540, 30);// Draw the AV/PÅ text
    text("+", 601, 30);// Draw the T2 + text
    text("-", 634, 30);// Draw the T2 - text
    energytab = 0;
    
    /// 12.10.2016 Javier Bruges -> Introducing new code
    
    dataTable = new Table();
   
    dataTable.addColumn("id"); //This column stores a unique identifier for each record. We will just count up from 0 - so your first reading will be ID 0, your second will be ID 1, etc. 
    
    //the following adds columns for time. You can also add milliseconds. See the Time/Date functions for Processing: https://www.processing.org/reference/ 
    //dataTable.addColumn("year");
    //dataTable.addColumn("month");
    //dataTable.addColumn("day");
    dataTable.addColumn("hour");
    dataTable.addColumn("minute");
    dataTable.addColumn("second");
    
    //the following are dummy columns for each data value. Add as many columns as you have data values. Customize the names as needed. Make sure they are in the same order as the order that Arduino is sending them!
    dataTable.addColumn("sensor1");
    dataTable.addColumn("sensor2");
    dataTable.addColumn("Setpoint1");
    dataTable.addColumn("Min.Val 1");
    dataTable.addColumn("Setpoint2");
    dataTable.addColumn("Min.Val 2");      
    dataTable.addColumn("On/Off Status Sensor 1");
    dataTable.addColumn("On/Off Status Sensor 2");
    dataTable.addColumn("PID or On/off");
    dataTable.addColumn("PID value");
    dataTable.addColumn("External Temp");
    
/// Finish new code <-Javier Bruges  12.10.2016
    
}// end setup()



void draw() {
  // Kollar om musknappen är tryckt
  if (mousePressed){
      mousecheck();
  } // end if (mousePressed)
  
  // Kontrollerar om det finns data på serieporten
    if (port.available() > 0) { // If data is available,
      val = port.read(); // Läser seriedata och sparar det i variabeln val
      
//      float varToTable[] = new float[9];
      
      switch(val){
        
        // Tar emot T1-börvärder - grön linje
        case 255: //<>//
         // val2 = ((port.read()/100)+ 50)*1024/500; // read it and store it in val
         val2 = (port.read()* 0.2);
//         varToTable[2] = val2;
//          println(val,"=",val2);
          stroke(mygreen);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Tar emot T2-börvärder - blå linje
        case 254: //<>//
          //val2 = ((port.read()/100) + 50)*1024/500; // read it and store it in val
          val2 = (port.read()* 0.2);
//          varToTable[4] = val2;
//          println(val,"=",val2);
          stroke(myblue);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Tar emot T1 temperatur - röd linje
        case 253: //<>//
          //val2 = ((port.read()/100)+ 50)*1024/500; // read it and store it in val
          val2 = (port.read()* 0.2);
//          varToTable[0] = val2;
//          println(val,"=",val2);
          stroke(myred);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Tar emot T2 temperatur - blå linje
        case 252:  //<>//
         //val2 = ((port.read()/100)+ 50)*1024/500; // read it and store it in val
          val2 = (port.read()* 0.2);
//          varToTable[1] = val2;
//          println(val,"=",val2);
          stroke(mydarkblue);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Tar emot T1minvärder - grön linje
        case 251: //<>//
          //val2 = ((port.read()/100)+ 50)*1024/500; // read it and store it in val
          val2 = (port.read()* 0.2);
//          varToTable[3] = val2;
//          println(val,"=",val2);
          stroke(mygreen);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Tar emot elementstatus - röd/grön för av/på linje
        case 250: //<>//
          val1 = port.read(); // read it and store it in val
////          println(val1," ",valold);
////          println(val,"=",val1);
          if (val1==0) {
            if (valold != val1 && counting == 1){
                count2 = tid;
                counting = 0;
            }
            stroke(mygreen);
          }
          else{
            if (valold != val1){
                count1 = tid;
                counting = 1;
            }
            stroke(myred);
          }
//          varToTable[6] = val1;
          line (81+tid, 579,81+tid, 574);
          noStroke();
          if (count1 > 0 && count2 > 0 && counting == 0){
             count = count2 - count1; 
//////             text("Tid för energi:", 81+tid, 80);// Draw the Tid text
//////             text(count, 140+tid, 80);// Draw the Tid text
///                 text("Tid för energi:", 670, 30);// Draw the Tid text
///                 text(count, 775, 30);// Draw the Tid text 
             counting = 2;    
           }
           valold = val1;
        break;
        
        // Tar emot fläkthastigheten - blå linje
        case 249: //<>//
        if(fan==0){
          val1 = port.read(); // read it and store it in val
          println(val,"=",val1);
          stroke(myblue);
          line (81+tid, 41,81+tid, 41+val1*2);
          noStroke();
        }
        else{ // fan==1

        }
        break; 
        
        // Tar emot trigger för att öka tiden 
         case 248: //<>//
////         println(val);  
          tid++;
          if(tid > 789)
            tid=0;
        break;  
        
        // Fläktkontroll 
         case 247: //<>//
          val1 = port.read(); // read it and store it in val
//          varToTable[8] = val1;
          fan = val1;
////          println(val,"=",val1);
        break;
        
        // Fläkt AV/PÅ min delta värde
         case 246: //<>//
          val2 = (port.read()*0.2); // read it and store it in val
//          varToTable[5] = val2;
//          println(val,"=",val2);
          stroke(myblue);
          point (81+tid, 580-(val2*10));
          noStroke();
        break;
        
        // Fläkt AV eller PÅ 
         case 245:
          val1 = port.read(); // read it and store it in val
////          println(val,"=",val1," fan=",fan);
          if (val1==0 && fan == 1) {
            stroke(mygreen);
          }
          if (val1==1 && fan == 1) {
            stroke(myred);
          }
//          varToTable[7] = val1;
          line (81+tid, 41,81+tid, 49);
          noStroke();
        break;
        
         case 244:
////          println(val);       
           stroke(255); 
////           println("Linjer och tid =",tid);
             if (tid <2){           
               line (81+tid+1, 41, 81+tid+1, 579);
               line (81+tid+2, 41, 81+tid+2, 579);
               line (81+tid+3, 41, 81+tid+3, 579);
               line (81+tid+4, 41, 81+tid+4, 579);
               line (81+tid+5, 41, 81+tid+5, 579);
               line (81+tid+6, 41, 81+tid+6, 579);
               line (81+tid+7, 41, 81+tid+7, 579);
               line (81+tid+8, 41, 81+tid+8, 579);
               line (81+tid+9, 41, 81+tid+9, 579);
               line (81+tid+10, 41, 81+tid+10, 579);
               line (81+tid+11, 41, 81+tid+11, 579);
              } // end if (tid < 2)            
             if (tid >1 && tid < 789){
               line (81+tid+12, 41, 81+tid+12, 579);
              } // end if (tid < 535) 
           noStroke();
        break;                
        
        
//        default:
//          println("annat värde =",val1);
//        break;
      } // end switch(val) //<>//
      
    } // end if port.available()
  
 
  
  
} // end draw()

/************************************************************
* Nedan finns functions() som används av programmet
* 
* 
* 
* 
*************************************************************/

void mousecheck(){
  int dtime=200; // delay time to avoid mouse click bouncing
////   print("Musklick kontroll ");
////   println(mouseclick);
   mouseclick++;
// checking if the mouse is pressed inside the START button
    if (mousePressed && mouseX<60 && mouseX>20 && mouseY<80 && mouseY>40 ){

    }
// checking if the mouse is pressed inside the STOP button
//// if (mousePressed && mouseX<60 && mouseX>20 && mouseY<140 && mouseY>100 ){

//// delay(dtime);
//// }// end if

// checking if the mouse is pressed inside the SAVE button
//// if (mousePressed && mouseX<60 && mouseX>20 && mouseY<200 && mouseY>160 ){
////  saving = 1;
////  delay(dtime);
//// }// end if the mouse is pressed inside the SAVE button


// checking if the mouse is pressed inside the T1+ button
 if (mousePressed && mouseX<140 && mouseX>120 && mouseY<35 && mouseY>15 ){
  sendref(1);
  delay(dtime);
 }// end if the mouse is pressed inside the T1+ button
 
 // checking if the mouse is pressed inside the T1- button
 if (mousePressed && mouseX<170 && mouseX>150 && mouseY<35 && mouseY>15 ){
  sendref(2);
  delay(dtime);
 }// end if the mouse is pressed inside the T1- button
 
 // checking if the mouse is pressed inside the T1min+ button
 if (mousePressed && mouseX<260 && mouseX>240 && mouseY<35 && mouseY>15 ){
  sendref(3);
  delay(dtime);
 }// end if the mouse is pressed inside the T1min+ button
 
 // checking if the mouse is pressed inside the T1min- button
 if (mousePressed && mouseX<290 && mouseX>270 && mouseY<35 && mouseY>15 ){
  sendref(4);
  delay(dtime);
 }// end if the mouse is pressed inside the T1min- button

 // checking if the mouse is pressed inside the T2+ button
 if (mousePressed && mouseX<380 && mouseX>360 && mouseY<35 && mouseY>15 ){
  sendref(5);
  delay(dtime);
 }// end if the mouse is pressed inside the T2+ button
 
 // checking if the mouse is pressed inside the T2- button
 if (mousePressed && mouseX<410 && mouseX>390 && mouseY<35 && mouseY>15 ){
  sendref(6);
  delay(dtime);
 }// end if the mouse is pressed inside the T2- button

 // checking if the mouse is pressed inside the PID button
 if (mousePressed && mouseX<525 && mouseX>490 && mouseY<35 && mouseY>15 ){
  sendref(7);
  delay(dtime);
 }// end if the mouse is pressed inside the PID button
 
  // checking if the mouse is pressed inside the AV/PÅ button
 if (mousePressed && mouseX<585 && mouseX>535 && mouseY<35 && mouseY>15 ){
  sendref(8);
  delay(dtime);
 }// end if the mouse is pressed inside the AV/PÅ button

 // checking if the mouse is pressed inside the AV/PÅ+ button
 if (mousePressed && mouseX<615 && mouseX>595 && mouseY<35 && mouseY>15 ){
  sendref(9);
  delay(dtime);
 }// end if the mouse is pressed inside the AV/PÅ+ button
 
 // checking if the mouse is pressed inside the AV/PÅ- button
 if (mousePressed && mouseX<645 && mouseX>625 && mouseY<35 && mouseY>15 ){
  sendref(0);
  delay(dtime);
 }// end if the mouse is pressed inside the AV/PÅ- button


// check if 
////if (mousePressed && mouseX<20 && mouseX>0 && mouseY<20 && mouseY>0 ){
////  exit();
//// }

delay(dtime);

} // end mousecheck()
void serialEvent(Serial myPort){
  try {
    float varToTable[] = new float[11];
    valSave = myPort.readStringUntil('\n'); //The newline separator separates each Arduino loop. We will parse the data by each newline separator. 
    
    if (valSave!= null) { //We have a reading! Record it.
      //println(valSave);
      varToTable = float(split(valSave, ','));
      println(varToTable);
 
      TableRow newRow = dataTable.addRow(); //add a row for this new reading
      newRow.setInt("id", dataTable.lastRowIndex());//record a unique identifier (the row's index)
      
      //record time stamp
      //newRow.setInt("year", year());
      //newRow.setInt("month", month());
      //newRow.setInt("day", day());
      newRow.setInt("hour", hour());
      newRow.setInt("minute", minute());
      newRow.setInt("second", second());
      
      
      //record sensor information. Customize the names so they match your sensor column names. 
      newRow.setFloat("sensor1", varToTable[1]);
      newRow.setFloat("sensor2", varToTable[2]);
      newRow.setFloat("Setpoint1", varToTable[3]);
      newRow.setFloat("Min.Val 1", varToTable[4]);
      newRow.setFloat("Setpoint2", varToTable[5]);
      newRow.setFloat("Min.Val 2", varToTable[6]);      
      newRow.setFloat("On/Off Status Sensor 1", varToTable[7]);
      newRow.setFloat("On/Off Status Sensor 2", varToTable[8]);
      newRow.setFloat("PID or On/off", varToTable[9]);
      newRow.setFloat("PID value", varToTable[10]);
      newRow.setFloat("External Temp", varToTable[11]);
      
      readingCounter++; //optional, use if you'd like to write your file every numReadings reading cycles
      
      //saves the table as a csv in the same folder as the sketch every numReadings. 
      if (readingCounter % numReadings ==0)//The % is a modulus, a math operator that signifies remainder after division. The if statement checks if readingCounter is a multiple of numReadings (the remainder of readingCounter/numReadings is 0)
      {
        fileName = str(year()) + str(month()) + str(day()) + str(dataTable.lastRowIndex()); //this filename is of the form year+month+day+readingCounter
        saveTable(dataTable, fileName, "csv"); //Woo! save it to your computer. It is ready for all your spreadsheet dreams. 
      }
     }
  }
  catch(RuntimeException e) {
    e.printStackTrace();
  }
}

// procedure to send referens levels to the Arduino
void sendref(int r) {
////     print("Skickar data ");
////     println(r);
     port.write(r); 
// println(r);
 } // end sendref(int r)
