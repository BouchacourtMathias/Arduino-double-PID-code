#define PIN_INPUT1 A0
#define PIN_INPUT2 A3
#include <string>
#include <sstream>
#include <stdlib.h>

int Setpoint1 = 504; // (1023/3.3V) * 1.62V = 504
int Setpoint2 = 508; // (1023/3.3V) * 1.64V = 508 

int input1, input2, Output1, Output2, Error1, Error2;
 
int Kp1=100, Ki1=100, Kd1=0; 
int Kp2=100, Ki2=100, Kd2=0;

int previous_error1 = 0, integral1 = 0, derivative1; 
int previous_error2 = 0, integral2 = 0, derivative2; 
int integral_count1 = 0, integral_count2 = 0;
int integral_count_reset_val1 = 150000, integral_count_reset_val2 = 350000;

int outputPin1 = 9;
int outputPin2 = 3;

int OutputValue1;
int OutputValue2;

bool ON1 = true; // run the loop (ON/OFF) parram
bool ON2 = true; 

String inistr;
String teststr;
String testval;
String num;

void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  pinMode(A0, INPUT);
  pinMode(A3, INPUT);

  digitalWrite(A0, HIGH);
  digitalWrite(A3, HIGH);
  
  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2,OUTPUT);
  
}

void loop() {
  
        /* COMMANDOS 
         * SHOW - displays input/output/OutputValue/Error/integral/integral_count for both diode
         * 
         * Specify the diode to be controlled with a number (1 for ECDL and 2 for slave) in front
         * of each command (e.g 1ON for turning on the ECDL temperature control) 
         * 
         * List of diode sensitive commands:
         * ON/OFF starts or puts and end to the script
         * 
         * SETP0000, SETI0000, SETD0000, SETS0000, SETC0000 
         * sets new value for P,I,D - parameter, the setpoint S 
         * or the integral_count_reset_val C, respectively, to 0000, the number entered (max 9999). 
         * In the case of SETC0000, the value 0000 is automatically multiplied by 1000 in the code.
         * (e.g 1SETP1000 sets the P-parrameter value of the ECDL to 1000).
         * 
         * PING - arduino answers PONG
         */
         
        if (Serial.available() > 0) {
          
          inistr = Serial.readString();
          num = inistr[0];
          teststr = inistr[1];
          teststr.concat(inistr[2]); 
                    
          if (teststr == "ON") { //Turn PID code On
             if (num == "1") {
             ON1 = true;
             Serial.println("script1 started");
             //integral_count1 = 0;
             //integral1 = 0;
             }
             else if (num == "2") {
             ON2 = true;
             Serial.println("script2 started");
             //integral_count2 = 0;
             //integral2 = 0;
             }
          }
          else if (num == "S") { 

              Serial.print(input1); Serial.print(" , "); 
              Serial.print(Output1); Serial.print(" , "); 
              Serial.print(OutputValue1); Serial.print(" , "); 
              Serial.print(Error1); Serial.print(" , "); 
              Serial.print(integral1); Serial.print(" , ");
              Serial.print(integral_count1); Serial.print(" / ");
              
              Serial.print(input2); Serial.print(" , "); 
              Serial.print(Output2); Serial.print(" , "); 
              Serial.print(OutputValue2); Serial.print(" , "); 
              Serial.print(Error2); Serial.print(" , "); 
              Serial.print(integral2); Serial.print(" , ");
              Serial.println(integral_count2);
          } 
          else if (teststr == "OF") { //Turn PID code Off
           
           if (num == "1") {
             ON1 = false;
             Serial.println("script1 terminated");
             //integral_count1 = 0;
             //integral1 = 0;
             }
             else if (num == "2") {
             ON2 = false;
             Serial.println("script2 terminated");
             //integral_count2 = 0;
             //integral2 = 0;
             }
          }
          else if (teststr == "SE") { 
          teststr = inistr[4];
          testval = inistr[5];
          testval.concat(inistr[6]);
          testval.concat(inistr[7]);
          testval.concat(inistr[8]);

              if (teststr == "P") {

                  if (num == "1") {
                    Kp1 = atoi(testval.c_str()); // string to int
                    Serial.print("P1 value changed to ");
                    Serial.println(Kp1);
                  }

                  else if (num == "2") {
                    Kp2 = atoi(testval.c_str()); // string to int
                    Serial.print("P2 value changed to ");
                    Serial.println(Kp2);
                  }
              }
                else if (teststr == "I") {

                  if (num == "1") {
                    Ki1 = atoi(testval.c_str()); // string to int
                    Serial.print("I1 value changed to ");
                    Serial.println(Ki1);
                  }
                  else if (num == "2") {
                    Ki2 = atoi(testval.c_str()); // string to int
                    Serial.print("I2 value changed to ");
                    Serial.println(Ki2);
                  }    
             }
                else if (teststr == "D") {

                  if (num == "1") {
                    Kd1 = atoi(testval.c_str()); // string to int
                    Serial.print("D1 value changed to ");
                    Serial.println(Kd1);
                  }
                  else if (num == "2") {
                    Kd2 = atoi(testval.c_str()); // string to int
                    Serial.print("D2 value changed to ");
                    Serial.println(Kd2);
                  }
                }
                else if (teststr == "C") {

                  if (num == "1") {
                    integral_count_reset_val1 = atoi(testval.c_str())*1000; 
                    Serial.print("integral_count_reset_val1 changed to ");
                    Serial.println(integral_count_reset_val1);
                  }
                  else if (num == "2")  {
                    integral_count_reset_val2 = atoi(testval.c_str())*1000;
                    Serial.print("integral_count_reset_val2 changed to ");
                    Serial.println(integral_count_reset_val2);
                  }
                }
                  
                else if (teststr == "S"){ 
                 
                  if (num == "1") {
                    Setpoint1 = atoi(testval.c_str());
                    Serial.print("Setpoint1 value changed to ");
                    Serial.println(Setpoint1);
                  }
                  else if (num == "2") {
                    Setpoint2 = atoi(testval.c_str());
                    Serial.print("Setpoint2 value changed to ");
                    Serial.println(Setpoint2);
                  }
                  
                }
                else {
                  Serial.println("Did not recognize command");
                }
          }
          else if (teststr == "PI") { // PING - answers PONG 
            if (num == "1") {
              Serial.println("PONG1");
            }
            else if (num == "2") {
              Serial.println("PONG2");
            }
                 
          }
          else {
            Serial.println("did not recognize command");  
          }
        }


if (ON1 || ON2) { // ON1 || ON2
Update_PID();
}
else {
  analogWrite(outputPin2,255);
  analogWrite(outputPin1,255);
}
}

void Update_PID() {
  // Convert the analog reading (which goes from 0 - 1023) 10bit to a voltage (0 - 3.3V):
  //float input = analogRead(PIN_INPUT) * (3.3 / 1023.0) ; //convert analog input read  
  input1 = analogRead(PIN_INPUT1); // input in bits
  //delay(5); //10ms
  //input1 = analogRead(PIN_INPUT1);
  
  input2 = analogRead(PIN_INPUT2); // input in bits
  //delay(5);
  //input2 = analogRead(PIN_INPUT2);

//****************** PID CODE 1 ECDL ********************************//
  Error1 = (Setpoint1 - input1); // Dividing the error reduces the resolution
  
  if (integral_count1 > integral_count_reset_val1) {
  integral1 = integral1 + Error1;
  integral_count1 = 0;
  }
  derivative1 = (Error1 - previous_error1);
  Output1 = (Kp1 * Error1)/100 + (Ki1 * integral1)/100 + Kd1 * derivative1;


// ******************** PID CODE 2 SLAVE ****************************//
  Error2 = (Setpoint2 - input2);
  
  if (integral_count2 > integral_count_reset_val2) {
  integral2 = integral2 + Error2;
  integral_count2 = 0;
  }
  derivative2 = (Error2 - previous_error2);
  Output2 = (Kp2 * Error2)/100 + (Ki2 * integral2)/100 + Kd2 * derivative2;

// *************** SETTING OUTPUTVALUES FOR ARDUINO *************************//
// integral_count doesn't count if OutputValue = 0
// ***** ECDL 
  if (ON1) {
     if (Output1 <= 0) { // input Voltage too high, need to "cool down"
        OutputValue1 = 255;
        integral_count1++;
      }
     else if (Output1 >= 255) { // input Voltage too low, maxing out heating
        OutputValue1 = 0;
      }
     else {  
        OutputValue1 = 255 - Output1;
        integral_count1++; 
      }
  }

// ****** SLAVE
  if (ON2) {
    if (Output2 <= 0) { // input Voltage too high, need to "cool down"
      OutputValue2 = 255;
      integral_count2++;
    }
    else if (Output2 >= 255) { // input Voltage too low, maxing out heating
      OutputValue2 = 0;
    }
    else {  
      OutputValue2 = 255 - Output2;
      integral_count2++;
    }
  }

// **************** SAFEGUARD IN CASE OF HIGH INPUT VALUE ********************//
  if (input1 >= 1000) { // Safety precaution
    ON1 = false;
    OutputValue1 = 255;
  }

  if (input2 >= 1000) {
    ON2 = false;
    OutputValue2 = 255;
  }

// *************** WRITING TO ARDUINO AND UPDATING VALUES ********************//
  if (ON1) {
    analogWrite(outputPin1, OutputValue1); 
  }
  else {
    analogWrite(outputPin1, 255);
  }
  
  if (ON2) {
    analogWrite(outputPin2, OutputValue2);
  }
  else {
    analogWrite(outputPin2, 255);
  }

  previous_error1 = Error1;
  previous_error2 = Error2;

}
