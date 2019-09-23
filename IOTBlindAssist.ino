#include "SoftwareSerial.h"
#include "TinyGPS++.h"
#include <SPI.h>
#include <Ethernet.h>
#define BLYNK_PRINT Serial
#include <BlynkSimpleSerialBLE.h>

SoftwareSerial SerialBLE(2, 3); //BLUETOOTH
SoftwareSerial serial_connection(10, 11); //GPS
char auth[] = "";//AUTH code from Blynk App
unsigned int index=1;
float latitude;
float longitude;
int c=0;
WidgetMap myMap(V0); 

int m=5;//motor1
int trigPin=6;//us1
int echoPin=7;
int m1=9;//motor2
int trigPin1=12;//us2
int echoPin1=13;

int sensor_pin = 0;//heartrateAnalog                   

volatile int heart_rate;          

volatile int analog_data;              

volatile int time_between_beats = 600;            

volatile boolean pulse_signal = false;    

volatile int beat[10];  //heartbeat values will be stored in this array    

volatile int peak_value = 512;          

volatile int trough_value = 512;        

volatile int thresh = 525;              

volatile int amplitude = 100;                 

volatile boolean first_heartpulse = true;      

volatile boolean second_heartpulse = false;    

volatile unsigned long samplecounter = 0;   //This counter will tell us the pulse timing

volatile unsigned long lastBeatTime = 0;




void setup() 
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(m,OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(m1,OUTPUT);
  serial_connection.begin(9600);
  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);
  interruptSetup();   
}

void loop() 
{
  ultra();
  delay(50);
  
   serial_connection.listen();
  while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  { getInfo();
    c++;}
    
if(c>0)//gps signal aquired
   {SerialBLE.listen();
    Blynk.run();
   displayInfo();}
}



void ultra() //ULTRASONIC SENSOR
{
long duration, distance;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  Serial.println(distance);

  if (distance <= 50 && distance > 0){
    analogWrite(m,255);
  }
  else if (distance <=75 && distance >=50){
    analogWrite(m,155);
  }
   else if (distance <=100 && distance >=75){
    analogWrite(m,100);
  }
  else if(distance==0||distance>50)
    {analogWrite(m,0);}
  
  delay(50);


  long duration1, distance1;
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;
  Serial.println(distance1);

  if (distance1 <= 50 && distance1 > 0){
    analogWrite(m1,255);
  }
  else if (distance <=75 && distance1 >=50){
    analogWrite(m1,155);
  }
   else if (distance1 <=100 && distance1 >=75){
    analogWrite(m1,100);
  }
  else if(distance1==0||distance1>50)
    {analogWrite(m1,0);}

}

    void displayInfo()//on phone
  { Blynk.virtualWrite(V1, String(latitude,6));   
    Blynk.virtualWrite(V2, String(longitude,6));
    Blynk.virtualWrite(V3, heart_rate);
    myMap.location(index, latitude, longitude, "GPS_Location");
  }



  void interruptSetup()

{    

  TCCR2A = 0x02;  // This will disable the PWM on pin 3 and 11

  OCR2A = 0X7C;   // This will set the top of count to 124 for the 500Hz sample rate

  TCCR2B = 0x06;  // DON'T FORCE COMPARE, 256 PRESCALER

  TIMSK2 = 0x02;  // This will enable interrupt on match between OCR2A and Timer

  sei();          // This will make sure that the global interrupts are enable

}


ISR(TIMER2_COMPA_vect)

{ 

  cli();                                     

  analog_data = analogRead(sensor_pin);            

  samplecounter += 2;                        

  int N = samplecounter - lastBeatTime;      


  if(analog_data < thresh && N > (time_between_beats/5)*3)

    {     

      if (analog_data < trough_value)

      {                       

        trough_value = analog_data;

      }

    }


  if(analog_data > thresh && analog_data > peak_value)

    {        

      peak_value = analog_data;

    }                          



   if (N > 250)

  {                            

    if ( (analog_data > thresh) && (pulse_signal == false) && (N > (time_between_beats/5)*3) )

      {       

        pulse_signal = true;

        time_between_beats = samplecounter - lastBeatTime;

        lastBeatTime = samplecounter;     



       if(second_heartpulse)

        {                        

          second_heartpulse = false;   

          for(int i=0; i<=9; i++)    

          {            

            beat[i] = time_between_beats; //Filling the array with the heart beat values                    

          }

        }


        if(first_heartpulse)

        {                        

          first_heartpulse = false;

          second_heartpulse = true;

          sei();            

          return;           

        }  


      word runningTotal = 0;  


      for(int i=0; i<=8; i++)

        {               

          beat[i] = beat[i+1];

          runningTotal += beat[i];

        }


      beat[9] = time_between_beats;             

      runningTotal += beat[9];   

      runningTotal /= 10;        

      heart_rate = 60000/runningTotal;

    }                      

  }




  if (analog_data < thresh && pulse_signal == true)

    {   

      pulse_signal = false;             

      amplitude = peak_value - trough_value;

      thresh = amplitude/2 + trough_value; 

      peak_value = thresh;           

      trough_value = thresh;

    }


  if (N > 2500)

    {                          

      thresh = 512;                     

      peak_value = 512;                 

      trough_value = 512;               

      lastBeatTime = samplecounter;     

      first_heartpulse = true;                 

      second_heartpulse = false;               

    }


  sei();                                

}

