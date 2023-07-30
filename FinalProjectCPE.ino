
// Andy Chiang & Tiffany Noda
// Final Project
// CPE 301

//Libraries
#include "Adafruit_Sensor.h"
#include "DHT.h"
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>

//definitions
#define DHTPIN 44
#define DHTTYPE DHT11
#define STEPS 32
#define RDA 0x80
#define TBE 0x20  

DHT dht(DHTPIN, DHTTYPE);

//STEPPER
Stepper stepper(STEPS, 37, 41, 39, 43);
int previousPos = 0;

//LCD
const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB =    (unsigned char *) 0x25;

//Define Port A Register Pointers for LEDs 
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21; 
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//Define Port F Register Pointers for Analog
volatile unsigned char* port_f = (unsigned char*) 0x31; 
volatile unsigned char* ddr_f  = (unsigned char*) 0x30; 
volatile unsigned char* pin_f  = (unsigned char*) 0x2F;

//RTC module
RTC_DS1307 rtc;

//temp range
float tempRange = 76.30;

//interupt buttons
const byte stopButtonPin = 18;
const byte resetButtonPin = 19;

//global ticks counter
int timer_running = 0;


void setup() {

  setup_timer_regs();
  U0init(9600);
  stepper.setSpeed(200);
  lcd.begin(16,2);
  dht.begin();
  adc_init();

  //Interrupts
  //set PB4 to output
  *portDDRB |= 0b11110000;
  //set PB4 LOW
  *portB &= 0b11101111;
  //set analog ports to output
  *ddr_f |= 0b00000001;

  //CLOCK
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay_s(10);
  }
}


void loop(){

  //water sensor values and current temp
  int waterLevel = adc_read(5);
  float currentTemp = dht.readTemperature(true);

  if(waterLevel < 100){
    reportTransition();
    error_state();
  }
  else{
    //compares temp range
      if(currentTemp <= tempRange){
          reportTransition();
          idle_state();
          stepperMotor();
      }
      else{
          reportTransition();
          running_state();
          stepperMotor();
      }
  }
}

//LCD monitor
void printTempHumidity(){

  float temp = dht.readTemperature(true);
  float humi = dht.readHumidity();

  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print((char) 223);
  lcd.print("F");

  lcd.setCursor(0,1); 
  lcd.print("Humidity: ");
  lcd.print(humi);
  lcd.print("%");
}


//LED STATES
void running_state(){
  //blue LED on
  *port_a |= 0b01000010;
  *port_a &= 0b01000010;
  *portB |= 0b10100000;
  printTempHumidity();
}

void idle_state(){
  //green LED on
  *port_a |= 0b10000000;
  *port_a &= 0b10000000;
  *portB &= 0b01011111;
}

void disabled_state(){
  //yellow LED on
  *port_a |= 0b00100000;
  *port_a &= 0b00100000;
  *portB &= 0b01011111;
}

void error_state(){
  //red LED on
  *port_a |= 0b00001000;
  *port_a &= 0b00001000;
  *portB &= 0b01011111;
  //LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error: ");
  lcd.setCursor(0,1);
  lcd.print("Water level low");
}



void stepperMotor(){
  //Stepper motor
  int currentPos = adc_read(0); 
  reportTransition();
  stepper.step(currentPos - previousPos);
  int previousPos = currentPos;
}

//RTC to serial monitor
void reportTransition(){
  DateTime now = rtc.now();
  char timeStr[9];// 8 characters + null terminator
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  for(int i = 0; i < 9; i++){
    U0putChar(timeStr[i]);
  }
  U0putChar('\n');
}

//check water level
unsigned int water_level() {
  int waterLevel = adc_read(5);
  return waterLevel;
}

//Timer setup function
void setup_timer_regs(){
  //setup the timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;

  //reset the TOV flag
  *myTIFR1 |= 0x01;
  //enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}

//TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect){
//Stop the Timer
  *myTCCR1B &= 0xF8;
  *myTCCR1B |= 0b00000001;
}

//delay
void delay_s(unsigned int freq){
  double period = 1.0/double(freq);
  double half_period = period/ 2.0f;
  double clk_period = 0.0000000625;
  unsigned int ticks = half_period / clk_period;
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  * myTCCR1B |= 0b00000001;
  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;        
  *myTIFR1 |= 0x01;
}

void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; 
  *my_ADCSRA &= 0b11011111; 
  *my_ADCSRA &= 0b11110111; 
  *my_ADCSRA &= 0b11111000;
  // setup the B register
  *my_ADCSRB &= 0b11110111; 
  *my_ADCSRB &= 0b11111000; 
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111; 
  *my_ADMUX  &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel){
  // (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel > 7){
    // remove most significant bit (bit 3)
    adc_channel -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

//UART FUNCTIONS
void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

unsigned char U0getChar(){
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata){
  while(!(TBE & *myUCSR0A));
  *myUDR0 = U0pdata;
}