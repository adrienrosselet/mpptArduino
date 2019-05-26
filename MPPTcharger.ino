#include <SoftwareSerial.h>
//#include <avr/io.h>
#define PINSWITCHING 4 
#define PINCURRENT 3
#define PINTENSION 2

unsigned long counter=0;
unsigned long counter2=0;
unsigned long counter3=0;
unsigned long current=0;
unsigned long nbr=0;
unsigned long power=0;
unsigned long oldPower=0;
unsigned long tension=0;
unsigned long temps=0;
SoftwareSerial mySerial(2, 1);

void setup() {
  // initialize digital pin 13 as an output.
  //pinMode(PINSWITCHING, OUTPUT);
  DDRB |= (1 << PINSWITCHING);    //pin is an output
  PORTB &= ~(1 << PINSWITCHING);  //no pull up resitor on pin
  DDRB &= ~(1 << PINCURRENT);    //pin is an input
  PORTB &= ~(1 << PINCURRENT);   //no pull up resitor on pin
  PORTB &= ~(1 << PINTENSION);   //no pull up resitor on pin
  ADMUX = 0b00000011; //input channel: PB3, reference voltage: Vcc
  ADCSRA = 0b10000111;//adcsra:aden,adsc,adate,adif,adie,adps2,adps1,adps0 I start the conversion
  
  //TCCR0A = 0b00100011;//clear OCOB on match, fast pwm
  //TCCR0B = 0b00000001;//select no prescaler for pwm clock
  //OCR0B = 0b01111101;//pwm duty cycle
  
  //TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  //TCCR0B = 0<<WGM02 | 1<<CS00; 
  TCCR1 = 0<<PWM1A | 0<<COM1A0 | 0<<COM1A1 | 1<<CS10;
  GTCCR = 1<<PWM1B | 2<<COM1B0;//enable pwm

  
  
  mySerial.begin(9600);
  temps=millis();
  counter=temps;
  counter2=temps;
  counter3=temps;
}

uint8_t duty = 0;
bool sign = 0;
void loop() {
  
  current+=currentMeasurement();
  tension+=tensionMeasurement();
  nbr++;

  temps=millis();
  if((temps-counter2) > 600){
    counter2=temps;
    current=current/nbr;
    tension=tension/nbr;
    power=current*tension;
    if(oldPower > power){
      sign = !sign;
    }
    
    if(sign){
      duty=duty-5;
      if(duty < 50){
        duty = 50;
      }
      
    }
    else{
      duty=duty+5;
      if(duty > 200){
        duty = 200;
      }
    }
    oldPower = power;
    OCR1B  = duty;
    mySerial.println(duty);
    current=0;
    tension=0;
    nbr=0;
  }
//  if((temps-counter3) > 1000){
//    counter3=temps;
//    if(sign){
//      duty=duty+20;
//      if(duty > 200){
//        sign = 0;
//      }
//    }
//    else{
//      duty=duty-20;
//      if(duty < 50){
//        sign = 1;
//      }
//    }
//    
//    OCR1B  = duty;
//  }

//  if((temps-counter) > 50){
//    //GTCCR = 0<<PWM1B;
//    counter=temps;
//    //current=analogRead(PINCURRENT);
//    //current=currentMeasurement();
//    //tension=tensionMeasurement();
//    //GTCCR = 1<<PWM1B;
//    //mySerial.print(tension);
//    //mySerial.print(" ");
//    
//    mySerial.print(duty);
//    mySerial.print(" ");
//    mySerial.println(power);
//    
//    //current=0;
//    //tension=0;
//    
//    //OCR1B  = duty - 1;
//    
//  }
}

int currentMeasurement(){
  unsigned int result=0;
  unsigned int adcl=0;
  unsigned int adch=0;
  ADMUX = 0b10010011;//reference: internal 2.56V, input: PB3
  //ADMUX = 0b00000011;//reference: Vcc, input: PB3
  ADCSRA |= (1 << ADSC);   //start conversion
  while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
  adcl=0x00FF & ADCL;//adcl has to be read first and then adch
  adch=0x0003 & ADCH;
  result = (adch << 8)|(adcl);   //store data in analogData variable
  return result;
}
int tensionMeasurement(){
  unsigned int result=0;
  unsigned int adcl=0;
  unsigned int adch=0;
  ADMUX = 0b00000001;//reference: Vcc, input: PB2
  ADCSRA |= (1 << ADSC);   //start conversion
  while (ADCSRA & (1 << ADSC) ); // wait till conversion complete
  adcl=0x00FF & ADCL;//adcl has to be read first and then adch
  adch=0x0003 & ADCH;
  result = (adch << 8)|(adcl);   //store data in analogData variable
  return result;
}
