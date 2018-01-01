
/*  Project: Measure the 8 Channel EEG and Receive the Event, and then Synchronize Data and Event to Output by Bluetooth
    Author: Kai-Chiang Chuang
    Version: 1.0.2
    Date: 2017/07/19 
 */
/* Include SPI and RF24 header */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(7, 8); // CNS, CE
const byte address[6] = "00003"; // Slave - 1
/* Define the value for setting  */
#define Setting_EEG_Channel 8
#define ADC_Prescaler 16 // conversion=13us/sample
#define Sampling_Rate 500 // unit:Hz
/* Define the Serial Baud Rate and Packet Head*/
#define Serial_Baud_Rate 115200
#define Packet_Head 2
/* Define the Event Code*/
#define Event_Code_init 0
#define Trigger_Code 1
#define Low_Pitch_Code 2
#define High_Pitch_Code 3
#define Break_Code 4
#define Button_Code 5
/* Define RF channel*/
#define RF_channel 108 
/* Declare the Global Variable */
uint8_t index = 0;
uint8_t RX_buffer = 0;
uint8_t Data_Val[2];
uint8_t Data_Val_Size = sizeof(Data_Val)/sizeof(Data_Val[0]);
uint16_t Clock_Count = 0;

/*Declare the Globle Variable for ISR */
volatile boolean flag = false; // initial flag value is fales
volatile uint8_t ADC_Pin_Count = 0;
volatile uint16_t count = 0;
volatile uint16_t Time_clock = 0;

/*Optimize bluetooth packet */
volatile uint16_t Packet[6] = {0};
volatile uint8_t Packet_len = sizeof(Packet)/sizeof(Packet[0]);
volatile uint16_t ADC_val[Setting_EEG_Channel] = {0};
volatile uint8_t ADC_len = sizeof(ADC_val)/sizeof(ADC_val[0]);

void Serial_init()
{
  Serial.begin(Serial_Baud_Rate); // Setting the baud rate
}

void RF_init()
{ 
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(RF_channel);
  radio.startListening();
}

void ADC_Prescaler_init() //ADC Prescaler = 16  // 100 (Register bit)
{  
  switch(ADC_Prescaler) // Change ADC Converter Speed
  {
    case 128:
      ADCSRA |=_BV(ADPS2);   // 1
      ADCSRA |=_BV(ADPS1);   // 1
      ADCSRA |=_BV(ADPS0);   // 1
      break;
    case 64:
      ADCSRA |=_BV(ADPS2);   // 1
      ADCSRA |=_BV(ADPS1);   // 1
      ADCSRA &=~_BV(ADPS0);  // 0
      break;
    case 32:
      ADCSRA |=_BV(ADPS2);   // 1
      ADCSRA &=~_BV(ADPS1);  // 0
      ADCSRA |=_BV(ADPS0);   // 0
      break;
    case 16:
      ADCSRA |=_BV(ADPS2);   // 1
      ADCSRA &=~_BV(ADPS1);  // 0
      ADCSRA &=~_BV(ADPS0);  // 0
      break;
  }
}
  
void Timer_init() //Timer1 Setting 
{
    TCCR1A = 0x00;               //Set timer mode
    TCCR1B |=_BV(CS12);          //TCCR1B set the timer prescale: CPU clock/256
    TCCR1B &=~_BV(CS11);       
    TCCR1B &=~_BV(CS10);  
    TCNT1 = 0;  // reset timer register
    Clock_Count = 65536-(1.0/Sampling_Rate*1000*1000/16);  //Timer Resolution: 16us
}

void Send_Data() //load value to packet and send data
{     
  for(index = 0;index < Packet_len;index++) // 28us
  {
    switch(index)
    {
      case 0: Packet[index] = (Packet_Head<<14)|(Time_clock<<5)|RX_buffer; break;
      case 1: Packet[index] = (ADC_val[0]<<6)|(ADC_val[1]>>4); break;
      case 2: Packet[index] = (ADC_val[1]&0x0F)<<12|(ADC_val[2]<<2)|(ADC_val[3]>>8); break;
      case 3: Packet[index] = (ADC_val[3]&0xFF)<<8|(ADC_val[4]>>2); break;
      case 4: Packet[index] = (ADC_val[4]&0x03)<<14|ADC_val[5]<<4|ADC_val[6]>>6; break;
      case 5: Packet[index] = (ADC_val[6]&0x3F)<<10|ADC_val[7]; break;
    } 
  }
  for(index=0;index<Packet_len;index++) // 1136 us (原本 1700us)
  {
     Data_Val[0] = Packet[index]>>0x08; //High 8 bits
     Data_Val[1] = Packet[index]&0xff;  //Low 8 bits	 
     Serial.write(Data_Val,Data_Val_Size); //Send 2 byte
  }
}

uint8_t Serial_Buffer_Read()
{
 if(radio.available()>0) // Read event by RF module
  {
    radio.read(&RX_buffer, sizeof(RX_buffer));
    radio.flush_rx();
    return RX_buffer; // return the RX_buffer value
  }
 else return 0; 
}

void Waitting_Trigger()
{
  while(1) //infinite loop
  {   
   // digitalWrite(2, LOW); 
    while(!radio.available()); //Waiting for comming string of trigger
    RX_buffer=Serial_Buffer_Read(); // read the event from serial buffer into RX_buffer value
    if(RX_buffer == Trigger_Code) //Trigger main function code
    { 
  //    digitalWrite(2, HIGH); 
      TIMSK1 |= _BV(TOIE1);    // enable timer overflow interrupt
      TCNT1 = Clock_Count;     // load clock count value into timer register          
      break; // break while loop
    }    
  }
}

void Debug_RF()
{
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW); 
  while(1)
  {
    while(!radio.available()); //Waiting for comming string of trigger
    RX_buffer = Serial_Buffer_Read(); // read the event from serial buffer into RX_buffer value
    if(RX_buffer == Trigger_Code) digitalWrite(2, HIGH);
    else digitalWrite(2, LOW);
  }
}
void setup() 
{
  Serial_init(); // inital serial port
  RF_init();
  Timer_init(); // initial timer
  ADC_Prescaler_init(); // initial the ADC prescaler

  //Debug_RF(); // debug RF action

  Waitting_Trigger(); // waitting for comming trigger
}
 
void loop() 
{ 
  if(flag) // initial flag is false, after complete AD converter, then flag is true
  {
    RX_buffer = Serial_Buffer_Read(); // read the buffer   
    Send_Data(); // send data
    if(RX_buffer == Break_Code){
      TIMSK1 &=~ _BV(TOIE1); // disable timer overfloat interrupt
      flag = false; // reset value of flag 
      Waitting_Trigger(); //return for 
     }
      flag = !flag; // reverse value of flag
  }// end if
}

ISR (TIMER1_OVF_vect) // Call the interrupt service routine when the timer1 over flow...
{
 Time_clock++; // Increase timming
 if(Time_clock > Sampling_Rate) Time_clock = 1; //Reset Clock Value
 TCNT1 = Clock_Count;  // load clock count value into timer register   
 while(ADC_Pin_Count < ADC_len) // Reading the ADC value in while loop
  { 
   ADC_val[ADC_Pin_Count] = analogRead(ADC_Pin_Count); //Read ADC Value
   ADC_Pin_Count++;  //Increase ADC Mux Pin
  } 
   ADC_Pin_Count = 0; // Reset  
   flag = !flag; // reverse value of flag
}


