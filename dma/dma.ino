#undef HID_ENABLED

// Arduino Due ADC->DMA->USB 1MSPS
// by stimmer. Adaptador por Giovani Fonseca Ravagnani Disperati.
// Input: Analog in A0
// Output: Raw stream of uint16_t in range 0-4095 on Native USB Serial/ACM
// Documentation links - http://asf.atmel.com/docs/latest/samg/html/group__sam__drivers__pmc__group.html#gad09de55bb493f4ebdd92305f24f27d62
// 

//on linux, to stop the OS cooking your data:
//stty -F /dev/ttyACM0 raw -iexten -echo -echoe -echok -echoctl -echoke -onlcr

volatile int bufn,obufn;
uint16_t buf[4][256];                   // 4 buffers of 256 readings

void ADC_Handler(){                     // move DMA pointers to next buffer
  int f=ADC->ADC_ISR;                    // ADC_ISR = Interrupt Status Register
  if (f&(1<<27)){                        // 1<<27, shift 27 bits 
    bufn=(bufn+1)&3;                      // 
    ADC->ADC_RNPR=(uint32_t)buf[bufn];    // RNPR - Receive next pointer register
    ADC->ADC_RNCR=256;                    // RNCR - Receive next counter register
  }
}

void setup(){
  SerialUSB.begin(115200);
  while(!SerialUSB);
  //while (!SerialUSB.available());
  pmc_enable_periph_clk(ID_ADC);         // Enable all peripheral clocks
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  ADC->ADC_MR |=0x80;                    // ADC_MR = Adc Mode Register. 0x80 = Free running
  ADC->ADC_CHER=0x80;                    // ADC_CHER = Channel Enable Register. 0x80 = Pin0
  
  NVIC_EnableIRQ(ADC_IRQn);              // 
  ADC->ADC_IDR=~(1<<27);
  ADC->ADC_IER=1<<27;
  ADC->ADC_RPR=(uint32_t)buf[0];         // DMA buffer
  ADC->ADC_RCR=256;
  ADC->ADC_RNPR=(uint32_t)buf[1];        // next DMA buffer
  ADC->ADC_RNCR=256;                     // 
  bufn=1;
  obufn=0;
  ADC->ADC_PTCR=1;
  ADC->ADC_CR=2;
}

void loop(){
  while((obufn + 1)%4==bufn); // wait for buffer to be full
  SerialUSB.write((uint8_t *)buf[obufn],512); // send it - 512 bytes = 256 uint16_t
  Serial.print((long)buf[obufn]);
  //SerialUSB.println(buf[obufn]); // send it - 512 bytes = 256 uint16_t
  obufn=(obufn+1)&3;     
}
