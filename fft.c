#include "mbed.h"
#include <stdio.h>
#include <ctype.h>
#include <arm_math.h>

//Serial pc(USBTX, USBRX);
DigitalOut led(PTB19);
DigitalOut led2(PTB18);
DigitalOut led3(PTD4);
DigitalOut CHIPEN(PTC11);
DigitalOut LR(PTC10);
//DigitalOut flag(PTE0);

const int SAMPLE_RATE_HZ = 9000;
const int FFT_SIZE = 128;
volatile int32_t BUFFER_A[FFT_SIZE*2];
volatile int32_t BUFFER_B[FFT_SIZE*2];
volatile int32_t *DMA_BUFF;
volatile int32_t *READ_BUFF;
float samples[FFT_SIZE*2];    
float magnitudes[FFT_SIZE*2];  
volatile bool SAMPLING_DONE = 0;
volatile bool USING_BUFFER_A = 1;

void pr_16(volatile uint16_t *reg);
void pr(volatile uint32_t *reg);
void START_I2S();
void START_DMA();
void RX_CALL();
void checkRequest(); 
void sendData(int dataType);
extern "C" void DMA0_IRQHandler();

int main(){
    pc.baud(230400);
    START_I2S();
    START_DMA();
    led = 0;
    led2 = 0;
    led3 = 0;
    int peaks[FFT_SIZE];
    int found = 0;
    //flag = 0; 
    while(1){
        if(SAMPLING_DONE){ 
            //flag = 1;  
            for (int f = 0; f < FFT_SIZE; f++){
                READ_BUFF[f] = (READ_BUFF[f] >> 16) ; 
            }
                   
            for (int j = 0; j < FFT_SIZE; j++){
                samples[j] = (float) READ_BUFF[j] ; //100000
            }
 
            arm_rfft_fast_instance_f32 fft_inst; 
            arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
            arm_rfft_fast_f32(&fft_inst, samples, magnitudes, 0);            
            arm_cmplx_mag_squared_f32(magnitudes, magnitudes, FFT_SIZE); 
            
            float sum = 0.0;
            float mean = 0.0;
            for (int x = 1; x < (FFT_SIZE/2); x++){
                sum = sum + magnitudes[x];
            }
            mean = sum/((FFT_SIZE/2) - 1);
            
            int in = 0;
            for (int x = 15; x < 55; x++){
                if (magnitudes[x] > 30*mean){//52
                    peaks[in] = x;
                    in++;
                }
            }
            
            if ( in > 0 && in < 4){
                found++;
            } else{
                found = 0;
            }
            
            if (found == 12){
                found = 0;
                if (led == 0){
                    led = 1;
                    led2 = 1;
                    led3 = 1;
                    wait(0.4);
                }else{
                    led = 0;
                    led2 = 0;
                    led3 = 0;
                    wait(0.4);
                }     
            }
            //found = 0;
            SAMPLING_DONE = 0;    
            //checkRequest();
            //flag = 0;  
        }      
    }     
}

//I2S Init
void START_I2S(){

    CHIPEN = 1;
    LR = 0;

    //Pins Init
    SIM_SCGC5  |= 0x42B82;          //Enables PORTC
    PORTC_PCR5 |= 0x400;            //PTC5 --> I2S0_RXD0 -> SD
    PORTC_PCR6 |= 0x400;            //PTC6 --> I2S0_RC_BCLK -> SCK
    PORTC_PCR7 |= 0x400;            //PTC7 --> I2S0_RX_FS -> WS

    //Clock Init
    SIM_SCGC6 |= 0x8000;            //Turn on I2S Clock Gate
    SIM_SOPT2 |= 0x30000;           //I2S Clocking Sourced from 48MHz IRC48MCLK
    I2S0_MCR  |= 0x43000000;        //Output Enable. Mux IRC48MCLK into Fractional Divider
    I2S0_RCR2 |= 0x700000D;         //MCLK MSEL to BCLK. Div -> 13, 9k Sample
    I2S0_MDR  |= 0x1F07C;           //Set I2S Master Clock to 12.2880MHz
    SIM_SCGC6 |= 0x8000;            //Re-Enable I2S Module

    //Receive Init
    I2S0_RCSR  = 0x0;               //Disable Receive as per Datasheet
    I2S0_RCR1 |= 0x1;               //Sets FIFO Watermark to 1
    I2S0_RCR3 |= 0x10000;           //Enable Receive Data Channel
    I2S0_RCR4 |= 0x10011F1B;        //Internal FS. Early FS. 32 SYWD. FRSZ 2. FCONT
    I2S0_RCR5 |= 0x1F1F1F00;        //32 Bits per Word. 32 Bits in First Word. Shifted 32.
    I2S0_RMR  |= 0x2;               //Mask 2nd Word (One Channel Only)
    
    I2S0_RCSR |= 0x80000001;        //Enable & Request DMA   
}

//DMA Init
void START_DMA(){
    
    SIM_SCGC6 |= 0x2;               //DMAMUX Module Turn On
    SIM_SCGC7 |= 0x2;               //DMA Module Turn On
    DMA_TCD0_CSR = 0;               //Rest Settings
    DMAMUX_CHCFG0 = 0x8C;           //I2S Source: Slot 12 & Activate 
    DMA_TCD0_CSR &= 0xBF;           //Inactive    
    DMA_TCD0_CSR |= 0x2;            //Interrupt Major Iteration 
    DMA_CR = 0x0;                   //Disable
    
    NVIC_EnableIRQ(DMA0_IRQn);      //Enable IRQ DMA Channel 0 
        
    DMA_TCD0_SADDR = (uint32_t)&I2S0_RDR0;     //Data Source 
    DMA_TCD0_DADDR = (uint32_t)BUFFER_A;       //Destination
    
    DMA_TCD0_SOFF = 0;                          //No Source Offset
    DMA_TCD0_SLAST = 0;                         //Nothing Added to Source Address after Major Loop 
    DMA_TCD0_DLASTSGA = 0;                      //Value Added to Destination Address after Major Loop
    DMA_TCD0_DOFF = 4;                          //4 Byte Destination Offset 
    DMA_TCD0_NBYTES_MLNO = 4;                   //4 Bytes Transfered in Minor Loop
    DMA_TCD0_BITER_ELINKNO = FFT_SIZE;          //Number of Bins   
    DMA_TCD0_CITER_ELINKNO = FFT_SIZE;          //Number of Bins
    DMA_TCD0_ATTR = 0x202;                      //32-Bit Transfer Size
    
    DMA_SERQ = 0x0;                             //Channel 0 Enable
}

//RX Interrupt Callback
void RX_CALL(){
   
    if(USING_BUFFER_A){
       USING_BUFFER_A = 0;
       DMA_BUFF = BUFFER_B;
       READ_BUFF = BUFFER_A;
    }
    else{
        USING_BUFFER_A = 1;
        DMA_BUFF = BUFFER_A;
        READ_BUFF = BUFFER_B;     
    }  
    DMA_TCD0_DADDR = (uint32_t) DMA_BUFF;
}

//DMA 0 Interrupt Handler
extern "C" void DMA0_IRQHandler(){
    DMA_CINT = 0;
    SAMPLING_DONE = 1;
    RX_CALL();
    return; 
}

//Debugging tool, it prints register [address]--> [contents]
//Pass the "address of" a register, i.e: pr(&PORTA_PCR1)
void pr_16(volatile uint16_t *reg) {
    //pc.printf("%x --> %u \r\n", reg, *reg);
}
//32-Bit
void pr(volatile uint32_t *reg) {
    //pc.printf("%x --> %x \r\n", reg, *reg);
}

/* MATLAB Serial Com. 
 * 1. Sends Magnitudes
 * 2. Sends Samples
 * 3. Sends FFT Size
 * 4. Sends Sample Rate 
 */
void sendData(int dataType) {
  if (dataType == 1) {
    //for (int i = 1; i < (FFT_SIZE/2) ; ++i) {
        //pc.printf("%f\n", magnitudes[i]);
    //}
  }
  else if (dataType == 2) {
    //for (int i = 0; i < FFT_SIZE; ++i) { 
        //pc.printf("%f\n", samples[i]);
    //}
  }
  else if (dataType == 3) {
      //pc.printf("%d\n", FFT_SIZE);
  }
  else if (dataType == 4) {
      //pc.printf("%d\n", SAMPLE_RATE_HZ);
  }
}

//Checks If MATLAB needs Data
void checkRequest(){
    int number;
    pc.scanf("%d", &number);     
    sendData(number);
}