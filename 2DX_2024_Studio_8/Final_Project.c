#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
#include <math.h>
 
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
											 
#define motor_index 32 //32 steps in one rotation 
uint16_t measurements[100000];
 
#define MAXRETRIES              5           // number of receive attempts before giving up
 
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                                                                      // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                                                              // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                                                                                       // ready?
 
    GPIO_PORTB_AFSEL_R |= 0x0C;                                                                                                                 // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;                                                                                                                   // 4) enable open drain on PB3 only
 
    GPIO_PORTB_DEN_R |= 0x0C;                                                                                                                   // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;                                                                                                        // 7) disable analog functionality on PB2,3
 
                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                                                                                                 // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                          // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                                                       // 8) configure for 100 kbps clock
}

 
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
 
    return;
}
 
void PortH_Init(void){ //Stepper Motor
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                                      // activate clock for Port H
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};                 // allow time for clock to stabilize
      GPIO_PORTH_DIR_R |= 0x0F;                                                                       // configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;                                                                  // disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;                                                                     // enable digital I/O on Port H pins (PH0-PH3)
                                                                                                                                                                        // configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;                                                                  // disable analog functionality on Port H pins (PH0-PH3)    
      return;
}
 
void PortN_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};
    GPIO_PORTN_DIR_R = 0b00000011;                                                                                      // PN1 and PN0 as outputs
    GPIO_PORTN_DEN_R = 0b00000011;                                                                                      // D1 is PN1 and D2 is PN0
            return;
 
}
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0){};
    GPIO_PORTF_DIR_R = 0b00010001;                                                                                      // PF4 and PF0 as outputs
    GPIO_PORTF_DEN_R = 0b00010001;                                                                                      // D3 is PF4 and D4 is PF0
            return;
}

void PortJ_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                                                  // Activate clock for Port J
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};                             // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R = 0b00000000;                                                                      // PJ1 and PJ0 as input
  GPIO_PORTJ_DEN_R = 0b00000011;                                                                // enable digital I/O on PJ1 and PJ0
      GPIO_PORTJ_PCTL_R &= ~0x000000FF;                                                                           // configure PJ1 and PJ0 as GPIO
      GPIO_PORTJ_AMSEL_R &= ~0b00000011;                                            // Disable analog functionality on PJ1 and PJ0        
      GPIO_PORTJ_PUR_R |= 0b00000011;                                                           // Enable weak pull up resistor on PJ1 and PJ0
      return;
}
//XSHUT     This pin is an active-low shutdown input;
//                            the board pulls it up to VDD to enable the sensor by default.
//                            Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}
 

void motor_step_deg(int clockwise) {
    uint32_t delay = 1;
		int motor_steps = 16; // 16 degrees per step 
    for (int i = 0; i < motor_steps; i++) {
        if (clockwise) {
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
        } else {  // reverse sequence
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait10ms(delay);
        }
    }
}

//*********************************************************************************************************
//*********************************************************************************************************
//***********                             MAIN Function                       *****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t    dev = 0x29;             //address of the ToF sensor as an I2C slave peripheral
int status=0;
 
int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum;
  uint8_t RangeStatus;
  uint8_t dataReady;
	
			int Max_Layer = 3;
      int clockwise = 1;  // Clockwise set to true
 
      //initialize
      PLL_Init();
      SysTick_Init();
      onboardLEDs_Init();
      I2C_Init();
      UART_Init();
      PortH_Init();  //motor ports PH0-PH3
      PortJ_Init();  // Push button PJ0
      PortN_Init();  

          
 
/* Those basic I2C read functions can be used to check your own I2C functions */
      status = VL53L1X_GetSensorId(dev, &wordData);
 
 
      // 1 Wait for device ToF booted
      while(sensorState==0){
            status = VL53L1X_BootState(dev, &sensorState);
            SysTick_Wait10ms(10);
  }
			
      FlashAllLEDs();
      status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

  /* 2 Initialize the sensor with the default setting  */
			status = VL53L1X_SensorInit(dev);
      Status_Check("SensorInit", status);

 
			status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
      //everyuhting in while loop has to be edited 
while(1){
	  
		//Debounce Logic 
    if ((GPIO_PORTJ_DATA_R & 0b00000001) == 0) {                  // If PJ0 is pressed trigger loop 
            while ((GPIO_PORTJ_DATA_R & 0b00000001) == 0){};  // Loop to wait for putton to be released (stops multi-clicks)
						
							FlashLED1(1); //Additional Status LED PN1
                  
    for (int layer = 0; layer < Max_Layer; layer++) 
							{
                  sprintf(printf_buffer, "%c\n", 'T');  						// Put 'T' into bufffer
                  UART_printf(printf_buffer);         							// Sedn to put PC in Ready Mode.
									FlashLED4(1); 																		//UART Transmission LED PF0
								
									for (int i = 0; i < motor_index; i++) 
									{
											dataReady = 0;
 
											while (dataReady == 0) 
											{
													status = VL53L1X_CheckForDataReady(dev, &dataReady);
													VL53L1_WaitMs(dev, 4);
											}
 
											status = VL53L1X_GetDistance(dev, &measurements[i]); //Get distance from sensor
											
											VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
											
											sprintf(printf_buffer, "%d,%d\n", i, measurements[i]); //prepare buffer to send index, distance
											
											UART_printf(printf_buffer); //send over UART
											
											FlashLED3(1); // Measurement LED PF4
 
											motor_step_deg(clockwise); 
									}
 
									VL53L1X_StopRanging(dev);     			// Wait after scan
									VL53L1X_ClearInterrupt(dev);  			// clear interrupt 
									VL53L1X_StartRanging(dev);    			// Start again
									clockwise = !clockwise;  						// change direction
									UART_printf("COMPLETE\r\n");				// Send completion message
									SysTick_Wait10ms(50);
            
            
}
}
}
}