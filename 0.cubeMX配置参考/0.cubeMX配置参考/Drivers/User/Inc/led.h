#ifndef __LED_H
#define __LED_H

/*------------------------------------------ LED config ----------------------------------*/

#define LED1_PIN            			 GPIO_PIN_13        				 	// LED1 pin
#define LED1_PORT           			 GPIOC                 			 	// LED1 GPIO port
#define __HAL_RCC_LED1_CLK_ENABLE    __HAL_RCC_GPIOC_CLK_ENABLE() 	// LED1 GPIO clock
 

  
/*----------------------------------------- LED control ----------------------------------*/
						
#define LED1_ON 	  	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET)		// Active-low: turn LED on
#define LED1_OFF 	  	HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET)			// Active-low: turn LED off
#define LED1_Toggle	HAL_GPIO_TogglePin(LED1_PORT,LED1_PIN);							// Toggle LED state
			
/*---------------------------------------- Function API ------------------------------------*/

void LED_Init(void);

#endif //__LED_H


