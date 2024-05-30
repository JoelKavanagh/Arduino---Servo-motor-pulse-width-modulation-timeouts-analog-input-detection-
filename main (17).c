/*
 * Project2DigitalSystems3.c
 *
 * Created: 21/04/2024 
 * Author : Joel Kavanagh (22336168) - Done Individually 
 
 This program satisfies the 'Project 2 Specification' as per the EE4524 Digital Systems 3 Brightspace site.
 There are header comments over each function explaining its purpose and implementation.
 
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>


#define F_CPU 20000000
#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)//float accepted here
#define TCB_TOP 50000 //for CCMP value in TCB3
#define THREE_5V 716    // ADC reading value that corresponds to 3.5V
#define MAXCOUNT 20
#define STEP_SIZE 63
// Define constants for servo position range and PWM value calculation
#define SERVO_MIN_POS 1250 // Starting PWM position value
#define SERVO_MAX_POS (SERVO_MIN_POS + STEP_SIZE * MAXCOUNT) // Max PWM position value


volatile uint16_t current_speed_setting = 0; // Speed setting corresponding to the last command
volatile uint16_t global_adc_value = 0; // Global variable to store ADC result
volatile uint8_t new_adc_result_available = 0; // Flag to indicate a new ADC result is available
volatile uint16_t new_input_capture_data = 0; // Global variable to store ADC result
volatile uint16_t period = 0; // Global variable to store 555 timer period
volatile uint16_t Pulse_Width = 0; // Global variable to store 555 timer pulse width
volatile uint16_t high_pulse_width = 0; // Represents high pulse width
volatile uint16_t last_tcb0_capture =  0xFFFF; // Initialize to an impossible value
volatile uint8_t continuous_timer_mode = 0; // continuous timer mode setting to toggle
volatile uint8_t continuous_adc_mode = 0; // continuous ADC mode setting to toggle
volatile uint16_t last_adc_result_mV; // stores the last ADC result in mV to be displayed in continuous mode
volatile uint16_t counter = 0; // Counts the number of ISR calls
volatile uint8_t servoCounter = 0; // Steps for servo position
volatile int8_t servoDirection = 1; // Direction of servo movement: 1 for forward, -1 for backward
volatile uint8_t servoEnabled = 1; // A flag to control servo movement
volatile uint8_t timeOut = 0;

unsigned char qcntr = 0,sndcntr = 0;   /*indexes into the queue*/
unsigned char queue[50];       /*character queue*/

// Function prototypes
void CLOCK_init(void);
void InitialiseLED_PORT_bits(void);
void EVENT_SYSTEM_init(void);
void WG_SSPWM_init(void);
void Initialise_TCB0_ICP_PWFRQ(void);
void Initialise_TCB3(void);
void ADC_Init(void);
void sendmsg(char *s);
void Initialise_TCB1(void);
void loop(char ch);


/* Use a struct to make the association between PORTs and bits connected to the LED array more explicit */
struct LED_BITS
{
	PORT_t *LED_PORT;
	uint8_t bit_mapping;
};

struct LED_BITS LED_Array[10] = {
	{&PORTC, PIN5_bm}, {&PORTC, PIN4_bm}, {&PORTA, PIN0_bm}, {&PORTF, PIN5_bm}, {&PORTC, PIN6_bm}, {&PORTB, PIN2_bm}, {&PORTF, PIN4_bm}, {&PORTA, PIN1_bm}, {&PORTA, PIN2_bm}, {&PORTA, PIN3_bm}
};


/**
 * Function Name: CLOCK_init
 * Description: Initializes the clock configuration by disabling the CLK_PER Prescaler.
 * @param none
 * @return None
 */
void CLOCK_init (void)
{
	/* Disable CLK_PER Prescaler */
	ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
	/* If set from the fuses during device programming, the CPU will now run at 20MHz (default is /6) */
}

/**
 * Function Name: InitialiseLED_PORT_bits
 * Description: Initializes the LED port bits as per the Project 2 Specification Appendix. 
 * @param none
 * @return None
 */
void InitialiseLED_PORT_bits()
{
	//set the specific pins as outputs.
	PORTA.DIR |= PIN0_bm;  // PA0 - UNO D2 (LED bit 2)
	PORTF.DIR |= PIN5_bm | PIN4_bm;  // PF5 - UNO D3 (LED bit 3), PF4 - UNO D6 (LED bit 6)
	PORTC.DIR |= PIN6_bm;  // PC6 - UNO D4 (LED bit 4)
}

/**
 * Function Name: WG_SSPWM_init
 * Description: Initializes Timer A0 to control PWM for servo operation. Sets the timer to single-slope PWM mode at 50Hz for servo control, configuring duty cycle and period.
 * @param void
 * @return void
 */
void WG_SSPWM_init(void)
{
	
	PORTA.DIRSET = PIN1_bm;		// WO-1 used for PWM
	TCA0.SINGLE.CTRLA = 0b00001001;  /* (TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm); */
	/*  we need 50Hz, 20ms period */
	TCA0.SINGLE.CTRLB = 0b00100011;  /* (TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc); */
	/* Frequency generation mode, with two output pins selected - this takes over the PORT pins */
	TCA0.SINGLE.PER = 24999; /* It's up to the designer to verify the target frequency for the end use */
	/* CMP0 gives the frequency. CMP1 is an offset (delayed) version */
	TCA0.SINGLE.CMP1 = 1250;		/* Start with 0 duty cycle */
	/* No interrupts used here ! */

}
/**
 * Function Name: Initialise_TCB0_ICP_PWFRQ
 * Description: Configures TCB0 for input capture frequency and pulse-width measurement, enabling related interrupts and setting the mode.
 * @param none
 * @return None
 */
void Initialise_TCB0_ICP_PWFRQ()
{
	/* Enable TCB0 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* May do enable later */
	
	/* Configure TCB0 in Input Capture Clock Frequency Measurement mode */
	TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc;
	
	/* Enable Capture or Timeout interrupt */
	TCB0.INTCTRL = TCB_CAPT_bm;
	
	
	//from lecture 10 slide 20
	TCB0.EVCTRL = TCB_EDGE_bm | TCB_CAPTEI_bm;
	/* Using the Falling Edge here */
	
}

/**
 * Function Name: Initialise_TCB1
 * Description: Configures TCB1 for periodic interrupt generation to manage timing operations like timeouts.
 * @param none
 * @return None
 */
void Initialise_TCB1() {
	
	
	// Enable TCB1 and set CLK_PER divider to 2: Timer clock = 10MHz now/
	TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
	
	TCB1.CTRLB = TCB_CNTMODE_TIMEOUT_gc; // Set the TCB1 timer's mode to 'Timeout'.
	
	TCB1.INTCTRL = TCB_ENABLE_bm; // Enable the interrupt for TCB1.
	
	TCB1.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm; // Configure the Event Control for TCB1 and TCB_EDGE_bm to trigger on a specific edge.
	
	 // Set the Compare/Capture register. The value is set to the current value of Pulse_Width
	 // plus 6000. This configuration determines when the timeout event (or capture) will trigger.
	 // For example, if Pulse_Width captures a period from an external source, adding 6000
	 // to this value sets a new threshold for timeout or next event capture.
	 
	TCB1.CCMP = Pulse_Width + 6000; 
	
}

/**
 * Function Name: Initialise_TCB3
 * Description: Sets up TCB3 for periodic interrupts, used in various timing and control operations within the system.
 * @param none
 * @return None
 */
void Initialise_TCB3()
{
	/* Enable TCB3 and set CLK_PER divider to 2: Timer clock = 10MHz now */
	TCB3.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  /* May do enable later */
	
	/* Configure TCB0 for Periodic Interrupt Mode  */
	TCB3.CTRLB = TCB_CNTMODE_INT_gc;
	
	/* Enable Capture or Timeout interrupt */
	TCB3.INTCTRL = TCB_CAPT_bm;
	
	TCB3.CCMP = TCB_TOP;              /* Set top value for 5 ms interval */

}

/**
 * Function Name: USART3_init
 * Description: Initializes USART3 for asynchronous serial communication, setting baud rate, frame format, and enabling transmitter and receiver.
 * @param none
 * @return None
 */
static void USART3_init(void)
{
	PORTB.DIR &= ~PIN5_bm;		/* this is the RX input */
	PORTB.DIR |= PIN4_bm;		/* this is the TX output */
	
	USART3.BAUD = (uint16_t)USART3_BAUD_RATE(115200); //for baud
	
	USART3.CTRLB |= (USART_TXEN_bm | USART_RXEN_bm);  //Enables both TX and RX
	
	PORTMUX.USARTROUTEA |= PORTMUX_USART3_ALT1_gc; //Nano Every uses PB4 and PB5 so PORTMUX is needed to select PB4 and PB5 for use with USART3
	
	
	USART3.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
	USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;                              //for 8 bit data, no parity - added the other 2 settings but they can be removed as theyre default (=0)
	
	USART3.CTRLA = USART_TXCIE_bm;    //Enables TXC interrupt - the fact RXC not added means its not enabled
}

/**
 * Function Name: ADC_Init
 * Description:  Initializes Analog to Digital Converter as per the Project 2 Specification. 
 * @param none
 * @return None
 */
void ADC_Init(void)
{
	/* ADC0 register bits are set/cleared by writing binary patterns to the registers  */
	
	ADC0.CTRLA = 0b00000000; 		                   /* 10-bit resolution selected, Free Running Mode selected, ADC0 not enabled yet */
	ADC0.CTRLB = 0b00000000; 		                    /* Simple No Accumulation operation selected, this line could be omitted */
	ADC0.CTRLC = 0b01010101; 		                    /* SAMPCAP=1; REFSEL: VDD; PRESC set to DIV128 */
	ADC0.CTRLD = 0b00000000;                         /* INITDLY set to 16 CLK_ADC cycles, default 0 would be OK so this line could be omitted */
	ADC0.MUXPOS = 0b00000011;                        /* Select AIN3 (shared with PORTD3), decision based on the Shield and adapters we use */
	ADC0.EVCTRL = 0b00000001;
	ADC0.INTCTRL = 0b00000001; 		             /* Enable an interrupt when conversion complete (RESRDY) */
	
	ADC0.CTRLA |= 0b00000001;		/* Enable ADC0 and leave the other CTRLA bits unchanged, note |= */
}

/**
 * Function Name: EVENT_SYSTEM_init
 * Description: Configures the Event System of the microcontroller. This setup involves specifying sources and users for event channels, 
 * facilitating direct peripheral-to-peripheral communication without CPU intervention.
 * Specific channels are configured for timer and ADC events, optimizing response times and system efficiency in capturing and reacting to external signals.
 * @param none
 * @return None
 */
 void EVENT_SYSTEM_init ()
 {
	 /* Set Port 0 Pin 3 (PE3) as input event this is on Channel 4 */
	 EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN3_gc;
	 /* Connect user to event channel 4 */
	 EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL4_gc;		/* TCB0 is the Channel 4 User */
	 
	 /* Set TCB3 CAPT as the input event, this is on Channel 0 */
	 EVSYS.CHANNEL0 = EVSYS_GENERATOR_TCB3_CAPT_gc; //NOTE - NOT VERIFIED TO BE CORRECT
	 /* Connect ADC0 to event channel 0 */
	 EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL0_gc;     /* ADC0 is the Channel 0 User */
	 
	 EVSYS.USERTCB1 = EVSYS_CHANNEL_CHANNEL4_gc;
	 
 }

/**
 * Function Name: main
 * Description: Calls relevant functions, enables global interrupts.
 * Provides functionality to continuous timer and ADC modes. Resets relevant flags. Calls the loop function to give functionality to the command line in the data visualizer  
 * @param none
 * @return type int
 */
int main(void)
{
	
	char ch = ' '; // Initialize with a default or specific value
	loop(ch); // Call loop with a char
	
	/* Initial setup functions: setting up clocks, event systems, peripherals, etc. */
	CLOCK_init();
	EVENT_SYSTEM_init();
	WG_SSPWM_init();
	Initialise_TCB0_ICP_PWFRQ();
	ADC_Init();
	USART3_init();
	Initialise_TCB3();
	InitialiseLED_PORT_bits();
	Initialise_TCB1();

	
	sei(); //enable global interrupts
	
	
    
    while (1) 
    { 
		 if (USART3.STATUS & USART_RXCIF_bm)
		 {	/* If a character has been received, read it - this structure allows other code to run */
			 ch = USART3.RXDATAL;
			 loop(ch); // Process the received character
		 }
		 /* Continuous mode operations for timer and ADC */
		 
		 // Check if the continuous timer mode is enabled
		 if(continuous_timer_mode) 
		 {
			 if(new_input_capture_data) // If new timer data has been captured
			 {
				 
				 
				  new_input_capture_data = 0; // Reset the flag to indicate that the new data has been handled
				  
				 if(timeOut == 1) // If a timeout has been detected
				 {
				
					 sendmsg("Timeout Detected\n"); // Send the timeout detected message
					 
				 } else {
					 char period_str[60];
					 snprintf(period_str, sizeof(period_str), "Timer period: %u us\n", period);
					 // Format a string that includes the captured timer period in microseconds
					 sendmsg(period_str);
					 
				 }
				 
				 
				 }
			 }
			 // Check if the continuous ADC mode is enabled
		else if (continuous_adc_mode) 
		{
			if(new_adc_result_available) // If new ADC data has been captured
			{
				char adc_str[50];
				snprintf(adc_str, sizeof(adc_str), "ADC result: %u mV\n", last_adc_result_mV);
				sendmsg(adc_str);   // Send the ADC result message
				new_adc_result_available = 0; // Reset the flag to indicate the ADC data has been handled
			}
		}
	}
}


/**
 * Function Name: ISR(ADC0_RESRDY_vect)
 * Description: Interrupt Service Routine for ADC. Reads the ADC result into a global variable global_adc_value. Clears the RESRDY flag. Sets flag to indicate new ADC result available 
 * (to be checked in main). 
 * @param none
 * @return None
 */
ISR(ADC0_RESRDY_vect) {
	
	global_adc_value = ADC0.RES; // Read the ADC result into the global variable
	
	ADC0.INTFLAGS = 0b00000001; // clear RESRDY flag by writing 1 to it
	

	new_adc_result_available = 1; // Set flag to indicate new ADC result is available
	
	
	 // Convert ADC result to millivolts
	 last_adc_result_mV = (uint32_t)global_adc_value * 5000 / 1023;
	 
	 
	if(global_adc_value > THREE_5V) { 
		LED_Array[6].LED_PORT->OUTSET = LED_Array[6].bit_mapping; // Turn on the LED if above threshold
	}
	
	else if (global_adc_value <= THREE_5V) {
       LED_Array[6].LED_PORT->OUTCLR = LED_Array[6].bit_mapping; // Turn off the LED if below threshold
	   
	}
}

/**
 * Function Name: ISR(TCB0_INT_vect)
 * Description: Interrupt Service Routine for Timer/Counter B0. This ISR handles input capture events, calculates the pulse width and period of input signals, updates relevant system variables,
 * and manages LED indicators based on the timing characteristics observed.
 * @param none
 * @return None 
 */
ISR(TCB0_INT_vect)
{
	uint16_t clocksT, clocksP;
	
	clocksT = TCB0.CNT;
	clocksP = TCB0.CCMP;
	TCB1.CNT = 0; // Reset TCB1 to avoid false timeout detection
	
	Pulse_Width = clocksP/10;
	period = clocksT/100;		/* Convert to microseconds - divided by 100 purely for the if/else loop values readability - still outputs intended time */
	
	
	  // Check if the period is within the expected range to clear the timeout flag
	  if (period > 90 && period < 450) { // 450 = timeout threshold - a bit above max time value
		  timeOut = 0; // Clear the timeout flag as the timer is oscillating within the expected range
	  }
	 
	
	new_input_capture_data = 1; // set the requisite flag
	
	high_pulse_width = period - Pulse_Width; //calculate high and low pulse widths
	
	if (period > 150) {
		LED_Array[2].LED_PORT->OUTSET = LED_Array[2].bit_mapping; // turn on LED 2
	}
	else {
		LED_Array[2].LED_PORT->OUTCLR = LED_Array[2].bit_mapping; // turn off LED 2
	}
	
	if (period > 320) {
		LED_Array[3].LED_PORT->OUTSET = LED_Array[3].bit_mapping; // turn on LED 3
	}
	else {
		LED_Array[3].LED_PORT->OUTCLR = LED_Array[3].bit_mapping; // turn off LED 3
	}
	
	LED_Array[4].LED_PORT->OUTCLR = LED_Array[4].bit_mapping; // turn off LED 4
	
	TCB0.INTFLAGS = TCB_CAPT_bm; /* Clear the interrupt flag */

}

/**
 * Function Name: ISR(TCB1_INT_vect)
 * Description: Interrupt Service Routine for Timer/Counter B1. Used to implement a software timer that checks for timeouts, managing a specific LED based on 
 * timeout occurrences to indicate system status or errors.
 * @param none
 * @return None
 */
ISR(TCB1_INT_vect) {
	
	TCB1.INTFLAGS = TCB_CAPT_bm;
	LED_Array[4].LED_PORT->OUTSET = LED_Array[4].bit_mapping;
	timeOut = 1;
	
}

/**
 * Function Name: ISR(TCB3_INT_vect)
 * Description: Interrupt Service Routine for Timer/Counter B3. Manages periodic events for servo motor control, including updating servo position based on 
 * predefined speed settings and reversing direction at boundaries.
 * @param none
 * @return None
 */
ISR(TCB3_INT_vect) {
	TCB3.INTFLAGS = TCB_CAPT_bm; // Clear the interrupt flag
	
	// Increment the ISR call counter
	counter++;
	if (servoEnabled) {
		// Check if counter has reached the current speed setting
		if (counter >= current_speed_setting) {
			counter = 0; // Reset counter for next period

			// Update the servo position and reverse direction if necessary
			if (servoCounter <= 0) {
				servoDirection = 1; // Change direction to forward
				} else if (servoCounter >= MAXCOUNT) {
				servoDirection = -1; // Change direction to backward
			}

			servoCounter += servoDirection; // Increment or decrement the servo position counter

			// Set the new PWM value for the servo position
			TCA0.SINGLE.CMP1BUF = SERVO_MIN_POS + (STEP_SIZE * servoCounter);
		}
	}
}

/**
 * Function Name: ISR(USART3_TXC_vect)
 * Description: Transmit Complete Interrupt Service Routine for USART3. Manages the sequential transmission of characters stored in a queue,
 * ensuring that each character is sent completely before sending the next one, and stops when the queue is empty.
 * @param none
 * @return None
 */
ISR(USART3_TXC_vect)
{
	/*send next character and increment index*/
	USART3.STATUS |= USART_TXCIF_bm;
	if (qcntr != sndcntr)
		USART3.TXDATAL = queue[sndcntr++];
	/* Stop sending when the queue is empty. TXC interrupts only happen when a character 
	   has been transmitted. Stopping sending stops the interrupts */
}

/**
 * Function Name: sendmsg
 * Description: This function queues a string message into a character queue and initiates the transmission of the first character.
 * The USART hardware takes care of sending each character in the queue sequentially. Once the first character is sent, subsequent characters are sent in the
 * USART3_TXC_vect ISR until the entire message is transmitted. This mechanism efficiently uses hardware transmission completion interrupts to ensure all characters are 
 * sent without blocking the main program.
 * @param s: A pointer to the null-terminated string that is to be sent over USART. The function increments through each character of the string, adding them to a queue for serial transmission.
 * @return None: This function does not return a value but directly interacts with the USART hardware to start the message sending process.
 */
void sendmsg (char *s)
{
	qcntr = 0;    /*preset indices*/
	sndcntr = 1;  /*set to one because first character already sent*/

	while (*s)
	queue[qcntr++] = *s++;   /*put characters into queue*/
	
	USART3.TXDATAL = queue[0];  /*send first character to start process*/
}

/**
 * Function Name: loop
 * Description: Processes input characters received via USART to control various functionalities including servo movement,
 * ADC readout, timer information, and more. The function interprets the character command, adjusts settings or states accordingly,
 * and communicates feedback or data through USART. Key operations include enabling/disabling servo movement based on specified speed settings,
 * toggling continuous operation modes for timers and ADC, and providing feedback such as ADC values, timer periods, and pulse widths.
 * This function serves as the main command interpreter for user inputs, ensuring that the system responds appropriately to control commands.
 * @param ch: The character received from USART, which determines the operation to be executed. It uses a switch-case structure to identify and execute the command associated with each character.
 * @return None: This function does not return a value but instead directly modifies the system state and outputs through peripheral interfaces.
 */
void loop(char ch)
{
	char str_buffer[60]; // Buffer for messages

	switch (ch)
	{
		case '0':
		servoEnabled = 0; // Disable servo movement
		sprintf(str_buffer, "The current speed setting is 0 (no movement)\n");
		sendmsg(str_buffer);
		break;
		case '1':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 200;
		sprintf(str_buffer, "The current speed setting is 1s per step\n");
		sendmsg(str_buffer);
		break;
		case '2':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 100;
		sprintf(str_buffer, "The current speed setting is 0.5s per step\n");
		sendmsg(str_buffer);
		break;
		case '3':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 80;
		sprintf(str_buffer, "The current speed setting is 0.4s per step\n");
		sendmsg(str_buffer);
		break;
		case '4':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 60;
		sprintf(str_buffer, "The current speed setting is 0.3s per step\n");
		sendmsg(str_buffer);
		break;
		case '5':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 50;
		sprintf(str_buffer, "The current speed setting is 0.25s per step\n");
		sendmsg(str_buffer);
		break;
		case '6':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 40;
		sprintf(str_buffer, "The current speed setting is 0.2s per step\n");
		sendmsg(str_buffer);
		break;
		case '7':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 30;
		sprintf(str_buffer, "The current speed setting is 0.15s per step\n");
		sendmsg(str_buffer);
		break;
		case '8':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 20;
		sprintf(str_buffer, "The current speed setting is 0.1s per step\n");
		sendmsg(str_buffer);
		break;
		case '9':
		servoEnabled = 1; // Enable servo movement
		current_speed_setting = 10;
		sprintf(str_buffer, "The current speed setting is 0.05s per step\n");
		sendmsg(str_buffer);
		break;
		case 'a':
		case 'A':
		sprintf(str_buffer, "The ADC result = %u\n", global_adc_value);
		sendmsg(str_buffer);
		break;
		case 'v':
		case 'V':
		sprintf(str_buffer, "The ADC result in mV = %u mV\n", last_adc_result_mV);
		sendmsg(str_buffer);
		break;
		case 't':
		case 'T':
		sprintf(str_buffer, "The period of the 555 Timer = %u us\n", period);
		sendmsg(str_buffer);
		break;
		case 'h':
		case 'H':
		sprintf(str_buffer, "High pulse width: %u us\n", high_pulse_width);
		sendmsg(str_buffer);
		break;
		case 'l':
		case 'L':
		sprintf(str_buffer, "Low pulse width: %u us\n", Pulse_Width);
		sendmsg(str_buffer);
		break;
		case 'C':
		case 'c':
		continuous_timer_mode = !continuous_timer_mode; // Toggle continuous timer mode
		break;
		case 'E':
		case 'e':
		continuous_timer_mode = 0; // Exit continuous timer mode
		break;
		case 'M':
		case 'm':
		continuous_adc_mode = !continuous_adc_mode; // Toggle continuous ADC mode
		break;
		case 'N':
		case 'n':
		continuous_adc_mode = 0; // Exit continuous ADC mode
		break;
		default:
		servoEnabled = 0; // disable servo movement
		sprintf(str_buffer, "Invalid Input\n");
		sendmsg(str_buffer);
		break;
	}
}