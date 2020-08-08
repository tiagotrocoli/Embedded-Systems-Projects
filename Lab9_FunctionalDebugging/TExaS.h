// TExaS.h
// Runs on LM4F120/TM4C123
// Periodic timer Timer5A which will interact with debugger and grade the lab 
// It initializes on reset and runs whenever interrupts are enabled
// Jonathan Valvano
// January 15, 2016

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015

 Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// IMPORTANT: These enumerated types are specific to Lab 9.
enum InputPorts{
  SW_PIN_PF40 = 5
};

enum OutputPorts{
  LED_PIN_PF1 = 5
};

// ************TExaS_Init*****************
// Initialize grader, triggered by timer 5A
// Lab 9 does not activate ADC1, PD3, or PLL
// This needs to be called once 
// Inputs: iport input(s) connected to this port
//         oport output(s) connected to this port
// Outputs: none
void TExaS_Init(enum InputPorts iport, enum OutputPorts oport);

// ************TExaS_Stop*****************
// Stop the transfer 
// Inputs:  none
// Outputs: none
void TExaS_Stop(void);
