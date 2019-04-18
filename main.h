/*******************************************************************************
 *
 * main.h
 *
 * PS_2__KEYBOARD_IO main header
 *
 * 2019/03/24
 * I. Yang
 *
 ******************************************************************************/

#ifndef MAIN_H_
#define MAIN_H_

#include <msp430.h>
#include <intrinsics.h>
#include <driverlib.h>

void Init_GPIO(void);
void Init_Timer(void);
void Init_RTC(void);


#endif /* MAIN_H_ */
