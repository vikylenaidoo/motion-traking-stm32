/*
 * motion_tracking.h
 *
 *  Created on: 18 Dec 2019
 *      Author: vikylenaidoo
 */

#ifndef MOTION_TRACKING_H_
#define MOTION_TRACKING_H_

void init_TIM14(void);
void init_TIM15(void);
void init_GPIOA(void);
void init_GPIOB(void);
void init_UART1(void);


void calibrate();
void update_motor_control(unsigned int current_steps);















#endif /* MOTION_TRACKING_H_ */
