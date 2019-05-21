/*
 * main.h
 *
 *  Created on: May 21, 2019
 *      Author: Mark Ibrahim
 */

#ifndef MAIN_H_
#define MAIN_H_

void ResetIterms(void);
void GenerateSpeed2AngleLUT(void);
void AltitudeUpdate(float alt_rate, float dT);
void HeadingUpdate(float heading_rate, float dT);
void CompassCalDone(void);
int  TakeoffControlModes(void);
int  GetMotorsState(void);
bool IsLidarOperational(void);


#define IN_THE_AIR(X) ( ( (( X ) > 0.2) && (GetMotorsState() == 1) ) ? 1 : 0 )


#endif /* MAIN_H_ */
