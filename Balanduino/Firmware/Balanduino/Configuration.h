/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This file contains all the configurations for all the objects on the field
and how they have been taught to Pixy.
*/

#ifndef _configuration_h_
#define _configuration_h_

const bool GOALKEEPER = true;

//These constants holds the indexes for each object in
//the arrays 'objectIndex' and 'objectDistance'.
const int BALL = 0;
const int GOAL1 = 1;
const int GOAL2 = 2; 
const int PLAYER1 = 3; 
const int PLAYER2 = 4; 
const int EDGE = 5;

//These constants holds the actual signatures that Pixy was taught.
const int SIGN_BALL = 1;
const int SIGN_GOAL = 045;
const int SIGN_PLAYER = 023;
const int SIGN_EDGE = 067;

//These constants holds the real size in cm for all objects.
const int REAL_WIDTH_BALL = 14;
const int REAL_HEIGHT_GOAL = 47;
const int REAL_HEIGHT_PLAYER = 15;
const int REAL_HEIGHT_EDGE = 3;


#endif
