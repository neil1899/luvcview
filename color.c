/****************************************************************************
#	 	GspcaGui:  Gspca/Spca5xx Grabber                            #
# 		Copyright (C) 2004 2005 2006 Michel Xhaard                  #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/ 
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> 
#include "color.h"

static int *LutYr_Left = NULL;
static int *LutYg_Left = NULL;;
static int *LutYb_Left = NULL;;
static int *LutVr_Left = NULL;;
static int *LutVrY_Left = NULL;;
static int *LutUb_Left = NULL;;
static int *LutUbY_Left = NULL;;
static int *LutRv_Left = NULL;
static int *LutGu_Left = NULL;
static int *LutGv_Left = NULL;
static int *LutBu_Left = NULL;

static int *LutYr_Right = NULL;
static int *LutYg_Right = NULL;;
static int *LutYb_Right = NULL;;
static int *LutVr_Right = NULL;;
static int *LutVrY_Right = NULL;;
static int *LutUb_Right = NULL;;
static int *LutUbY_Right = NULL;;
static int *LutRv_Right = NULL;
static int *LutGu_Right = NULL;
static int *LutGv_Right = NULL;
static int *LutBu_Right = NULL;

#if 0
#define RGB24_TO_Y(r,g,b) LutYr[(r)] + LutYg[(g)] + LutYb[(b)]
#define YR_TO_V(r,y) LutVr[(r)] + LutVrY[(y)]
#define YB_TO_U(b,y) LutUb[(b)] + LutUbY[(y)]

#define R_FROMYV(y,v)  CLIP((y) + LutRv[(v)])
#define G_FROMYUV(y,u,v) CLIP((y) + LutGu[(u)] + LutGv[(v)])
#define B_FROMYU(y,u) CLIP((y) + LutBu[(u)])
#endif

unsigned char
RGB24_TO_Y_Left(unsigned char r, unsigned char g, unsigned char b)
{
return (LutYr_Left[(r)] + LutYg_Left[(g)] + LutYb_Left[(b)]);
}
unsigned char
RGB24_TO_Y_Right(unsigned char r, unsigned char g, unsigned char b)
{
return (LutYr_Right[(r)] + LutYg_Right[(g)] + LutYb_Right[(b)]);
}

unsigned char
YR_TO_V_Left(unsigned char r, unsigned char y)
{
return (LutVr_Left[(r)] + LutVrY_Left[(y)]);
}
unsigned char
YR_TO_V_Right(unsigned char r, unsigned char y)
{
return (LutVr_Right[(r)] + LutVrY_Right[(y)]);
}

unsigned char
YB_TO_U_Left(unsigned char b, unsigned char y)
{
return (LutUb_Left[(b)] + LutUbY_Left[(y)]);
}
unsigned char
YB_TO_U_Right(unsigned char b, unsigned char y)
{
return (LutUb_Right[(b)] + LutUbY_Right[(y)]);
}

unsigned char
R_FROMYV_Left(unsigned char y, unsigned char v)
{
return CLIP((y) + LutRv_Left[(v)]);
}
unsigned char
R_FROMYV_Right(unsigned char y, unsigned char v)
{
return CLIP((y) + LutRv_Right[(v)]);
}

unsigned char
G_FROMYUV_Left(unsigned char y, unsigned char u, unsigned char v)
{
return CLIP((y) + LutGu_Left[(u)] + LutGv_Left[(v)]);
}
unsigned char
G_FROMYUV_Right(unsigned char y, unsigned char u, unsigned char v)
{
return CLIP((y) + LutGu_Right[(u)] + LutGv_Right[(v)]);
}

unsigned char
B_FROMYU_Left(unsigned char y, unsigned char u)
{
return CLIP((y) + LutBu_Left[(u)]);
}
unsigned char
B_FROMYU_Right(unsigned char y, unsigned char u)
{
return CLIP((y) + LutBu_Right[(u)]);
}

void initLut_Left(void)
{
	int i;
	#define Rcoef 299 
	#define Gcoef 587 
	#define Bcoef 114 
	#define Vrcoef 711 //656 //877 
	#define Ubcoef 560 //500 //493 564
	
	#define CoefRv 1402
	#define CoefGu 714 // 344
	#define CoefGv 344 // 714
	#define CoefBu 1772
	
	LutYr_Left = malloc(256*sizeof(int));
	LutYg_Left = malloc(256*sizeof(int));
	LutYb_Left = malloc(256*sizeof(int));
	LutVr_Left = malloc(256*sizeof(int));
	LutVrY_Left = malloc(256*sizeof(int));
	LutUb_Left = malloc(256*sizeof(int));
	LutUbY_Left = malloc(256*sizeof(int));
	
	LutRv_Left = malloc(256*sizeof(int));
	LutGu_Left = malloc(256*sizeof(int));
	LutGv_Left = malloc(256*sizeof(int));
	LutBu_Left = malloc(256*sizeof(int));
	for (i= 0;i < 256;i++){
	    LutYr_Left[i] = i*Rcoef/1000 ;
	    LutYg_Left[i] = i*Gcoef/1000 ;
	    LutYb_Left[i] = i*Bcoef/1000 ;
	    LutVr_Left[i] = i*Vrcoef/1000;
	    LutUb_Left[i] = i*Ubcoef/1000;
	    LutVrY_Left[i] = 128 -(i*Vrcoef/1000);
	    LutUbY_Left[i] = 128 -(i*Ubcoef/1000);
	    LutRv_Left[i] = (i-128)*CoefRv/1000;
	    LutBu_Left[i] = (i-128)*CoefBu/1000;
	    LutGu_Left[i] = (128-i)*CoefGu/1000;
	    LutGv_Left[i] = (128-i)*CoefGv/1000;
	}	
}

void initLut_Right(void)
{
	int i;
	#define Rcoef 299 
	#define Gcoef 587 
	#define Bcoef 114 
	#define Vrcoef 711 //656 //877 
	#define Ubcoef 560 //500 //493 564
	
	#define CoefRv 1402
	#define CoefGu 714 // 344
	#define CoefGv 344 // 714
	#define CoefBu 1772
	
	LutYr_Right = malloc(256*sizeof(int));
	LutYg_Right = malloc(256*sizeof(int));
	LutYb_Right = malloc(256*sizeof(int));
	LutVr_Right = malloc(256*sizeof(int));
	LutVrY_Right = malloc(256*sizeof(int));
	LutUb_Right = malloc(256*sizeof(int));
	LutUbY_Right = malloc(256*sizeof(int));
	
	LutRv_Right = malloc(256*sizeof(int));
	LutGu_Right = malloc(256*sizeof(int));
	LutGv_Right = malloc(256*sizeof(int));
	LutBu_Right = malloc(256*sizeof(int));
	for (i= 0;i < 256;i++){
	    LutYr_Right[i] = i*Rcoef/1000 ;
	    LutYg_Right[i] = i*Gcoef/1000 ;
	    LutYb_Right[i] = i*Bcoef/1000 ;
	    LutVr_Right[i] = i*Vrcoef/1000;
	    LutUb_Right[i] = i*Ubcoef/1000;
	    LutVrY_Right[i] = 128 -(i*Vrcoef/1000);
	    LutUbY_Right[i] = 128 -(i*Ubcoef/1000);
	    LutRv_Right[i] = (i-128)*CoefRv/1000;
	    LutBu_Right[i] = (i-128)*CoefBu/1000;
	    LutGu_Right[i] = (128-i)*CoefGu/1000;
	    LutGv_Right[i] = (128-i)*CoefGv/1000;
	}	
}

void freeLut_Left(void){
	free(LutYr_Left);
	free(LutYg_Left);
	free(LutYb_Left);
	free(LutVr_Left);
	free(LutVrY_Left);
	free(LutUb_Left);
	free(LutUbY_Left);
	
	free(LutRv_Left);
	free(LutGu_Left);
	free(LutGv_Left);
	free(LutBu_Left);
}

void freeLut_Right(void){
	free(LutYr_Right);
	free(LutYg_Right);
	free(LutYb_Right);
	free(LutVr_Right);
	free(LutVrY_Right);
	free(LutUb_Right);
	free(LutUbY_Right);
	
	free(LutRv_Right);
	free(LutGu_Right);
	free(LutGv_Right);
	free(LutBu_Right);
}
