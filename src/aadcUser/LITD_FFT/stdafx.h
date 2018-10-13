// stdafx.h: Includedatei f�r Standardsystem-Includedateien
// oder h�ufig verwendete projektspezifische Includedateien,
// die nur in unregelm��igen Abst�nden ge�ndert werden.
//

#pragma once

#include <stdio.h>

// TODO: Hier auf zus�tzliche Header, die das Programm erfordert, verweisen.

#ifdef WIN32
#include <windows.h>
#endif

//adtf
#include <adtf3.h>
#include <adtf_platform_inc.h>
#include <a_utils_platform_inc.h>
#include <adtf_systemsdk.h>


#ifdef WIN32
#define _USE_MATH_DEFINES
#else
#include <math.h>
#endif

#include <iostream>
#include <fftw3.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

using namespace std;
using namespace cv;