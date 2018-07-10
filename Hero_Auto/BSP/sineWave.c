#include "sineWave.h"
double sineWave(float amp,float frequency,float time){
	return amp*sin(2*PI*frequency*time);
}
