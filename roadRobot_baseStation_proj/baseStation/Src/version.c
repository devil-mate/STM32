#include "version.h"


char Hardware_Version[25];
char Software_Version[50];

void genVersion()
{
	sprintf(Hardware_Version,"V0.7.0");
	sprintf(Software_Version,"V0.0.3(%s/%s)",__DATE__,__TIME__);
}
	