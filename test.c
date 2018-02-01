#include <stdio.h>
#include <math.h>

double applyFilterWithZeros(double input, double *u0, double *u1, double outputTemp, double *y0, double *y1, double samplePeriod){

	double term1 = (0.694444*pow(samplePeriod,2)+0.4)*input;
	double term2 = (2*0.694444*pow(samplePeriod,2))*(*u1);																			//u(n-1)
	double term3 = (0.694444*(pow(samplePeriod,2)-0.6))*(*u0);																		//u(n-0)
	double term4 = -(1.388889*pow(samplePeriod,2)-0.555556)*(*y1);																	//y(n-1)
	double term5 = -(6.388889*pow(samplePeriod,2)+0.277778)*(*y0);																	//y(n-0)

	*u0 = *u1;
	*u1 = input;
	*y0 = *y1;
	*y1 = outputTemp;

	return term1 + term2 + term3 + term4 + term5;
}

double applyFilterZeroless(double input, double *u0, double *u1, double outputTemp, double *y0, double *y1, double samplePeriod){

	double term1 = input;
	double term2 = 2*(*u1);																			//u(n-1)
	double term3 = *u0;																		//u(n-0)
	double term4 = 2*(*y1);																	//y(n-1)
	double term5 = *y0;																	//y(n-0)

	*u0 = *u1;
	*u1 = input;
	*y0 = *y1;
	*y1 = outputTemp;

	return term1 + term2 + term3 + term4 + term5;
}

int main(){
	printf("Everything is fine. There is no need to worry\n");
}