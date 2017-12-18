//pass by references could be replaced by volatiles if they don't work
#include <stdio.h>
#include <math.h>

//These structs speak for themselves. Theyŕe just here so I can return these from functions.
struct Vector3 {
	double x;
	double y;
	double w;
};

//ditto
struct Vector4 {
	double a;
	double b;
	double c;
	double d;
};


//forward declaring everything for testiung purposes
double differentiate(double value, double *previousValue, double timeStep);
struct Vector3 rotate(double matrix[3][3], struct Vector3 input);
struct Vector4 world2Wheels(double matrix[4][3], struct Vector3 input);
struct Vector3 wheels2World(double matrix[3][4], struct Vector4 input);
double applySecondOrderTransferFunction(double input, double *prevIn, double *prevOut, double *pprevOut, double *ppprevOut, double nCoeff[2], double dCoeff[3], double timeStep);
struct Vector3 disturbanceObserver(struct Vector3 localBodyReference, struct Vector3 localBodyVelocity);
struct Vector3 getScale(struct Vector4 controlOutput);
struct Vector4 lowLevelControl(struct Vector4 input, struct Vector3 reference, double speed);
double limiter(double a,double b);
struct Vector3 humouringJelle(struct Vector3 input, double speed);

//main is here for testing.
void main(void){
	
	struct Vector4 in;
	in.a = 9000000;
	in.b = 8000000;
	in.c = 7000000;
	in.d = 6000000;

	struct Vector3 out = getScale(in);
	
	printf("The output is [%lf,%lf,%lf]. \n",out.x,out.y,out.w);
}


//tested. Speaks for itself.
double limiter(double a,double b){
	return fmin(a,b);
}


//Speaks for itself, but I dunno why it´s here. tested.
struct Vector3 humouringJelle(struct Vector3 input, double speed){

	struct Vector3 output;
	output.x = input.x;
	output.y = input.y;
	output.w = input.w;

	if(speed < 0.02){
		output.x = 0;
		output.y = 0;
		output.w = 0;
	}

	return output;
}

//allows for differentiation of a real time signal
double differentiate(double value, double *previousValue, double timeStep){
	double output = (value - *previousValue)/timeStep;
	*previousValue = value;

	return output;
}

//multiplies a 3*3 matrix by a vector of 3 elements. Tested.
struct Vector3 rotate(double matrix[3][3], struct Vector3 input){
	struct Vector3 output;

	output.x = matrix[0][0]*input.x + matrix[0][1]*input.y + matrix[0][2]*input.w;
	output.y = matrix[1][0]*input.x + matrix[1][1]*input.y + matrix[1][2]*input.w;
	output.w = matrix[2][0]*input.x + matrix[2][1]*input.y + matrix[2][2]*input.w;

	return output;
}

//multiplies a 4*3 matrix by a vector of 4 elements. Tested.
struct Vector4 world2Wheels(double matrix[4][3], struct Vector3 input){
	struct Vector4 output;

	output.a = matrix[0][0]*input.x + matrix[0][1]*input.y + matrix[0][2]*input.w;
	output.b = matrix[1][0]*input.x + matrix[1][1]*input.y + matrix[1][2]*input.w;
	output.c = matrix[2][0]*input.x + matrix[2][1]*input.y + matrix[2][2]*input.w;
	output.d = matrix[3][0]*input.x + matrix[3][1]*input.y + matrix[3][2]*input.w;	

	return output;
}

//multiplies a 3*4 matrix by a vector of 3 elements. Tested.
struct Vector3 wheels2World(double matrix[3][4], struct Vector4 input){
	struct Vector3 output;

	output.x = matrix[0][0]*input.a + matrix[0][1]*input.b + matrix[0][2]*input.c + matrix[0][3]*input.d;
	output.y = matrix[1][0]*input.a + matrix[1][1]*input.b + matrix[1][2]*input.c + matrix[1][3]*input.d;
	output.w = matrix[2][0]*input.a + matrix[2][1]*input.b + matrix[2][2]*input.c + matrix[2][3]*input.d;

	return output;
}

//Applies a transfer function to an input. This could be better designed using discrete methods.
//Pay close attention to variable names. I had to make them concise so shit can be readable.
double applySecondOrderTransferFunction(double input, double *prevIn, double *prevOut, double *pprevOut, double *ppprevOut, double nCoeff[2], double dCoeff[3], double timeStep){

	double output;

	double term1 = nCoeff[0]*differentiate(input,prevIn,timeStep);
	double term2 = nCoeff[1]*input;
	double term3 = dCoeff[0]*differentiate(differentiate(*prevOut,ppprevOut,timeStep),pprevOut,timeStep);
	double term4 = dCoeff[1]*differentiate(*prevOut,pprevOut,timeStep);

	output = (term1 + term2 + term3 + term4)/dCoeff[2];

	return output;
}

//Takes in velocity and reference and outputs disturbance observer output.
//Same thing with variable names.
struct Vector3 disturbanceObserver(struct Vector3 localBodyReference, struct Vector3 localBodyVelocity){

	//initial values for the filters
	static double prevFilter1Out = 0;
	static double pprevFilter1Out = 0;
	static double ppprevFilter1Out = 0;
	static double prevFilter2Out = 0;
	static double pprevFilter2Out = 0;
	static double ppprevFilter2Out = 0;
	static double prevFilter3Out = 0;
	static double pprevFilter3Out = 0;
	static double ppprevFilter3Out = 0;

	//initial values for the inverse plants
	static double prevIP1In = 0;
	static double prevIP1Out = 0;
	static double pprevIP1Out = 0;
	static double ppprevIP1Out = 0;
	static double prevIP2In = 0;
	static double prevIP2Out = 0;
	static double pprevIP2Out = 0;
	static double ppprevIP2Out = 0;
	static double prevIP3In = 0;
	static double prevIP3Out = 0;
	static double pprevIP3Out = 0;
	static double ppprevIP3Out = 0;

	//transfer function coefficients
	double nFilterCoeff[2] = {0,1};
	double dFilterCoeff[3] = {0.1,0.44,1};
	double nIPCoeff[2] = {2,10};
	double dIPCoeff[3] = {1,4.4,10};

	struct Vector3 output;

	//arbitrary timestep
	output.x = -1*applySecondOrderTransferFunction(localBodyReference.x,0,&prevFilter1Out,&pprevFilter1Out,&ppprevFilter1Out,nFilterCoeff,dFilterCoeff,1) + applySecondOrderTransferFunction(localBodyVelocity.x,&prevIP1In,&prevIP1Out,&pprevIP1Out,&ppprevIP1Out,nIPCoeff,dIPCoeff,1);
	output.y = -1*applySecondOrderTransferFunction(localBodyReference.y,0,&prevFilter2Out,&pprevFilter2Out,&ppprevFilter2Out,nFilterCoeff,dFilterCoeff,1) + applySecondOrderTransferFunction(localBodyVelocity.y,&prevIP2In,&prevIP2Out,&pprevIP2Out,&ppprevIP2Out,nIPCoeff,dIPCoeff,1);
	output.w = -1*applySecondOrderTransferFunction(localBodyReference.w,0,&prevFilter3Out,&pprevFilter3Out,&ppprevFilter3Out,nFilterCoeff,dFilterCoeff,1) + applySecondOrderTransferFunction(localBodyVelocity.w,&prevIP3In,&prevIP3Out,&pprevIP3Out,&ppprevIP3Out,nIPCoeff,dIPCoeff,1);

	return output;
}

//fetches the scaling to apply to the compensation signal. No fucking clue how these values were found.
struct Vector3 getScale(struct Vector4 controlOutput){

	double maxEl = fmax(fmax(controlOutput.a,controlOutput.b),fmax(controlOutput.c,controlOutput.d))*0.0000015;//unnecessaary. Divide by the inverse.
	double scale = 1;

	if(maxEl > 0.12){
		scale = 0.12/maxEl;
	}

	struct Vector3 output;
	output.x = scale;
	output.y = scale;
	output.w = 1;

	return output;
}

// Takes in a 3D reference, a 4D motor velocity, and a robot speed, and outputs a 4D motor velocity command.
// This calls disturbanceObserver and needs no further transfer functions, so no more cancerous initial conditions

struct Vector4 lowLevelControl(struct Vector4 input, struct Vector3 reference, double speed){

	//This is the M^T matrix.
	double w2w_t[3][4] = {{0.5/0.0275,0.5/0.0275,0.5/0.0275,0.5/0.0275},
						{0.866/0.0275,-0.866/0.0275,0.866/0.0275,-0.866/0.0275},
						{0.09/0.0275,0.09/0.0275,-0.09/0.0275,-0.09/0.0275}};

	//This is M matrix.
	double w2w[4][3] = {{0.5/0.0275,0.866/0.0275,0.09/0.0275},
						{0.5/0.0275,-0.866/0.0275,0.09/0.0275},
						{0.5/0.0275,0.866/0.0275,-0.09/0.0275},
						{0.5/0.0275,-0.866/0.0275,-0.09/0.0275}};

	// Converts real wheel velocities into real robot velocity.
	struct Vector3 localVel = wheels2World(w2w_t,input);

	//This needs to be initialised as it comes from a loop. Hopefully static does what I think it does. This is the velocity reference in 3D.
	static struct Vector3 localBody;
	localBody.x = 0;
	localBody.y = 0;
	localBody.w = 0;

	//computes disturbance observer output
	struct Vector3 obsOut = disturbanceObserver(localBody,localVel);

	//computes filtered disturbance observer output.
	struct Vector3 obsOut1 = humouringJelle(obsOut,speed);

	//placeholder
	struct Vector3 sum1;
	sum1.x = reference.x - obsOut1.x;
	sum1.y = reference.y - obsOut1.y;
	sum1.w = reference.w - obsOut1.w;

	//This also needs to be thus initialised as getScale needs it.
	static struct Vector4 controlOutput;
	controlOutput.a = 0;
	controlOutput.b = 0;
	controlOutput.c = 0;
	controlOutput.d = 0;

	//ditto
	static struct Vector4 sum2;
	sum2.a = 0;
	sum2.b = 0;
	sum2.c = 0;
	sum2.d = 0;

	//computes scaling factor for localBody
	struct Vector3 scale = getScale(controlOutput);

	//perform scaling
	localBody.x = sum1.x * scale.x;
	localBody.y = sum1.y * scale.y;
	localBody.w = sum1.w * scale.w;

	//converts the reference back into 4D after compensation
	struct Vector4 compensatedRef = world2Wheels(w2w,localBody);


	//subtracts the input to get a control output
	sum2.a = compensatedRef.a - input.a;
	sum2.b = compensatedRef.b - input.b;
	sum2.c = compensatedRef.c - input.c;
	sum2.d = compensatedRef.d - input.d;

	//applies P gain
	controlOutput.a = sum2.a*4000;
	controlOutput.b = sum2.b*4000;
	controlOutput.c = sum2.c*4000;
	controlOutput.d = sum2.d*4000;

	return controlOutput;
}