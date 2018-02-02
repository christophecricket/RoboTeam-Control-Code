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

struct Vector3 disturbanceObserver(struct Vector3 localBodyReference, struct Vector3 localBodyVelocity){
	return localBodyReference;																				//Will be made into an actual disturbance observer when Jelle decides what he wants
}

//multiplies a 4*3 matrix by a vector of 4 elements. Tested.
struct Vector4 body2Wheels(double matrix[4][3], struct Vector3 input){
	struct Vector4 output;

	output.a = matrix[0][0]*input.x + matrix[0][1]*input.y + matrix[0][2]*input.w;
	output.b = matrix[1][0]*input.x + matrix[1][1]*input.y + matrix[1][2]*input.w;
	output.c = matrix[2][0]*input.x + matrix[2][1]*input.y + matrix[2][2]*input.w;
	output.d = matrix[3][0]*input.x + matrix[3][1]*input.y + matrix[3][2]*input.w;	

	return output;
}

//multiplies a 3*4 matrix by a vector of 3 elements. Tested.
struct Vector3 wheels2Body(double matrix[3][4], struct Vector4 input){
	struct Vector3 output;

	output.x = matrix[0][0]*input.a + matrix[0][1]*input.b + matrix[0][2]*input.c + matrix[0][3]*input.d;
	output.y = matrix[1][0]*input.a + matrix[1][1]*input.b + matrix[1][2]*input.c + matrix[1][3]*input.d;
	output.w = matrix[2][0]*input.a + matrix[2][1]*input.b + matrix[2][2]*input.c + matrix[2][3]*input.d;

	return output;
}

struct Vector3 rotate(double yaw, struct Vector3 input){
	struct Vector3 output;

	output.x = cos(yaw)*input.x - sin(yaw)*input.y;
	output.y = sin(yaw)*input.x + cos(yaw)*input.y;
	output.w = input.w;

	return output;
}

//p should maybe become a vector. TBD
struct Vector3 pController(struct Vector3 input, double p){
	struct Vector3 output;
	output.x = p*input.x;
	output.y = p*input.y;
	output.w = input.w;
}

struct Vector3 limiter(struct Vector3 input,double maxEl){
	double scale;
	if (maxEl > 0.095){
		scale = 0.095/maxEl;
	}
	else{
		scale = 0;
	}

	struct Vector3 output;
	output.x = scale*input.x;
	output.y = scale*input.y;
	output.w = input.w;
	return output;
}

//observer output should just be pre-declared as 0 before any looping starts
struct Vector4 controller(struct Vector3 reference, struct Vector4 encoderData, double xsensYaw, struct Vector3 *observerOutput, struct Vector3 xsensInput){

	//This is the M^T matrix.
	double w2w_t[3][4] = {{0.5/0.0275,0.5/0.0275,0.5/0.0275,0.5/0.0275},
						{0.866/0.0275,-0.866/0.0275,0.866/0.0275,-0.866/0.0275},
						{0.09/0.0275,0.09/0.0275,-0.09/0.0275,-0.09/0.0275}};

	//This is M matrix.
	double w2w[4][3] = {{0.5/0.0275,0.866/0.0275,0.09/0.0275},
						{0.5/0.0275,-0.866/0.0275,0.09/0.0275},
						{0.5/0.0275,0.866/0.0275,-0.09/0.0275},
						{0.5/0.0275,-0.866/0.0275,-0.09/0.0275}};
	
	struct Vector3 localReference = rotate(xsensYaw,reference);

	struct Vector3 localVel = wheels2Body(w2w_t,encoderData);

	struct Vector3 error;
	error.x = localReference.x - localVel.x;
	error.y = localReference.y - localVel.y;
	error.w = localReference.w - localVel.w;

	struct Vector3 limitedError = pController(error,3);//the 3 is completely arbitrary. Obviously needs tuning.

	struct Vector3 placeholder = *observerOutput;

	struct Vector3 postObserverError;
	postObserverError.x = limitedError.x - placeholder.x;
	postObserverError.y = limitedError.y - placeholder.y;
	postObserverError.w = limitedError.w - placeholder.w;

	struct Vector4 output = body2Wheels(w2w,postObserverError);

	double maxEl = fmax(fmax(output.a,output.b),fmax(output.c,output.d))*0.0000015;
	
	struct Vector3 disturbanceObserverInput = limiter(postObserverError,maxEl);

	placeholder = disturbanceObserver(xsensInput,disturbanceObserverInput);

	return output;
}

void main(){
	printf("Everything is fine.\n");
}