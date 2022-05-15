#include <main.h>
#include "my_math.h"

#include <arm_math.h>

//Taken from Quake III, I could never have thought of it myself
float fast_inverse_square_root(float number)
{
	//Intuitively, this work by calculating the log of a number and multiplying
	//it by -1/2

	uint32_t bits = * (uint32_t*) &number; //We extract the bits into an int

	//The log of a float is approximately equal to its bit representation
	//interpreted as an integer, minus an offset
	uint32_t offset = 0x5f3759df;
	bits = offset - (bits >> 1); //we replace the division by two by a bitshift
	float out = * (float*) &bits; //We convert it back to a float
	
	//We finally do a Newton iteration to make the result better
	//newOut = oldOut - f/f', with f(out) = 1/out^2 - number
	//(as we want out = 1/sqrt(x))
	//f' = -2/out^3, so newOut = oldOut + 1/2 * (oldOut - number*oldOut^3) 
	out = out * (1.5f - 0.5f*number*out*out);
	return out;
}

void normalize_vec3(float* vec)
{
	float norm_2 = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
	float inv_norm = fast_inverse_square_root(norm_2);

	vec[0] *= inv_norm;
	vec[1] *= inv_norm;
	vec[2] *= inv_norm;
}

static void doGramSchmidtVec3(float* vecSrc, float* vecDest)
{
	float dot_product[3];
	arm_dot_prod_f32(vecSrc, vecDest, 3, dot_product);
	dot_product[2] = dot_product[1] = dot_product[0];

	float proj[3] = {0};
	arm_mult_f32(vecSrc, dot_product, proj, 3);
	arm_sub_f32(vecDest, proj, vecDest, 3);
}

void normalize_matrix33(float* mat)
{
	normalize_vec3(mat);
	doGramSchmidtVec3(mat, mat+3);
	normalize_vec3(mat+3);
	doGramSchmidtVec3(mat, mat+2*3);
	doGramSchmidtVec3(mat+3, mat+2*3);
	normalize_vec3(mat+2*3);
}

