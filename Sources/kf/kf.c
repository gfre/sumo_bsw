/*************************************************************************************************
 * @file		kf_cgf.c
 * @ingroup		<group label>
 * @brief 		<This is a brief description.>
 *
 * <This is a detailed description.>
 *
 * @author 	<I>. <Surname>, Simon Helling@tf.uni-kiel.de, Chair of Automatic Control, University Kiel
 * @date 	23.06.2017
 *
 * @copyright @<LGPLx_x>
 *
 ***************************************************************************************************/

#define MASTER_kf_C_

/*======================================= >> #INCLUDES << ========================================*/
#include "kf.h"
#include "tacho_api.h"
#include "tacho.h"
#include "kf_cfg.h"

/*======================================= >> #DEFINES << =========================================*/

/*=================================== >> TYPE DEFINITIONS << =====================================*/

/*============================= >> LOKAL FUNCTION DECLARATIONS << ================================*/


/*=================================== >> GLOBAL VARIABLES << =====================================*/
Matrix KF_A_Transposed[] = {
		{0, 0},
		{0, 0},
};

Matrix KF_K_k[] = { // Kalman Gain
		{0,	0},
		{0,	0},
};

Matrix KF_P_k[] = { //error covariance matrix
		{0,	0},
		{0, 0},
};

Vector KF_x_k_est[] =  {
		{0}, //position
		{0}, //velocity
 };


Vector KF_x_k_prev_est[] =  {
		{0}, //position
		{0}, //velocity
 };

Vector KF_z_k[] = //measured data
{
		{0},
		{0},
};

int32_t KF_currSpeed, KF_currDelta;

/*============================== >> LOKAL FUNCTION DEFINITIONS << ================================*/

static void KF_MultMatrices(const Matrix* m, const Matrix* n, Matrix* result) //2x2
{
	result->column[0] = m->column[0] * n->column[0] + m->column[1] * (++n)->column[0];
	result->column[1] = m->column[0] * (--n)->column[1] + m->column[1] * (++n)->column[1];
	(++result)->column[0] = (++m)->column[0] * (--n)->column[0] + m->column[1] * (++n)->column[0];
	result->column[1] = m->column[0] * (--n)->column[1] + m->column[1] * (++n)->column[1];
}

static void KF_MultMatrixVector(const Matrix* m, const Vector* v, Vector* result)
{
	result->column[0] = m->column[0] * v->column[0] + m->column[1] * (++v)->column[0];
	(++result)->column[0] = (++m)->column[0] * (--v)->column[0] + m->column[1] * (++v)->column[0];
}

static void KF_TransposeMatrix(const Matrix* m, Matrix* result)
{
	result->column[0] = m->column[0];
	result->column[1] = (++m)->column[0];
	(++result)->column[0] = (--m)->column[1];
	result->column[1] = (++m)->column[1];
}

static void KF_TransposeVector(const Vector* v, Vector* result, bool isLineVector)
{
	if(isLineVector)
	{
		result->column[0] = v->column[0];
		(++result)->column[0] = v->column[1];
	}else
	{
		result->column[0] = v->column[0];
		result->column[1] = (++v)->column[0];
	}
}


static void KF_AddMatrices(const Matrix* m, const Matrix* n, Matrix* result, bool add)
{
	if(add)
	{ //add
		result->column[0] = m->column[0] + n->column[0];
		result->column[1] = m->column[1] + n->column[1];
		(++result)->column[0] = (++m)->column[0] + (++n)->column[0];
		result->column[1] = m->column[1] + n->column[1];
	}else
	{ //subtract
		result->column[0] = m->column[0] - n->column[0];
		result->column[1] = m->column[1] - n->column[1];
		(++result)->column[0] = (++m)->column[0] - (++n)->column[0];
		result->column[1] = m->column[1] - n->column[1];
	}
}
static void KF_AddVectors(const Vector* v,const Vector* w, Vector* result, bool add)
{
	result->column[0] = v->column[0] + w->column[0];
	(++result)->column[0] = (++v)->column[0] + (++w)->column[0];
}


static void KF_InvertMatrix(const Matrix* m, Matrix* result)
{
	double det = m->column[0]*(++m)->column[1] - (--m)->column[1] * (++m)->column[0];
	det = 1/det;

	result->column[0] = det * m->column[1];
	result->column[1] = -(det*(--m)->column[1]);
	(++result)->column[0] = -(det*(++m)->column[0]);
	result->column[1] = det * (--m)->column[0];
}

/*============================= >> GLOBAL FUNCTION DEFINITIONS << ================================*/


#ifdef MASTER_kf_C_
#undef MASTER_kf_C_
#endif /* !MASTER_kf_C_ */
