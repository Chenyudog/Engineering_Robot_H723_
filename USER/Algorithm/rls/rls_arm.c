#include <malloc.h>
#include "stm32h7xx_hal.h"
#include "rls_arm.h"

void RLS_Init(RLS_Info_TypeDef *RLS, uint8_t X_Size, uint8_t Y_Size, float Lamda, float P_Init)
{
    RLS->sizeof_float = sizeof(float);

    RLS->X_Size = X_Size;
    RLS->P_Size = X_Size*X_Size;
    RLS->Y_Size = Y_Size;

    RLS->Data.W = (float *)malloc(RLS->sizeof_float * X_Size);
    memset(RLS->Data.W, 0, RLS->sizeof_float * X_Size);
    Matrix_Init(&RLS->Mat.W, X_Size, 1, (float *)RLS->Data.W);

    RLS->Data.X = (float *)malloc(RLS->sizeof_float * X_Size);
    memset(RLS->Data.X, 0, RLS->sizeof_float * X_Size);
    Matrix_Init(&RLS->Mat.X, X_Size, 1, (float *)RLS->Data.X);

    RLS->Data.XT = (float *)malloc(RLS->sizeof_float * X_Size);
    memset(RLS->Data.XT, 0, RLS->sizeof_float * X_Size);
    Matrix_Init(&RLS->Mat.XT, 1, X_Size, (float *)RLS->Data.XT);

    RLS->Data.Lamda = (float *)malloc(RLS->sizeof_float);
    memset(RLS->Data.Lamda, 0, RLS->sizeof_float);
    Matrix_Init(&RLS->Mat.Lamda, 1, 1, (float *)RLS->Data.Lamda);
    RLS->Data.Lamda[0] = Lamda;

    RLS->Data.P = (float *)malloc(RLS->sizeof_float * RLS->P_Size);
    memset(RLS->Data.P, 0, RLS->sizeof_float * RLS->P_Size);
    Matrix_Init(&RLS->Mat.P, X_Size, X_Size, (float *)RLS->Data.P);
    for (int i=0; i < RLS->P_Size; i += (X_Size + 1))
        RLS->Data.P[i] = P_Init;

    RLS->Data.Y = (float *)malloc(RLS->sizeof_float * Y_Size);
    memset(RLS->Data.Y, 0, RLS->sizeof_float * Y_Size);
    Matrix_Init(&RLS->Mat.Y, Y_Size, 1, (float *)RLS->Data.Y);

    RLS->Data.U = (float *)malloc(RLS->sizeof_float * Y_Size);
    memset(RLS->Data.U, 0, RLS->sizeof_float * Y_Size);
    Matrix_Init(&RLS->Mat.U, Y_Size, 1, (float *)RLS->Data.U);

    RLS->Data.E = (float *)malloc(RLS->sizeof_float * Y_Size);
    memset(RLS->Data.E, 0, RLS->sizeof_float * Y_Size);
    Matrix_Init(&RLS->Mat.E, Y_Size, 1, (float *)RLS->Data.E);

    RLS->Data.K = (float *)malloc(RLS->sizeof_float);
    memset(RLS->Data.K, 0, RLS->sizeof_float);
    Matrix_Init(&RLS->Mat.K, X_Size, 1, (float *)RLS->Data.K);

    RLS->Data.K_Numerator = (float *)malloc(RLS->sizeof_float * RLS->P_Size);
    memset(RLS->Data.K_Numerator, 0, RLS->sizeof_float * RLS->P_Size);
    Matrix_Init(&RLS->Mat.K_Numerator, X_Size, X_Size, (float *)RLS->Data.K_Numerator);

    RLS->Data.K_Denominator = (float *)malloc(RLS->sizeof_float);
    memset(RLS->Data.K_Denominator, 0, RLS->sizeof_float);
    Matrix_Init(&RLS->Mat.K_Denominator, 1, 1, (float *)RLS->Data.K_Denominator);

    RLS->Data.Cache_Matrix[0] = (float *)malloc(RLS->sizeof_float * RLS->P_Size);
    memset(RLS->Data.Cache_Matrix[0], 0, RLS->sizeof_float * RLS->P_Size);
    Matrix_Init(&RLS->Mat.Cache_Matrix[0], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[0]);

    RLS->Data.Cache_Matrix[1] = (float *)malloc(RLS->sizeof_float * RLS->P_Size);
    memset(RLS->Data.Cache_Matrix[1], 0, RLS->sizeof_float * RLS->P_Size);
    Matrix_Init(&RLS->Mat.Cache_Matrix[1], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[1]);

    RLS->Data.Cache_Matrix[2] = (float *)malloc(RLS->sizeof_float * RLS->P_Size);
    memset(RLS->Data.Cache_Matrix[2], 0, RLS->sizeof_float * RLS->P_Size);
    Matrix_Init(&RLS->Mat.Cache_Matrix[2], X_Size, X_Size, (float *)RLS->Data.Cache_Matrix[2]);

    RLS->Data.Output = (float *)malloc(RLS->sizeof_float * X_Size);
    memset(RLS->Data.Output, 0, RLS->sizeof_float * X_Size);
    Matrix_Init(&RLS->Mat.Output, RLS->X_Size, 1, (float *)RLS->Data.Output);
}

void RLS_Update(RLS_Info_TypeDef *RLS)
{
    // XT(n)
    RLS->MatStatus = Matrix_Transpose(&RLS->Mat.X, &RLS->Mat.XT);
    //U(n)=XT(n)*W(n-1)
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.XT, &RLS->Mat.W, &RLS->Mat.U);
    // E(n) = Y(n) - U(n)
    RLS->MatStatus = Matrix_Subtract(&RLS->Mat.Y, &RLS->Mat.U, &RLS->Mat.E);
    // P(n-1)*X(n)
    RLS->Mat.K_Numerator.numRows = RLS->Mat.P.numRows;
    RLS->Mat.K_Numerator.numCols = RLS->Mat.X.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.P, &RLS->Mat.X, &RLS->Mat.K_Numerator);

    // P(n-1)*X(n)*XT(n)
    RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.K_Numerator.numRows;
    RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.XT.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K_Numerator, &RLS->Mat.XT, &RLS->Mat.Cache_Matrix[1]);

    // P(n-1)*X(n)*XT(n)*P(n-1)
    RLS->Mat.Cache_Matrix[2].numRows = RLS->Mat.Cache_Matrix[1].numRows;
    RLS->Mat.Cache_Matrix[2].numCols = RLS->Mat.P.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[1], &RLS->Mat.P, &RLS->Mat.Cache_Matrix[2]);

    // XT(n)*P(n-1)
    RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.XT.numRows;
    RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.P.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.XT, &RLS->Mat.P, &RLS->Mat.Cache_Matrix[0]);

    // XT(n)*P(n-1)*X(n)
    RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.Cache_Matrix[0].numRows;
    RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.X.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.Cache_Matrix[0], &RLS->Mat.X, &RLS->Mat.Cache_Matrix[1]);

    // lamda + XT(n)*P(n-1)*X(n)
    RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.Cache_Matrix[1].numRows;
    RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.Cache_Matrix[1].numCols;
    RLS->MatStatus = Matrix_Add(&RLS->Mat.Lamda, &RLS->Mat.Cache_Matrix[1], &RLS->Mat.Cache_Matrix[0]);

    // 1/(lamda + XT(n)*P(n-1)*X(n))

    RLS->MatStatus = Matrix_Inverse(&RLS->Mat.Cache_Matrix[0], &RLS->Mat.K_Denominator);

    // K(n)=P(n-1)*X(n)/(lamda + XT(n)*P(n-1)*X(n))
    RLS->Mat.K.numRows = RLS->Mat.K_Numerator.numRows;
    RLS->Mat.K.numCols = RLS->Mat.K_Denominator.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K_Numerator, &RLS->Mat.K_Denominator, &RLS->Mat.K);

    // W(n)=W(n-1)+K(n)*E(n)
    RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.K.numRows;
    RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.E.numCols;
    RLS->MatStatus = Matrix_Multiply(&RLS->Mat.K, &RLS->Mat.E, &RLS->Mat.Cache_Matrix[0]);

    RLS->MatStatus = Matrix_Add(&RLS->Mat.W, &RLS->Mat.Cache_Matrix[0], &RLS->Mat.Output);

    for (uint8_t i = 0; i < RLS->X_Size; i++)
    {
        RLS->Data.W[i] = RLS->Data.Output[i];
    }

    // P(n-1)*X(n)*XT(n)*P(n-1)/(lamda + XT(n)*P(n-1)*X(n))
    RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.Cache_Matrix[2].numRows;
    RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.Cache_Matrix[2].numCols;

    for (uint8_t i = 0; i < RLS->P_Size; i++)
    {
        RLS->Data.Cache_Matrix[0][i] = RLS->Data.Cache_Matrix[2][i] * RLS->Data.K_Denominator[0];
    }

    // P(n-1) - [P(n-1)*X(n)*XT(n)*P(n-1)]/[lamda + XT(n)*P(n-1)*X(n)]
    RLS->Mat.Cache_Matrix[1].numRows = RLS->Mat.P.numRows;
    RLS->Mat.Cache_Matrix[1].numCols = RLS->Mat.Cache_Matrix[0].numCols;
    RLS->MatStatus = Matrix_Subtract(&RLS->Mat.P, &RLS->Mat.Cache_Matrix[0], &RLS->Mat.Cache_Matrix[1]);

    // 1/lamda
    RLS->Mat.Cache_Matrix[0].numRows = RLS->Mat.Lamda.numRows;
    RLS->Mat.Cache_Matrix[0].numCols = RLS->Mat.Lamda.numCols;
    RLS->Mat.Cache_Matrix[0].pData[0] = 1 / RLS->Data.Lamda[0];

    // 1 / lamda *P(n-1) - [P(n-1)*X(n)*XT(n)*P(n-1)]/[lamda + XT(n)*P(n-1)*X(n)]
    for (uint8_t i = 0; i < RLS->P_Size; i++)
    {
        RLS->Data.P[i] = RLS->Data.Cache_Matrix[0][0] * RLS->Data.Cache_Matrix[1][i];
    }
}








