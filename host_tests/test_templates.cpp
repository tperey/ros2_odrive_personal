#include "FurataKFBase.hpp"
#include <stdio.h>

// template funcs are protected, so a tiny derived struct re-exposes it for testing.
// We never construct it (the base is abstract); we only call the static function.
struct Expose : FurataKFBase {
    using FurataKFBase::_matMul;
    using FurataKFBase::_transpose;
    using FurataKFBase::_matAdd;
};

int main() {
    float A[2][3] = {{1, 2, 3},
                     {4, 5, 6}};
    float B[3][2] = {{ 7,  8},
                     { 9, 10},
                     {11, 12}};
    float C[2][2];

    Expose::_matMul(A, B, C);

    printf("_matMul\n");
    printf("[%g %g]\n[%g %g]\n", C[0][0], C[0][1], C[1][0], C[1][1]);
    printf("expected:\n[58 64]\n[139 154]\n");


    float Ct[2][2];
    Expose::_transpose(C, Ct);

    printf("_transpose\n");
    printf("[%g %g]\n[%g %g]\n", Ct[0][0], Ct[0][1], Ct[1][0], Ct[1][1]);
    printf("expected:\n[58 139]\n[64 154]\n");

    float twoC[2][2];
    Expose::_matAdd(C, C, twoC);
    printf("_matAdd\n");
    printf("[%g %g]\n[%g %g]\n", twoC[0][0], twoC[0][1], twoC[1][0], twoC[1][1]);
    printf("expected:\n[116 128]\n[278 308]\n");


    return 0;
}
