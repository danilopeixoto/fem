// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MATH_DECOMPOSITION_H
#define OPENTISSUE_MATH_DECOMPOSITION_H

#include <opentissue/configuration.h>

#include <cmath>

namespace opentissue {
    namespace math {
        template<typename matrix_type, typename vector_type>
        inline void eigen_decomposition(matrix_type const &A, matrix_type &V,
            vector_type &diag) {
            typedef typename vector_type::value_type real_type;
            typedef typename vector_type::value_traits value_traits;

            using std::fabs;
            using std::sqrt;

            vector_type sub_diag;

            sub_diag.clear();
            diag.clear();

            V = A;

            real_type const &fM00 = V(0, 0);
            real_type fM01 = V(0, 1);
            real_type fM02 = V(0, 2);
            real_type const &fM11 = V(1, 1);
            real_type const &fM12 = V(1, 2);
            real_type const &fM22 = V(2, 2);

            diag(0) = fM00;
            sub_diag(2) = value_traits::zero();

            if (fM02 != value_traits::zero()) {
                real_type fLength = sqrt(fM01 * fM01 + fM02 * fM02);
                real_type fInvLength = (value_traits::one()) / fLength;

                fM01 *= fInvLength;
                fM02 *= fInvLength;

                real_type fQ = (value_traits::two()) * fM01 * fM12 + fM02 * (fM22 - fM11);

                diag(1) = fM11 + fM02 * fQ;
                diag(2) = fM22 - fM02 * fQ;
                sub_diag(0) = fLength;
                sub_diag(1) = fM12 - fM01 * fQ;

                V(0, 0) = value_traits::one();
                V(0, 1) = value_traits::zero();
                V(0, 2) = value_traits::zero();
                V(1, 0) = value_traits::zero();
                V(1, 1) = fM01;
                V(1, 2) = fM02;
                V(2, 0) = value_traits::zero();
                V(2, 1) = fM02;
                V(2, 2) = -fM01;
            }
            else {
                diag(1) = fM11;
                diag(2) = fM22;
                sub_diag(0) = fM01;
                sub_diag(1) = fM12;

                V(0, 0) = value_traits::one();
                V(0, 1) = value_traits::zero();
                V(0, 2) = value_traits::zero();
                V(1, 0) = value_traits::zero();
                V(1, 1) = value_traits::one();
                V(1, 2) = value_traits::zero();
                V(2, 0) = value_traits::zero();
                V(2, 1) = value_traits::zero();
                V(2, 2) = value_traits::one();
            }

            const unsigned int max_iterations = 32;
            const unsigned int dim = 3;

            for (unsigned int i0 = 0; i0 < dim; i0++) {
                unsigned int i1;

                for (i1 = 0; i1 < max_iterations; i1++) {
                    unsigned int i2;

                    for (i2 = i0; i2 <= dim - 2; i2++) {
                        real_type fTmp = fabs(diag(i2)) + fabs(diag(i2 + 1));

                        if (fabs(sub_diag(i2)) + fTmp == fTmp)
                            break;
                    }

                    if (i2 == i0)
                        break;

                    real_type fG = (diag(i0 + 1) - diag(i0)) / (value_traits::two() *
                        sub_diag(i0));
                    real_type fR = sqrt(fG * fG + value_traits::one());

                    if (fG < value_traits::zero())
                        fG = diag(i2) - diag(i0) + sub_diag(i0) / (fG - fR);
                    else
                        fG = diag(i2) - diag(i0) + sub_diag(i0) / (fG + fR);

                    real_type fSin = value_traits::one();
                    real_type fCos = value_traits::one();
                    real_type fP = value_traits::zero();

                    for (unsigned int i3 = i2 - 1; i3 > i0; i3--) {
                        real_type fF = fSin * sub_diag(i3);
                        real_type fB = fCos * sub_diag(i3);

                        if (fabs(fF) >= fabs(fG)) {
                            fCos = fG / fF;
                            fR = sqrt(fCos * fCos + value_traits::one());
                            sub_diag(i3 + 1) = fF * fR;
                            fSin = value_traits::one() / fR;
                            fCos *= fSin;
                        }
                        else {
                            fSin = fF / fG;
                            fR = sqrt(fSin * fSin + value_traits::one());
                            sub_diag(i3 + 1) = fG * fR;
                            fCos = value_traits::one() / fR;
                            fSin *= fCos;
                        }

                        fG = diag(i3 + 1) - fP;
                        fR = (diag(i3) - fG) * fSin + value_traits::two() * fB * fCos;
                        fP = fSin * fR;
                        diag(i3 + 1) = fG + fP;
                        fG = fCos * fR - fB;

                        for (unsigned int i4 = 0; i4 < dim; i4++) {
                            fF = V(i4, i3 + 1);
                            V(i4, i3 + 1) = fSin * V(i4, i3) + fCos * fF;
                            V(i4, i3) = fCos * V(i4, i3) - fSin * fF;
                        }
                    }

                    diag(i0) -= fP;
                    sub_diag(i0) = fG;
                    sub_diag(i2) = value_traits::zero();
                }

                if (i1 == max_iterations)
                    break;
            }
        }

        template<typename matrix_type, typename real_type>
        inline void polar_decomposition(matrix_type const &A, unsigned int max_iterations,
            real_type const &tolerance, matrix_type &U, matrix_type &P) {
            typedef typename matrix_type::value_traits value_traits;

            using std::pow;

            U = A;
            P = inverse(A);

            for (unsigned int i = 0; i < max_iterations; i++) {
                real_type u_one_norm = norm_1(U);

                real_type u_norm = norm_inf(U) * u_one_norm;
                real_type p_norm = norm_inf(P) * norm_1(P);

                if (u_norm == value_traits::zero() || p_norm == value_traits::zero())
                    break;

                real_type gamma = pow(p_norm / u_norm, 0.25);
                matrix_type D = value_traits::half() * (U * (gamma - value_traits::two()) +
                    trans(P) / gamma);

                U += D;
                P = inverse(U);

                if (norm_1(D) <= tolerance * u_one_norm)
                    break;
            }

            P = trans(U) * A;
            P = value_traits::half() * (P + trans(P));
        }

        template<typename matrix_type>
        inline void orthonormalize(matrix_type &A) {
            typedef typename matrix_type::value_type real_type;
            typedef typename matrix_type::value_traits value_traits;
            typedef typename matrix_type::vector_type vector_type;

            vector_type &r0 = A.row(0);
            vector_type &r1 = A.row(1);
            vector_type &r2 = A.row(2);

            real_type n0 = length(r0);

            if (n0 != value_traits::zero())
                r0 /= n0;

            r1 -= r0 * dot(r0, r1);

            real_type n1 = length(r1);

            if (n1 != value_traits::zero())
                r1 /= n1;

            r2 = cross(r0, r1);
        }
    }
}

#endif