// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_COMPUTE_KE_H
#define OPENTISSUE_FEM_COMPUTE_KE_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace fem {
        namespace detail {
            template <typename real_type, typename vector_type, typename matrix_type>
            inline void compute_Ke(vector_type const &p0, vector_type const &p1,
                vector_type const &p2, vector_type const &p3, real_type const &E,
                real_type const &nu, matrix_type Ke[4][4]) {
                using std::fabs;

                real_type d = p0(0);
                real_type d4 = p0(1);
                real_type d8 = p0(2);
                real_type d1 = p1(0) - d;
                real_type d5 = p1(1) - d4;
                real_type d9 = p1(2) - d8;
                real_type d2 = p2(0) - d;
                real_type d6 = p2(1) - d4;
                real_type d10 = p2(2) - d8;
                real_type d3 = p3(0) - d;
                real_type d7 = p3(1) - d4;
                real_type d11 = p3(2) - d8;
                real_type d12 = (d1 * d6 * d11 + d2 * d7 * d9 + d3 * d5 * d10)
                    - d1 * d7 * d10 - d2 * d5 * d11 - d3 * d6 * d9;
                real_type d13 = 1.0 / d12;
                real_type d14 = fabs(d12 / 6.0);

                vector_type B[4];
                B[0].clear();
                B[1].clear();
                B[2].clear();
                B[3].clear();

                B[1](0) = (d10 * d7 - d6 * d11) * d13;
                B[2](0) = (d5 * d11 - d9 * d7) * d13;
                B[3](0) = (d9 * d6 - d5 * d10) * d13;
                B[0](0) = -B[1](0) - B[2](0) - B[3](0);
                B[1](1) = (d2 * d11 - d10 * d3) * d13;
                B[2](1) = (d9 * d3 - d1 * d11) * d13;
                B[3](1) = (d1 * d10 - d9 * d2) * d13;
                B[0](1) = -B[1](1) - B[2](1) - B[3](1);
                B[1](2) = (d6 * d3 - d2 * d7) * d13;
                B[2](2) = (d1 * d7 - d5 * d3) * d13;
                B[3](2) = (d5 * d2 - d1 * d6) * d13;
                B[0](2) = -B[1](2) - B[2](2) - B[3](2);

                real_type d15 = E / (1.0 + nu) / (1.0 - 2 * nu);
                real_type d16 = (1.0 - nu) * d15;
                real_type d17 = nu * d15;
                real_type d18 = E / 2 / (1.0 + nu);

                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        real_type d19 = B[i](0);
                        real_type d20 = B[i](1);
                        real_type d21 = B[i](2);
                        real_type d22 = B[j](0);
                        real_type d23 = B[j](1);
                        real_type d24 = B[j](2);
                        Ke[i][j](0, 0) = d16 * d19 * d22 + d18 * (d20 * d23 + d21 * d24);
                        Ke[i][j](0, 1) = d17 * d19 * d23 + d18 * (d20 * d22);
                        Ke[i][j](0, 2) = d17 * d19 * d24 + d18 * (d21 * d22);
                        Ke[i][j](1, 0) = d17 * d20 * d22 + d18 * (d19 * d23);
                        Ke[i][j](1, 1) = d16 * d20 * d23 + d18 * (d19 * d22 + d21 * d24);
                        Ke[i][j](1, 2) = d17 * d20 * d24 + d18 * (d21 * d23);
                        Ke[i][j](2, 0) = d17 * d21 * d22 + d18 * (d19 * d24);
                        Ke[i][j](2, 1) = d17 * d21 * d23 + d18 * (d20 * d24);
                        Ke[i][j](2, 2) = d16 * d21 * d24 + d18 * (d20 * d23 + d19 * d22);
                        Ke[i][j] *= d14;
                    }
                }
            }

            template <typename real_type, typename vector_type, typename matrix_type>
            inline void compute_Ke(vector_type *B, vector_type const &D,
                real_type const &volume, matrix_type Ke[4][4]) {
                for (unsigned int i = 0; i < 4; ++i)
                    for (unsigned int j = i; j < 4; ++j)
                        compute_Ke_ij(B[i], D, B[j], volume, Ke[i][j]);

                for (unsigned int i = 1; i < 4; ++i)
                    for (unsigned int j = 0; j < i; ++j)
                        Ke[i][j] = trans(Ke[j][i]);
            }

            template <typename real_type, typename vector_type, typename matrix_type>
            inline void compute_Ke_ij(vector_type const &Bi, vector_type const &D,
                vector_type const &Bj, real_type const &volume, matrix_type &Ke_ij) {
                assert(Ke_ij.size1() == 3 || !"Incompatible matrix dimensions.");
                assert(Ke_ij.size2() == 3 || !"Incompatible matrix dimensions.");

                real_type bi = Bi(0);
                real_type ci = Bi(1);
                real_type di = Bi(2);

                real_type bj = Bj(0);
                real_type cj = Bj(1);
                real_type dj = Bj(2);

                real_type D0 = D(0) * volume;
                real_type D1 = D(1) * volume;
                real_type D2 = D(2) * volume;

                Ke_ij(0, 0) = D0 * bi * bj + D2 * (ci * cj + di * dj);
                Ke_ij(0, 1) = D1 * bi * cj + D2 * (ci * bj);
                Ke_ij(0, 2) = D1 * bi * dj + D2 * (di * bj);

                Ke_ij(1, 0) = D1 * ci * bj + D2 * (bi * cj);
                Ke_ij(1, 1) = D0 * ci * cj + D2 * (bi * bj + di * dj);
                Ke_ij(1, 2) = D1 * ci * dj + D2 * (di * cj);

                Ke_ij(2, 0) = D1 * di * bj + D2 * (bi * dj);
                Ke_ij(2, 1) = D1 * di * cj + D2 * (ci * dj);
                Ke_ij(2, 2) = D0 * di * dj + D2 * (ci * cj + bi * bj);
            }
        }
    }
}

#endif