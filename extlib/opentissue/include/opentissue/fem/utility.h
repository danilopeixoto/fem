// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_FEM_UTILITY_H
#define OPENTISSUE_FEM_UTILITY_H

#include <opentissue/configuration.h>

#include <opentissue/math/vector.h>

namespace opentissue {
    namespace fem {
        template <typename fem_mesh, typename vector_type>
        inline void apply_force(fem_mesh &mesh, vector_type const & force) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_f_external += force;
        }

        template <typename fem_mesh, typename vector_type>
        inline void clear_forces(fem_mesh &mesh) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_f_external = 0;
        }

        template <typename fem_mesh, typename vector_type>
        inline void apply_acceleration(fem_mesh &mesh, vector_type const & acceleration) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_f_external += N->m_mass * acceleration;
        }

        template <typename fem_mesh, typename vector_type>
        inline void apply_velocity(fem_mesh &mesh, vector_type const & velocity) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_velocity += velocity;
        }

        template <typename fem_mesh, typename vector_type>
        inline void apply_angular_velocity(fem_mesh &mesh,
            vector_type const & angular_velocity) {
            vector_type mass_center;

            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                mass_center += N->m_coord;

            if (mesh.size_nodes() != 0)
                mass_center /= (double)mesh.size_nodes();

            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_velocity += cross(angular_velocity, N->m_coord - mass_center);
        }

        template <typename fem_mesh>
        inline void set_fixed(fem_mesh &mesh, bool fixed) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_fixed = fixed;
        }

        template <typename fem_mesh>
        inline void update_original_coord(fem_mesh &mesh) {
            for (fem_mesh::node_iterator N = mesh.node_begin(); N != mesh.node_end(); ++N)
                N->m_model_coord = N->m_coord;
        }

        template <typename fem_mesh, typename matrix_type>
        inline void export_K(fem_mesh &mesh, matrix_type &bigK) {
            typedef typename fem_mesh::real_type real_type;
            typedef typename fem_mesh::vector_type vector_type;
            typedef typename fem_mesh::matrix_type matrix_type;
            typedef typename fem_mesh::node_iterator node_iterator;
            typedef typename fem_mesh::node_type::matrix_iterator matrix_iterator;

            unsigned int N = mesh.size_nodes();

            bigK.resize(N * 3, N * 3, false);
            bigK.clear();

            unsigned int row = 0;

            for (unsigned int i = 0; i < N; ++i) {
                node_iterator n_i = mesh.node(i);

                for (unsigned int r = 0; r < 3; ++r, ++row) {
                    if (n_i->m_fixed) {
                        bigK.push_back(row, row, 1.0);
                    }
                    else {
                        matrix_iterator Kbegin = n_i->Kbegin();
                        matrix_iterator Kend = n_i->Kend();

                        for (matrix_iterator K = Kbegin; K != Kend; ++K) {
                            unsigned int j = K->first;
                            node_iterator n_j = mesh.node(j);

                            if (n_j->m_fixed)
                                continue;

                            matrix_type &K_ij = K->second;
                            unsigned int column = j * 3;

                            for (unsigned int c = 0; c < 3; ++c)
                                bigK.push_back(row, column + c, K_ij(r, c));
                        }
                    }
                }
            }
        }
    }
}

#endif