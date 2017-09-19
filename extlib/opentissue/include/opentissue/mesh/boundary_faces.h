// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_BOUNDARY_FACES_H
#define OPENTISSUE_MESH_BOUNDARY_FACES_H

#include <opentissue/configuration.h>

#include <cassert>
#include <vector>

namespace opentissue {
    namespace mesh {
        template <typename M, typename F> class Face : public F {
        public:
            typedef M tetrahedral_mesh_type;
            typedef F face_traits;
            typedef Face<M, F> face_type;
            typedef typename tetrahedral_mesh_type::index_type index_type;

        protected:
            index_type m_idx0;
            index_type m_idx1;
            index_type m_idx2;

        public:
            Face() : m_idx0(-1), m_idx1(-1), m_idx2(-1) {}

            Face(index_type const &index0, index_type const &index1,
                index_type const &index2)
                : m_idx0(index0), m_idx1(index1), m_idx2(index2) {}

        public:
            const index_type &idx0() const { return m_idx0; }
            const index_type &idx1() const { return m_idx1; }
            const index_type &idx2() const { return m_idx2; }
        };

        template <class M, typename F> class BoundaryFaces {
        public:
            typedef M tetrahedral_mesh_type;
            typedef F face_traits;
            typedef Face<M, F> face_type;
            typedef std::vector<face_type> face_container;
            typedef typename face_container::iterator face_iterator;
            typedef typename face_container::const_iterator const_face_iterator;

        protected:
            face_container m_faces;

        public:
            face_iterator begin() { return m_faces.begin(); }
            face_iterator end() { return m_faces.end(); }
            const_face_iterator begin() const { return m_faces.begin(); }
            const_face_iterator end() const { return m_faces.end(); }

        public:
            BoundaryFaces() : m_faces() {}

            BoundaryFaces(tetrahedral_mesh_type &mesh) : m_faces() {
                typename tetrahedral_mesh_type::tetrahedron_iterator tetrahedron;

                for (tetrahedron = mesh.tetrahedron_begin();
                    tetrahedron != mesh.tetrahedron_end(); ++tetrahedron) {
                    if (tetrahedron->jkm() == mesh.tetrahedron_end())
                        m_faces.push_back(face_type(tetrahedron->j()->idx(),
                            tetrahedron->k()->idx(),
                            tetrahedron->m()->idx()));

                    if (tetrahedron->ijm() == mesh.tetrahedron_end())
                        m_faces.push_back(face_type(tetrahedron->i()->idx(),
                            tetrahedron->j()->idx(),
                            tetrahedron->m()->idx()));

                    if (tetrahedron->kim() == mesh.tetrahedron_end())
                        m_faces.push_back(face_type(tetrahedron->k()->idx(),
                            tetrahedron->i()->idx(),
                            tetrahedron->m()->idx()));

                    if (tetrahedron->ikj() == mesh.tetrahedron_end())
                        m_faces.push_back(face_type(tetrahedron->i()->idx(),
                            tetrahedron->k()->idx(),
                            tetrahedron->j()->idx()));
                }
            }
        };
    }
}

#endif