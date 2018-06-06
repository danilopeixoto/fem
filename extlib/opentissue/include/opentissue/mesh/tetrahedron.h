// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_TETRAHEDRON_H
#define OPENTISSUE_MESH_TETRAHEDRON_H

#include <opentissue/configuration.h>

namespace opentissue {
    namespace mesh {
        template<typename tetrahedral_mesh_type_>
        class Tetrahedron : public tetrahedral_mesh_type_::tetrahedron_traits {
        public:
            typedef tetrahedral_mesh_type_ tetrahedral_mesh_type;
            typedef typename tetrahedral_mesh_type::node_type node_type;
            typedef typename tetrahedral_mesh_type::tetrahedron_type tetrahedron_type;
            typedef typename tetrahedral_mesh_type::index_type index_type;

            typedef typename tetrahedral_mesh_type::node_iterator node_iterator;
            typedef typename tetrahedral_mesh_type::const_node_iterator const_node_iterator;

            typedef typename tetrahedral_mesh_type::tetrahedron_iterator tetrahedron_iterator;
            typedef typename node_type::tetrahedron_circulator tetrahedron_circulator;

        protected:
            index_type m_idx;
            tetrahedral_mesh_type *m_owner;
            index_type m_nodes[4];

        private:
            friend class CoreAccess;

            void set_index(index_type idx) { m_idx = idx; }
            void set_owner(tetrahedral_mesh_type *owner) { m_owner = owner; }
            void set_node0(index_type idx) { m_nodes[0] = idx; }
            void set_node1(index_type idx) { m_nodes[1] = idx; }
            void set_node2(index_type idx) { m_nodes[2] = idx; }
            void set_node3(index_type idx) { m_nodes[3] = idx; }

        public:
            Tetrahedron() : m_idx(tetrahedral_mesh_type::undefined()), m_owner(0) {
                m_nodes[0] = tetrahedral_mesh_type::undefined();
                m_nodes[1] = tetrahedral_mesh_type::undefined();
                m_nodes[2] = tetrahedral_mesh_type::undefined();
                m_nodes[3] = tetrahedral_mesh_type::undefined();
            }

        public:
            index_type idx() const { return m_idx; }

            index_type node_idx(index_type const &local_idx) const {
                return m_nodes[local_idx];
            }

            node_iterator i() { return m_owner->node(m_nodes[0]); }
            const_node_iterator i() const { return m_owner->const_node(m_nodes[0]); }
            node_iterator j() { return m_owner->node(m_nodes[1]); }
            const_node_iterator j() const { return m_owner->const_node(m_nodes[1]); }
            node_iterator k() { return m_owner->node(m_nodes[2]); }
            const_node_iterator k() const { return m_owner->const_node(m_nodes[2]); }
            node_iterator m() { return m_owner->node(m_nodes[3]); }
            const_node_iterator m() const { return m_owner->const_node(m_nodes[3]); }

            tetrahedron_iterator jkm() const {
                node_iterator a = j();
                node_iterator b = k();
                node_iterator c = m();

                for (tetrahedron_circulator it = a->begin(); it != a->end(); it++) {
                    if (it->has_face(c, b, a))
                        return m_owner->tetrahedron(it->idx());
                }

                return m_owner->tetrahedron_end();
            }

            tetrahedron_iterator ijm() const {
                node_iterator a = i();
                node_iterator b = j();
                node_iterator c = m();

                for (tetrahedron_circulator it = a->begin(); it != a->end(); it++) {
                    if (it->has_face(c, b, a))
                        return m_owner->tetrahedron(it->idx());
                }

                return m_owner->tetrahedron_end();
            }

            tetrahedron_iterator kim() const {
                node_iterator a = k();
                node_iterator b = i();
                node_iterator c = m();

                for (tetrahedron_circulator it = a->begin(); it != a->end(); it++) {
                    if (it->has_face(c, b, a))
                        return m_owner->tetrahedron(it->idx());
                }

                return m_owner->tetrahedron_end();
            }

            tetrahedron_iterator ikj() const {
                node_iterator a = i();
                node_iterator b = k();
                node_iterator c = j();

                for (tetrahedron_circulator it = a->begin(); it != a->end(); it++) {
                    if (it->has_face(c, b, a))
                        return m_owner->tetrahedron(it->idx());
                }

                return m_owner->tetrahedron_end();
            }

            tetrahedral_mesh_type *owner() { return m_owner; }
            tetrahedral_mesh_type const *owner() const { return m_owner; }

            node_iterator node(index_type local_idx) {
                return m_owner->node(this->global_index(local_idx));
            }

            const_node_iterator node(index_type local_idx) const {
                return m_owner->const_node(this->global_index(local_idx));
            }

            node_iterator global_index_node(index_type global_idx) {
                return m_owner->node(global_idx);
            }

            const_node_iterator global_index_node(index_type global_idx) const {
                return m_owner->const_node(global_idx);
            }

            index_type global_index(index_type local_idx) const {
                return m_nodes[local_idx];
            }

            index_type local_index(index_type global_idx) const {
                if (global_idx == m_nodes[0])
                    return 0;

                if (global_idx == m_nodes[1])
                    return 1;

                if (global_idx == m_nodes[2])
                    return 2;

                if (global_idx == m_nodes[3])
                    return 3;

                return tetrahedral_mesh_type::undefined();
            }

            bool has_face(node_iterator a, node_iterator b, node_iterator c) const {
                index_type i = m_nodes[0];
                index_type j = m_nodes[1];
                index_type k = m_nodes[2];
                index_type m = m_nodes[3];

                if ((i == a->idx() && j == b->idx() && m == c->idx()) ||
                    (j == a->idx() && k == b->idx() && m == c->idx()) ||
                    (k == a->idx() && i == b->idx() && m == c->idx()) ||
                    (i == a->idx() && k == b->idx() && j == c->idx()) ||
                    (i == b->idx() && j == c->idx() && m == a->idx()) ||
                    (j == b->idx() && k == c->idx() && m == a->idx()) ||
                    (k == b->idx() && i == c->idx() && m == a->idx()) ||
                    (i == b->idx() && k == c->idx() && j == a->idx()) ||
                    (i == c->idx() && j == a->idx() && m == b->idx()) ||
                    (j == c->idx() && k == a->idx() && m == b->idx()) ||
                    (k == c->idx() && i == a->idx() && m == b->idx()) ||
                    (i == c->idx() && k == a->idx() && j == b->idx()))
                    return true;

                return false;
            }
        };
    }
}

#endif