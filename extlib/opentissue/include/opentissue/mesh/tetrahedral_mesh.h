// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_TETRAHEDRAL_MESH_H
#define OPENTISSUE_MESH_TETRAHEDRAL_MESH_H

#include <opentissue/configuration.h>

#include <opentissue/math/math.h>
#include <opentissue/mesh/core_access.h>
#include <opentissue/mesh/node.h>
#include <opentissue/mesh/tetrahedron.h>
#include <opentissue/mesh/default_traits.h>

#include <tbb/concurrent_vector.h>

namespace opentissue {
    namespace mesh {
        template<typename M = opentissue::math::Types<double, size_t>,
            typename N = DefaultNodeTraits<M>,
            typename T = DefaultTetrahedronTraits>
            class TetrahedralMesh {
            public:
                typedef M math_types;
                typedef N node_traits;
                typedef T tetrahedron_traits;
                typedef TetrahedralMesh<M, N, T> tetrahedral_mesh_type;
                typedef Node<tetrahedral_mesh_type> node_type;
                typedef Tetrahedron<tetrahedral_mesh_type> tetrahedron_type;
                typedef size_t index_type;
                typedef typename node_type::tetrahedron_circulator tetrahedron_circulator;

                static index_type const &undefined() {
                    static index_type value = math::detail::highest<index_type>();

                    return value;
                }

            protected:
                typedef tbb::concurrent_vector<node_type> node_container;
                typedef tbb::concurrent_vector<tetrahedron_type> tetrahedra_container;

                node_container m_nodes;
                tetrahedra_container m_tetrahedra;

            public:
                typedef typename tetrahedra_container::iterator tetrahedron_iterator;
                typedef typename tetrahedra_container::const_iterator const_tetrahedron_iterator;

                tetrahedron_iterator tetrahedron_begin() {
                    return m_tetrahedra.begin();
                }
                tetrahedron_iterator tetrahedron_end() {
                    return m_tetrahedra.end();
                }
                const_tetrahedron_iterator tetrahedron_begin() const {
                    return m_tetrahedra.cbegin();
                }
                const_tetrahedron_iterator tetrahedron_end() const {
                    return m_tetrahedra.cend();
                }

                typedef typename node_container::iterator node_iterator;
                typedef typename node_container::const_iterator const_node_iterator;

                node_iterator node_begin() {
                    return m_nodes.begin();
                }
                node_iterator node_end() {
                    return m_nodes.end();
                }
                const_node_iterator node_begin() const {
                    return m_nodes.cbegin();
                }
                const_node_iterator node_end() const {
                    return m_nodes.cend();
                }

                TetrahedralMesh() {}

                TetrahedralMesh(TetrahedralMesh const & copy) { *this = copy; }

                TetrahedralMesh(size_t size_nodes, size_t size_tetrahedra) {
                    m_nodes.reserve(size_nodes);
                    m_tetrahedra.reserve(size_tetrahedra);
                }

                TetrahedralMesh &operator=(TetrahedralMesh const &rhs) {
                    this->m_nodes = rhs.m_nodes;
                    this->m_tetrahedra = rhs.m_tetrahedra;

                    for (node_iterator n = this->node_begin(); n != this->node_end(); n++)
                        CoreAccess::set_owner(*n, this);

                    for (tetrahedron_iterator t = this->tetrahedron_begin();
                        t != this->tetrahedron_end(); t++)
                        CoreAccess::set_owner(*t, this);

                    return *this;
                }

                void update_mesh_data() {
                    m_nodes.shrink_to_fit();
                    m_tetrahedra.shrink_to_fit();
                }

                void clear() {
                    m_nodes.clear();
                    m_tetrahedra.clear();
                }

                node_iterator node(index_type idx) {
                    return m_nodes.begin() + idx;
                }

                const_node_iterator const_node(index_type idx) const {
                    return m_nodes.cbegin() + idx;
                }

                tetrahedron_iterator tetrahedron(index_type idx) {
                    return m_tetrahedra.begin() + idx;
                }

                const_tetrahedron_iterator const_tetrahedron(index_type idx) const {
                    return m_tetrahedra.cbegin() + idx;
                }

                size_t size_nodes() const { return m_nodes.size(); }
                size_t size_tetrahedra() const { return m_tetrahedra.size(); }

                node_iterator insert() {
                    m_nodes.push_back(node_type());

                    node_type &nd = m_nodes.back();
                    CoreAccess::set_index(nd, size_nodes() - 1);
                    CoreAccess::set_owner(nd, this);

                    return m_nodes.begin() + nd.idx();
                }

                template<typename vector_type>
                node_iterator insert(vector_type const & m_coord) {
                    node_iterator node = insert();
                    node->m_coord = m_coord;

                    return node;
                }

                tetrahedron_iterator insert(node_iterator i, node_iterator j, node_iterator k,
                    node_iterator m) {
                    m_tetrahedra.push_back(tetrahedron_type());

                    tetrahedron_type &t = m_tetrahedra.back();

                    CoreAccess::set_index(t, size_tetrahedra() - 1);
                    CoreAccess::set_owner(t, this);

                    CoreAccess::set_node0(t, i->idx());
                    CoreAccess::set_node1(t, j->idx());
                    CoreAccess::set_node2(t, k->idx());
                    CoreAccess::set_node3(t, m->idx());

                    CoreAccess::tetrahedra_push_back(*i, t.idx());
                    CoreAccess::tetrahedra_push_back(*j, t.idx());
                    CoreAccess::tetrahedra_push_back(*k, t.idx());
                    CoreAccess::tetrahedra_push_back(*m, t.idx());

                    return m_tetrahedra.begin() + t.idx();
                }

                tetrahedron_iterator insert(index_type i, index_type j, index_type k,
                    index_type m) {
                    return insert(m_nodes.begin() + i, m_nodes.begin() + j,
                        m_nodes.begin() + k, m_nodes.begin() + m);
                }

                tetrahedron_iterator find(node_iterator i, node_iterator j, node_iterator k,
                    node_iterator m) {
                    for (tetrahedron_circulator it = i->begin(); it != i->end(); it++) {
                        if (it->node_idx(0) == i->idx() && it->node_idx(1) == j->idx() &&
                            it->node_idx(2) == k->idx() && it->node_idx(3) == m->idx())
                            return m_tetrahedra.begin() + it->idx();
                    }

                    return m_tetrahedra.end();
                }

                tetrahedron_iterator erase(tetrahedron_iterator &where) {
                    tetrahedron_iterator I(m_tetrahedra.end() - 1);
                    tetrahedron_iterator last(m_tetrahedra.begin() + I->idx());

                    if (where != last)
                        this->swap(where, last);

                    this->unlink(last);
                    m_tetrahedra.pop_back();

                    return where;
                }

            protected:
                void unlink(tetrahedron_iterator &I) {
                    node_iterator i = I->i();
                    node_iterator j = I->j();
                    node_iterator k = I->k();
                    node_iterator m = I->m();

                    CoreAccess::tetrahedra_remove(*i, I->idx());
                    CoreAccess::tetrahedra_remove(*j, I->idx());
                    CoreAccess::tetrahedra_remove(*k, I->idx());
                    CoreAccess::tetrahedra_remove(*m, I->idx());

                    CoreAccess::set_node0(*I, this->undefined());
                    CoreAccess::set_node1(*I, this->undefined());
                    CoreAccess::set_node2(*I, this->undefined());
                    CoreAccess::set_node3(*I, this->undefined());
                }

                void link(tetrahedron_iterator &I, node_iterator &i, node_iterator &j,
                    node_iterator &k, node_iterator &m) {
                    CoreAccess::tetrahedra_push_back(*i, I->idx());
                    CoreAccess::tetrahedra_push_back(*j, I->idx());
                    CoreAccess::tetrahedra_push_back(*k, I->idx());
                    CoreAccess::tetrahedra_push_back(*m, I->idx());

                    CoreAccess::set_node0(*I, i->idx());
                    CoreAccess::set_node1(*I, j->idx());
                    CoreAccess::set_node2(*I, k->idx());
                    CoreAccess::set_node3(*I, m->idx());
                }

                void swap(tetrahedron_iterator &A, tetrahedron_iterator &B) {
                    index_type Aidx = A->idx();
                    index_type Bidx = B->idx();

                    node_iterator Ai = A->i();
                    node_iterator Aj = A->j();
                    node_iterator Ak = A->k();
                    node_iterator Am = A->m();

                    node_iterator Bi = B->i();
                    node_iterator Bj = B->j();
                    node_iterator Bk = B->k();
                    node_iterator Bm = B->m();

                    unlink(A);
                    unlink(B);

                    tetrahedron_traits *TA = &(*A);
                    tetrahedron_traits tmp;
                    tetrahedron_traits *TB = &(*B);
                    tmp = (*TA);
                    (*TA) = (*TB);
                    (*TB) = tmp;

                    CoreAccess::set_index(*A, Aidx);
                    CoreAccess::set_index(*B, Bidx);

                    link(A, Bi, Bj, Bk, Bm);
                    link(B, Ai, Aj, Ak, Am);
                }
        };
    }
}

#endif