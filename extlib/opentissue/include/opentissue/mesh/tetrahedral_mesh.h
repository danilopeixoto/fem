// OpenTissue Template Library
//
// A generic toolbox for physics based modeling and simulation.
// Copyright (c) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php.

#ifndef OPENTISSUE_MESH_TETRAHEDRAL_MESH_H
#define OPENTISSUE_MESH_TETRAHEDRAL_MESH_H

#include <opentissue/configuration.h>

#include <opentissue/math/constants.h>
#include <opentissue/math/basic_types.h>
#include <opentissue/mesh/mesh_core_access.h>
#include <opentissue/mesh/node.h>
#include <opentissue/mesh/tetrahedron.h>
#include <opentissue/mesh/default_point_container.h>
#include <opentissue/mesh/default_traits.h>

#include <cassert>
#include <vector>

namespace opentissue {
    namespace mesh {
        template <typename M = opentissue::math::BasicMathTypes<double, size_t>,
            typename N = mesh::DefaultNodeTraits<M>,
            typename T = mesh::DefaultTetrahedronTraits>
            class TetrahedralMesh {
            public:
                typedef M math_types;
                typedef N node_traits;
                typedef T tetrahedron_traits;
                typedef TetrahedralMesh<M, N, T> tetrahedral_mesh_type;
                typedef Node<tetrahedral_mesh_type> node_type;
                typedef Tetrahedron<tetrahedral_mesh_type> tetrahedron_type;
                typedef size_t index_type;

                static index_type const &undefined() {
                    static index_type value = math::detail::highest<index_type>();
                    return value;
                }

            protected:
                typedef std::vector<node_type> node_container;
                typedef std::vector<tetrahedron_type> tetrahedra_container;

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

            public:
                TetrahedralMesh() : m_nodes(), m_tetrahedra() {}

                TetrahedralMesh(TetrahedralMesh const & cpy) { *this = cpy; }

                TetrahedralMesh &operator=(TetrahedralMesh const &rhs) {
                    this->m_nodes = rhs.m_nodes;
                    this->m_tetrahedra = rhs.m_tetrahedra;

                    for (node_iterator n = this->node_begin(); n != this->node_end(); ++n)
                        mesh_core_access::set_owner((*n), this);

                    for (tetrahedron_iterator t = this->tetrahedron_begin();
                        t != this->tetrahedron_end(); ++t)
                        mesh_core_access::set_owner((*t), this);

                    return (*this);
                }

            public:
                void clear() {
                    m_nodes.clear();
                    m_tetrahedra.clear();
                }

                node_iterator node(index_type idx) {
                    if (!(idx >= 0 && idx < size_nodes()))
                        throw std::out_of_range("Node index out of range.");

                    return m_nodes.begin() + idx;
                }

                const_node_iterator const_node(index_type idx) const {
                    if (!(idx >= 0 && idx < size_nodes()))
                        throw std::out_of_range("Node index out of range.");

                    return m_nodes.const_begin() + idx;
                }

                tetrahedron_iterator tetrahedron(index_type idx) {
                    if (!(idx >= 0 && idx < size_tetrahedra()))
                        throw std::out_of_range("Tetrahedron index out of range.");

                    return m_tetrahedra.begin() + idx;
                }

                const_tetrahedron_iterator tetrahedron(index_type idx) const {
                    if (!(idx >= 0 && idx < size_tetrahedra()))
                        throw std::out_of_range("Tetrahedron index out of range.");

                    return m_tetrahedra.const_begin() + idx;
                }

                size_t size_nodes() const { return m_nodes.size(); }
                size_t size_tetrahedra() const { return m_tetrahedra.size(); }

            public:
                node_iterator insert() {
                    m_nodes.push_back(node_type());
                    node_type &nd = m_nodes.back();
                    mesh_core_access::set_index(nd, size_nodes() - 1);
                    mesh_core_access::set_owner(nd, this);
                    return m_nodes.begin() + nd.idx();
                }

                template <typename vector_type>
                node_iterator insert(vector_type const & m_model_coord) {
                    node_iterator node = insert();
                    node->m_model_coord = m_model_coord;

                    return node;
                }

                tetrahedron_iterator insert(index_type i, index_type j, index_type k,
                    index_type m) {
                    return insert(m_nodes.begin() + i, m_nodes.begin() + j,
                        m_nodes.begin() + k, m_nodes.begin() + m);
                }

                tetrahedron_iterator insert(node_iterator i, node_iterator j, node_iterator k,
                    node_iterator m) {
                    verify_nodes(i, j, k, m);

                    assert(find(i, j, k, m) == tetrahedron_end() ||
                        !"Duplicate tetrahedron found.");

                    m_tetrahedra.push_back(tetrahedron_type());
                    tetrahedron_type &t = m_tetrahedra.back();

                    mesh_core_access::set_index(t, size_tetrahedra() - 1);
                    mesh_core_access::set_owner(t, this);
                    mesh_core_access::set_node0(t, i->idx());
                    mesh_core_access::set_node1(t, j->idx());
                    mesh_core_access::set_node2(t, k->idx());
                    mesh_core_access::set_node3(t, m->idx());

                    mesh_core_access::tetrahedra_push_back(*i, t.idx());
                    mesh_core_access::tetrahedra_push_back(*j, t.idx());
                    mesh_core_access::tetrahedra_push_back(*k, t.idx());
                    mesh_core_access::tetrahedra_push_back(*m, t.idx());

                    return m_tetrahedra.begin() + t.idx();
                }

                tetrahedron_iterator find(node_iterator i, node_iterator j, node_iterator k,
                    node_iterator m) {
                    verify_nodes(i, j, k, m);

                    typename node_type::tetrahedron_circulator tit = i->begin();

                    for (; tit != i->end(); ++tit) {
                        if (tit->node_idx(0) == i->idx() && tit->node_idx(1) == j->idx() &&
                            tit->node_idx(2) == k->idx() && tit->node_idx(3) == m->idx())
                            return m_tetrahedra.begin() + tit->idx();
                    }

                    return m_tetrahedra.begin() + size_tetrahedra();
                }

                tetrahedron_iterator erase(tetrahedron_iterator &where) {
                    verify_tetrahedron(where);

                    tetrahedron_iterator I = tetrahedron(size_tetrahedra() - 1);
                    tetrahedron_iterator last(m_tetrahedra.begin() + I->idx());

                    if (where != last) {
                        this->swap(where, last);
                    }

                    this->unlink(last);

                    m_tetrahedra.pop_back();
                    return where;
                }

            protected:
                void unlink(tetrahedron_iterator &I) {
                    verify_tetrahedron(I);

                    node_iterator i = I->i();
                    node_iterator j = I->j();
                    node_iterator k = I->k();
                    node_iterator m = I->m();

                    verify_nodes(i, j, k, m);

                    mesh_core_access::tetrahedra_remove(*i, I->idx());
                    mesh_core_access::tetrahedra_remove(*j, I->idx());
                    mesh_core_access::tetrahedra_remove(*k, I->idx());
                    mesh_core_access::tetrahedra_remove(*m, I->idx());

                    mesh_core_access::set_node0(*I, this->undefined());
                    mesh_core_access::set_node1(*I, this->undefined());
                    mesh_core_access::set_node2(*I, this->undefined());
                    mesh_core_access::set_node3(*I, this->undefined());
                }

                void link(tetrahedron_iterator &I, node_iterator &i, node_iterator &j,
                    node_iterator &k, node_iterator &m) {
                    verify_tetrahedron(I);

                    if (I->node_idx(0) != this->undefined() ||
                        I->node_idx(1) != this->undefined() ||
                        I->node_idx(2) != this->undefined() ||
                        I->node_idx(3) != this->undefined())
                        throw std::invalid_argument("Undefined node on tetrahedron.");

                    verify_nodes(i, j, k, m);

                    mesh_core_access::tetrahedra_push_back(*i, I->idx());
                    mesh_core_access::tetrahedra_push_back(*j, I->idx());
                    mesh_core_access::tetrahedra_push_back(*k, I->idx());
                    mesh_core_access::tetrahedra_push_back(*m, I->idx());

                    mesh_core_access::set_node0(*I, i->idx());
                    mesh_core_access::set_node1(*I, j->idx());
                    mesh_core_access::set_node2(*I, k->idx());
                    mesh_core_access::set_node3(*I, m->idx());
                }

                void swap(tetrahedron_iterator &A, tetrahedron_iterator &B) {
                    if (A == B)
                        throw std::invalid_argument("Same element to swap.");

                    verify_tetrahedron(A);
                    verify_tetrahedron(B);

                    index_type Aidx = A->idx();
                    index_type Bidx = B->idx();

                    node_iterator Ai = A->i();
                    node_iterator Aj = A->j();
                    node_iterator Ak = A->k();
                    node_iterator Am = A->m();

                    verify_nodes(Ai, Aj, Ak, Am);

                    node_iterator Bi = B->i();
                    node_iterator Bj = B->j();
                    node_iterator Bk = B->k();
                    node_iterator Bm = B->m();

                    verify_nodes(Bi, Bj, Bk, Bm);

                    unlink(A);
                    unlink(B);

                    tetrahedron_traits *TA = &(*A);
                    tetrahedron_traits tmp;
                    tetrahedron_traits *TB = &(*B);
                    tmp = (*TA);
                    (*TA) = (*TB);
                    (*TB) = tmp;

                    mesh_core_access::set_index(*A, Aidx);
                    mesh_core_access::set_index(*B, Bidx);

                    link(A, Bi, Bj, Bk, Bm);
                    link(B, Ai, Aj, Ak, Am);
                }

            protected:
                void verify_nodes(node_iterator const &i, node_iterator const &j,
                    node_iterator const &k, node_iterator const &m) const {
                    if (i->owner() != this || j->owner() != this ||
                        k->owner() != this || m->owner() != this)
                        throw std::logic_error("Node don't belong to mesh.");

                    if (!(i->idx() >= 0 && i->idx() < size_nodes()) ||
                        !(j->idx() >= 0 && j->idx() < size_nodes()) ||
                        !(k->idx() >= 0 && k->idx() < size_nodes()) ||
                        !(m->idx() >= 0 && m->idx() < size_nodes()))
                        throw std::out_of_range("Node index out of range.");

                    if (i == j || i == k || i == m || j == k || j == m || k == m)
                        throw std::logic_error("Invalid tetrahedron.");
                }

                void verify_tetrahedron(tetrahedron_iterator const &I) const {
                    if (!((I->idx() >= 0) && (I->idx() < size_tetrahedra())))
                        throw std::out_of_range("Tetrahedron index out of range.");

                    if (I->owner() != this)
                        throw std::logic_error("Tetrahedron don't belong to mesh.");
                }
        };
    }
}

#endif