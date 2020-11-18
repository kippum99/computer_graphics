#ifndef HALFEDGE_H
#define HALFEDGE_H

#include <algorithm>
#include <cassert>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include "structs.hpp"

/* Halfedge structs */

struct HE // HE for halfedge
{
    // the vertex that this halfedge comes out off
    struct HEV *vertex;
    // the face adjacent to this halfedge
    struct HEF *face;
    // the flip and next halfedges
    struct HE *flip, *next;
};

struct HEF // HEF for halfedge face
{
    // the halfedge associated with this face
    struct HE *edge;
    // this variable is used to help orientate the halfedge when building it
    bool oriented;
};

struct HEV // HEV for halfedge vertex
{
    // the coordinates of the vertex in the mesh
    float x, y, z;
    // the halfedge going out off this vertex
    struct HE *out;
    // can be used to store an index for this vertex
    int index;
    // can be used to store the normal vector for the vertex
    Vec3f normal;
};

/* Function prototypes */

static std::pair<int, int> get_edge_key(int x, int y);
static void hash_edge(std::map<std::pair<int, int>, HE*> &edge_hash,
                      std::pair<int, int> edge_key,
                      HE *edge);

static bool check_flip(HE *edge);
static bool check_edge(HE *edge);
static bool check_face(HEF *face);

static bool orient_flip_face(HE *edge);
static bool orient_face(HEF *face);

static bool build_HE(Mesh_Data *mesh,
                     std::vector<HEV*> *hevs,
                     std::vector<HEF*> *hefs);

static void delete_HE(std::vector<HEV*> *hevs, std::vector<HEF*> *hefs);

/* Function implementations */

static std::pair<int, int> get_edge_key(int x, int y)
{
    assert(x != y);
    return std::pair<int, int>(std::min(x, y), std::max(x, y));
}

static void hash_edge(std::map<std::pair<int, int>, HE*> &edge_hash,
                     std::pair<int, int> edge_key,
                     HE *edge)
{
    if(edge_hash.count(edge_key) != 0)
    {
        HE *flip = edge_hash[edge_key];
        flip->flip = edge;
        edge->flip = flip;
    }
    else
        edge_hash[edge_key] = edge;
}

static bool check_flip(HE *edge)
{
    return edge->flip == NULL || edge->flip->vertex != edge->vertex;
}

static bool check_edge(HE *edge)
{
    return check_flip(edge) || !edge->face->oriented || !edge->flip->face->oriented;
}

static bool check_face(HEF *face)
{
    bool b1 = check_edge(face->edge);
    bool b2 = (face->edge->next != NULL) ? check_edge(face->edge->next) : 1;
    bool b3;

    if(face->edge->next != NULL)
        b3 = (face->edge->next->next != NULL) ? check_edge(face->edge->next->next) : 1;
    else
        b3 = 1;

    return b1 && b2 && b3;
}

static bool orient_flip_face(HE *edge)
{
    if(edge->flip == NULL)
        return 1;

    HE *flip = edge->flip;
    HEF *face = flip->face;

    if(face->oriented)
        return check_face(face);

    if (!check_flip(edge))
    {
        HEV *v1 = face->edge->vertex;
        HEV *v2 = face->edge->next->vertex;
        HEV *v3 = face->edge->next->next->vertex;

        assert(v1 != v2 && v1 != v3 && v2 != v3);

        HE *e3 = face->edge;
        HE *e1 = face->edge->next;
        HE *e2 = face->edge->next->next;

        assert(e3->vertex == v1);
        assert(e1->vertex == v2);
        assert(e2->vertex == v3);

        e3->vertex = v3;
        e3->next = e2;

        e1->vertex = v1;
        e1->next = e3;

        e2->vertex = v2;
        e2->next = e1;

        v1->out = e3;
        v2->out = e1;
        v3->out = e2;

        assert(face->edge->next->next->next == face->edge);
    }

    face->oriented = 1;

    assert(check_flip(edge));
    assert(check_face(face));

    return check_face(face) && orient_face(face);
}

static bool orient_face(HEF *face)
{
    assert(face->oriented);
    return orient_flip_face(face->edge)
           && orient_flip_face(face->edge->next)
           && orient_flip_face(face->edge->next->next)
           && check_face(face);
}

static bool build_HE(Mesh_Data *mesh,
                     std::vector<HEV*> *hevs,
                     std::vector<HEF*> *hefs)
{
    std::vector<Vertex*> *vertices = mesh->vertices;
    std::vector<Face*> *faces = mesh->faces;

    hevs->push_back(NULL);
    std::map<std::pair<int, int>, HE*> edge_hash;

    int size_vertices = vertices->size();

    for(int i = 1; i < size_vertices; ++i)
    {
        HEV *hev = new HEV;
        hev->x = vertices->at(i)->x;
        hev->y = vertices->at(i)->y;
        hev->z = vertices->at(i)->z;
        hev->out = NULL;

        hevs->push_back(hev);
    }

    HEF *first_face = NULL;
    int num_faces = faces->size();

    for (int i = 0; i < num_faces; ++i)
    {
        Face *f = faces->at(i);

        HE *e1 = new HE;
        HE *e2 = new HE;
        HE *e3 = new HE;

        e1->flip = NULL;
        e2->flip = NULL;
        e3->flip = NULL;

        HEF *hef = new HEF;

        hef->oriented = 0;
        hef->edge = e1;

        e1->face = hef;
        e2->face = hef;
        e3->face = hef;

        e1->next = e2;
        e2->next = e3;
        e3->next = e1;

        e1->vertex = hevs->at(f->idx1);
        e2->vertex = hevs->at(f->idx2);
        e3->vertex = hevs->at(f->idx3);

        hevs->at(f->idx1)->out = e1;
        hevs->at(f->idx2)->out = e2;
        hevs->at(f->idx3)->out = e3;

        hash_edge(edge_hash, get_edge_key(f->idx1, f->idx2), e1);
        hash_edge(edge_hash, get_edge_key(f->idx2, f->idx3), e2);
        hash_edge(edge_hash, get_edge_key(f->idx3, f->idx1), e3);

        hefs->push_back(hef);

        if(first_face == NULL)
        {
            first_face = hef;
            first_face->oriented = 1;
        }
    }

    return orient_face(first_face);
}

static void delete_HE(std::vector<HEV*> *hevs, std::vector<HEF*> *hefs)
{
    int hev_size = hevs->size();
    int num_hefs = hefs->size();

    for(int i = 1; i < hev_size; ++i)
        delete hevs->at(i);

    for(int i = 0; i < num_hefs; ++i)
    {
        delete hefs->at(i)->edge->next->next;
        delete hefs->at(i)->edge->next;
        delete hefs->at(i)->edge;
        delete hefs->at(i);
    }

    delete hevs;
    delete hefs;
}

#endif
