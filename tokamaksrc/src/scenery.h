/*************************************************************************
 *                                                                       *
 * Tokamak Physics Engine, Copyright (C) 2002-2007 David Lam.            *
 * All rights reserved.  Email: david@tokamakphysics.com                 *
 *                       Web: www.tokamakphysics.com                     *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT for more details.                                         *
 *                                                                       *
 *************************************************************************/

#ifndef NE_SCENERY_H
#define NE_SCENERY_H

class neTriangleTree;

class neFixedTimeStepSimulator;

/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neTriangle_
*
*	Desc:
*
****************************************************************************/ 

class neTriangle_: public neTriangle
{
PLACEMENT_MAGIC
public:
//	void * operator new (size_t s, void * addr) {
//		return addr;
//	}
//	void operator delete (void *) {}

};

/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neQuadTreeNode
*
*	Desc:
*
****************************************************************************/ 

//#define NE_TREE_DIM 2

//#define NE_TREE_SECTOR_COUNT  (2 * 2)

class neTreeNode
{
public:
	void Initialise(neTriangleTree * _tree, int32_t _parent, const neV3 & minBound, const neV3 & maxBound);

	void Build(neSimpleArray<int32_t> & triIndex, int32_t level);

	void CountTriangleInSector(neSimpleArray<int32_t> &tris, neSimpleArray<int32_t> &sectorTris, const neV3 & com, int32_t i);

	int32_t CountTriangleInSector2(neSimpleArray<int32_t> &tris, const neV3 & com, int32_t sector);

	void MakeLeaf(neSimpleArray<int32_t> &tris);

	bool IsOverlapped(const neV3 & minBound, const neV3 & maxBound);

	void GetCandidateNodes(neSimpleArray<neTreeNode*> & nodes, const neV3 & minBound, const neV3 & maxBound, int32_t level);

	void SelectBound(const neV3 & com, neV3 & minBound, neV3 & maxBound, int32_t sector);

	void DrawTriangles();

	void DrawBounds();

public:
	static int32_t numOfChildren;

	void * operator new (size_t t, void * addr){
		(void)t;
		return addr;
	}
	void operator delete [] (void *, void *){}

	void operator delete (void *, void *){}

	neTreeNode();

	neTriangleTree * tree;

	int32_t parent;

	int32_t children[4];
	
	neV3 bounds[3];//min/max x,y,z

	neSimpleArray<int32_t> triangleIndices; //leaf only
};

/****************************************************************************
*
*	NE Physics Engine 
*
*	Class: neTree
*
*	Desc:
*
****************************************************************************/ 

class neTriangleTree
{
public: 
	neTriangleTree();

	~neTriangleTree();

	neBool BuildTree(neV3 * vertices, int32_t vertexCount, neTriangle * tris, int32_t triCount, neAllocatorAbstract * _alloc);

	void FreeTree();

	neTreeNode & GetRoot(){ return root;}

	bool HasTerrain() {return nodes.GetUsedCount() > 0;};

	neTreeNode & GetNode(int32_t nodeIndex);

public:
	neV3 * vertices;

	int32_t vertexCount;

	neArray<neTriangle_> triangles;

	neArray<neTreeNode> nodes;

	neAllocatorAbstract * alloc;

	neAllocatorDefault allocDef;

	neTreeNode root;

	neFixedTimeStepSimulator * sim;
};

#endif //NE_SCENERY_H
