#include "testApp.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btGeometryUtil.h"
#include "LinearMath/btConvexHullComputer.h"
#include <stdio.h> //printf debugging
//#include <set>
#include <omp.h>
//Number of random voronoi points to generate for shattering
#define VORONOIPOINTS 60

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (2048)

using namespace std;

btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
//btDynamicsWorld*		m_dynamicsWorld;
btClock m_perfmTimer;

void testApp::getVerticesInsidePlanes(const btAlignedObjectArray<btVector3>& planes, btAlignedObjectArray<btVector3>& verticesOut, std::set<int>& planeIndicesOut)
{
	// Based on btGeometryUtil.cpp (Gino van den Bergen / Erwin Coumans)
	verticesOut.resize(0);
	planeIndicesOut.clear();
	const int numPlanes = planes.size();
	int i, j, k, l;
	for (i=0;i<numPlanes;i++)
	{
		const btVector3& N1 = planes[i];
		for (j=i+1;j<numPlanes;j++)
		{
			const btVector3& N2 = planes[j];
			btVector3 n1n2 = N1.cross(N2);
			if (n1n2.length2() > btScalar(0.0001))
			{
				for (k=j+1;k<numPlanes;k++)
				{
					const btVector3& N3 = planes[k];
					btVector3 n2n3 = N2.cross(N3);
					btVector3 n3n1 = N3.cross(N1);
					if ((n2n3.length2() > btScalar(0.0001)) && (n3n1.length2() > btScalar(0.0001) ))
					{
						btScalar quotient = (N1.dot(n2n3));
						if (btFabs(quotient) > btScalar(0.0001))
						{
							btVector3 potentialVertex = (n2n3 * N1[3] + n3n1 * N2[3] + n1n2 * N3[3]) * (btScalar(-1.) / quotient);
							for (l=0; l<numPlanes; l++)
							{
								const btVector3& NP = planes[l];
								if (btScalar(NP.dot(potentialVertex))+btScalar(NP[3]) > btScalar(0.000001))
									break;
							}
							if (l == numPlanes)
							{
								// vertex (three plane intersection) inside all planes
								verticesOut.push_back(potentialVertex);
								planeIndicesOut.insert(i);
								planeIndicesOut.insert(j);
								planeIndicesOut.insert(k);
							}
						}
					}
				}
			}
		}
	}
}

struct VoronoiShatterCmp	{
        SIMD_FORCE_INLINE bool operator()(const btVector3& p1, const btVector3& p2) const {
                        return p1.length2() < p2.length2();
        }
};

void testApp::voronoiBBShatter(const btAlignedObjectArray<btVector3>& points, const btVector3& bbmin, const btVector3& bbmax, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity) {
	// points define voronoi cells in world space (avoid duplicates)
	// bbmin & bbmax = bounding box min and max in local space
	// bbq & bbt = bounding box quaternion rotation and translation
	// matDensity = Material density for voronoi shard mass calculation
	btVector3 bbvx = quatRotate(bbq, btVector3(1.0, 0.0, 0.0));
	btVector3 bbvy = quatRotate(bbq, btVector3(0.0, 1.0, 0.0));
	btVector3 bbvz = quatRotate(bbq, btVector3(0.0, 0.0, 1.0));
	btQuaternion bbiq = bbq.inverse();
	btConvexHullComputer chc;
	btConvexHullComputer* convexHC = &chc;
	btAlignedObjectArray<btVector3> vertices;
	btVector3 rbb, nrbb, icp;
	btScalar nlength, maxDistance, distance;
	btAlignedObjectArray<btVector3> sortedVoronoiPoints;
	sortedVoronoiPoints.copyFromArray(points);
	btVector3 normal, plane;
	btAlignedObjectArray<btVector3> planes;
	std::set<int> planeIndices;
	std::set<int>::iterator planeIndicesIter;
	int numplaneIndices;
	int cellnum = 0;
	int i, j, k, l;

	int numpoints = points.size();
    btVector3 curVoronoiPoint;
    // These variables will store the results of the parallel for loop:
    btAlignedObjectArray< btAlignedObjectArray<btVector3> > verticesArray;verticesArray.resize(numpoints);
    btAlignedObjectArray<btVector3> curVoronoiPointArray;	curVoronoiPointArray.resize(numpoints);

	#pragma omp parallel for schedule(dynamic, 1) private(i,j,k,l,icp,rbb,nrbb,sortedVoronoiPoints,planes,curVoronoiPoint,normal,nlength,plane,planeIndices,vertices,numplaneIndices,planeIndicesIter,distance,maxDistance)
	for (i=0; i < numpoints ;i++) {
		curVoronoiPoint = curVoronoiPointArray[i] = points[i];
		btVector3 icp = quatRotate(bbiq, curVoronoiPoint - bbt);
		rbb = icp - bbmax;
		nrbb = bbmin - icp;
		planes.resize(6);
		planes[0] = bbvx; planes[0][3] = rbb.x();
		planes[1] = bbvy; planes[1][3] = rbb.y();
		planes[2] = bbvz; planes[2][3] = rbb.z();
		planes[3] = -bbvx; planes[3][3] = nrbb.x();
		planes[4] = -bbvy; planes[4][3] = nrbb.y();
		planes[5] = -bbvz; planes[5][3] = nrbb.z();
		maxDistance = SIMD_INFINITY;
		planeIndices.clear();
       	sortedVoronoiPoints.copyFromArray(points);
        for (l = 0;l<numpoints;l++)	{
            //sortedVoronoiPoints[l]=points[l]-curVoronoiPoint;	// I was thinking about resizing and setting instead of copyFromArray()...
        	sortedVoronoiPoints[l]-=curVoronoiPoint;
        }
		sortedVoronoiPoints.quickSort(VoronoiShatterCmp());
		// No need to undo the subtraction in sortedVoronoiPoints...
		for (j=1; j < numpoints; j++) {
			normal = sortedVoronoiPoints[j];// - curVoronoiPoint;
			nlength = normal.length();
			if (nlength > maxDistance)
				break;
			plane = normal.normalized();
			plane[3] = -nlength / btScalar(2.);
			planes.push_back(plane);
			getVerticesInsidePlanes(planes, vertices, planeIndices);
			if (vertices.size() == 0)
				break;
			numplaneIndices = planeIndices.size();
			if (numplaneIndices != planes.size()) {
				planeIndicesIter = planeIndices.begin();
				for (k=0; k < numplaneIndices; k++) {
					if (k != *planeIndicesIter)
						planes[k] = planes[*planeIndicesIter];
					planeIndicesIter++;
				}
				planes.resize(numplaneIndices);
			}
			maxDistance = vertices[0].length();
			for (k=1; k < vertices.size(); k++) {
				distance = vertices[k].length();
				if (maxDistance < distance)
					maxDistance = distance;
			}
			maxDistance *= btScalar(2.);
		}
		if (vertices.size() == 0)
			continue;
        verticesArray[i].copyFromArray(vertices);
   }

    for (int i=0;i<numpoints;i++)	{
        const btAlignedObjectArray<btVector3>& vertices = verticesArray[i];
        if (vertices.size()==0) continue;
        const btVector3& curVoronoiPoint = curVoronoiPointArray[i];

		// Clean-up voronoi convex shard vertices and generate edges & faces
		convexHC->compute(&vertices[0].getX(), sizeof(btVector3), vertices.size(),0.0,0.0);

		// At this point we have a complete 3D voronoi shard mesh contained in convexHC

		// Calculate volume and center of mass (Stan Melax volume integration)
		int numFaces = convexHC->faces.size();
		int v0, v1, v2; // Triangle vertices
		btScalar volume = btScalar(0.);
		btVector3 com(0., 0., 0.);
		for (j=0; j < numFaces; j++) {
			const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[j]];
			v0 = edge->getSourceVertex();
			v1 = edge->getTargetVertex();
			edge = edge->getNextEdgeOfFace();
			v2 = edge->getTargetVertex();
			while (v2 != v0) {
				// Counter-clockwise triangulated voronoi shard mesh faces (v0-v1-v2) and edges here...
				btScalar vol = convexHC->vertices[v0].triple(convexHC->vertices[v1], convexHC->vertices[v2]);
				volume += vol;
				com += vol * (convexHC->vertices[v0] + convexHC->vertices[v1] + convexHC->vertices[v2]);
				edge = edge->getNextEdgeOfFace();
				v1 = v2;
				v2 = edge->getTargetVertex();
			}
		}
		com /= volume * btScalar(4.);
		volume /= btScalar(6.);

		// Shift all vertices relative to center of mass
		int numVerts = convexHC->vertices.size();
		for (j=0; j < numVerts; j++)
		{
			convexHC->vertices[j] -= com;
		}



		// Note:
		// At this point convex hulls contained in convexHC should be accurate (line up flush with other pieces, no cracks),
		// ...however Bullet Physics rigid bodies demo visualizations appear to produce some visible cracks.
		// Use the mesh in convexHC for visual display or to perform boolean operations with.

		// Create Bullet Physics rigid body shards
		btCollisionShape* shardShape = new btConvexHullShape(&(convexHC->vertices[0].getX()), convexHC->vertices.size());
		shardShape->setMargin(0.); // for this demo; note convexHC has optional margin parameter for this
		m_collisionShapes.push_back(shardShape);
		btTransform shardTransform;
		shardTransform.setIdentity();
		shardTransform.setOrigin(curVoronoiPoint + com); // Shard's adjusted location
		btDefaultMotionState* shardMotionState = new btDefaultMotionState(shardTransform);
		btScalar shardMass(volume * matDensity);
		btVector3 shardInertia(0.,0.,0.);
		shardShape->calculateLocalInertia(shardMass, shardInertia);

        btVector3 btCentroid;
		btCentroid = shardTransform.getOrigin();

		shards.push_back( new ofxBulletCustomShape() );
		shards[0]->addShape(m_collisionShapes[0], ofVec3f(btCentroid.getX(), btCentroid.getY(), btCentroid.getZ()));

		for (int i = 0; i < m_collisionShapes.size(); i++){
		//shards.push_back( new ofxBulletCustomShape() );
		//shards[i]->addShape(m_collisionShapes[i], ofVec3f(btCentroid.getX(), btCentroid.getY(), btCentroid.getZ()));
		//shards[i]->create( world.world, ofVec3f(btCentroid.getX(), btCentroid.getY(), btCentroid.getZ()), shardMass );
		shards[i]->create( world.world, shardTransform, shardMass );
		shards[i]->add();

		}

		//shards[m_collisionShapes.size()]->add();
        //shards[shards.size()]->add();
		//btRigidBody::btRigidBodyConstructionInfo shardRBInfo(shardMass, shardMotionState, shardShape, shardInertia);
		//btRigidBody* shardBody = new btRigidBody(shardRBInfo);
		//m_dynamicsWorld->addRigidBody(shardBody);

		cellnum ++;

	}
	printf("Generated %d voronoi btRigidBody shards\n", cellnum);
}
/*
void testApp::voronoiConvexHullShatter(const btAlignedObjectArray<btVector3>& points, const btAlignedObjectArray<btVector3>& verts, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity) {
	const bool getPlanesFromVerticesUsingConvexHullComputer =
							#ifndef DONT_GET_CONVEX_HULL_PLANES_FROM_VERTICES_USING_CONVEX_HULL_COMPUTER
								true
							#else
								false
							#endif
								;

	// points define voronoi cells in world space (avoid duplicates)
	// verts = source (convex hull) mesh vertices in local space
	// bbq & bbt = source (convex hull) mesh quaternion rotation and translation
	// matDensity = Material density for voronoi shard mass calculation
	btConvexHullComputer chc;
	btConvexHullComputer* convexHC = &chc;
	btAlignedObjectArray<btVector3> vertices, chverts;
	btVector3 rbb, nrbb;
	btScalar nlength, maxDistance, distance;
	btAlignedObjectArray<btVector3> sortedVoronoiPoints;
	sortedVoronoiPoints.copyFromArray(points);
	btVector3 normal, plane;
	btAlignedObjectArray<btVector3> planes, convexPlanes;
	std::set<int> planeIndices;
	std::set<int>::iterator planeIndicesIter;
	int numplaneIndices;
	int cellnum = 0;
	int i, j, k, l;

	// Convert verts to world space and get convexPlanes
	int numverts = verts.size();
	chverts.resize(verts.size());
	for (i=0; i < numverts ;i++) {
		chverts[i] = quatRotate(bbq, verts[i]) + bbt;
	}
	if (getPlanesFromVerticesUsingConvexHullComputer) btGeometryUtil::getPlaneEquationsFromVertices(chverts, convexPlanes);
	else	{
		convexHC->compute(&chverts[0].getX(), sizeof(btVector3), numverts, 0.0, 0.0);
		int numFaces = convexHC->faces.size();
		int v0, v1, v2; // vertices
		for (i=0; i < numFaces; i++) {
			const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[i]];
			v0 = edge->getSourceVertex();
			v1 = edge->getTargetVertex();
			edge = edge->getNextEdgeOfFace();
			v2 = edge->getTargetVertex();
			plane = (convexHC->vertices[v1]-convexHC->vertices[v0]).cross(convexHC->vertices[v2]-convexHC->vertices[v0]).normalize();
			plane[3] = -plane.dot(convexHC->vertices[v0]);
			convexPlanes.push_back(plane);
		}
	}
	const int numconvexPlanes = convexPlanes.size();

	int numpoints = points.size();
    btVector3 curVoronoiPoint;
    // These variables will store the results of the parallel for loop:
    btAlignedObjectArray< btAlignedObjectArray<btVector3> > verticesArray;verticesArray.resize(numpoints);
    btAlignedObjectArray<btVector3> curVoronoiPointArray;	curVoronoiPointArray.resize(numpoints);

    #pragma omp parallel for schedule(dynamic, 1) private(i,j,k,l,sortedVoronoiPoints,planes,curVoronoiPoint,normal,nlength,plane,planeIndices,vertices,numplaneIndices,planeIndicesIter,distance,maxDistance)
	for (i=0; i < numpoints ;i++) {
    	curVoronoiPoint = curVoronoiPointArray[i] = points[i];
        planes.copyFromArray(convexPlanes);
        for (j=0; j < numconvexPlanes ;j++) {
        	planes[j][3] += planes[j].dot(curVoronoiPoint);
        }
        maxDistance = SIMD_INFINITY;
        sortedVoronoiPoints.copyFromArray(points);
        //sortedVoronoiPoints.resize(numpoints);
        for (l = 0;l<numpoints;l++)	{
        	//sortedVoronoiPoints[l]=points[l]-curVoronoiPoint;
            sortedVoronoiPoints[l]-=curVoronoiPoint;
        }
        sortedVoronoiPoints.quickSort(VoronoiShatterCmp());
        // No need to undo the subtraction in sortedVoronoiPoints
		for (j=1; j < numpoints; j++) {
			normal = sortedVoronoiPoints[j];// - curVoronoiPoint;
			nlength = normal.length();
			if (nlength > maxDistance)
				break;
			plane = normal.normalized();
			plane[3] = -nlength / btScalar(2.);
			planes.push_back(plane);
			getVerticesInsidePlanes(planes, vertices, planeIndices);
			if (vertices.size() == 0)
				break;
			numplaneIndices = planeIndices.size();
			if (numplaneIndices != planes.size()) {
				planeIndicesIter = planeIndices.begin();
				for (k=0; k < numplaneIndices; k++) {
					if (k != *planeIndicesIter)
						planes[k] = planes[*planeIndicesIter];
					planeIndicesIter++;
				}
				planes.resize(numplaneIndices);
			}
			maxDistance = vertices[0].length();
			for (k=1; k < vertices.size(); k++) {
				distance = vertices[k].length();
				if (maxDistance < distance)
					maxDistance = distance;
			}
			maxDistance *= btScalar(2.);
		}
		if (vertices.size() == 0)
			continue;
        verticesArray[i].copyFromArray(vertices);
	}

    for (i=0;i<numpoints;i++)	{
    	const btAlignedObjectArray<btVector3>& vertices = verticesArray[i];
        if (vertices.size()==0) continue;
        const btVector3& curVoronoiPoint = curVoronoiPointArray[i];

		// Clean-up voronoi convex shard vertices and generate edges & faces
		convexHC->compute(&vertices[0].getX(), sizeof(btVector3), vertices.size(),0.0,0.0);

		// At this point we have a complete 3D voronoi shard mesh contained in convexHC

		// Calculate volume and center of mass (Stan Melax volume integration)
		int numFaces = convexHC->faces.size();
		int v0,v1,v2; // Triangle vertices
		btScalar volume = btScalar(0.);
		btVector3 com(0., 0., 0.);
		for (j=0; j < numFaces; j++) {
			const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[j]];
			v0 = edge->getSourceVertex();
			v1 = edge->getTargetVertex();
			edge = edge->getNextEdgeOfFace();
			v2 = edge->getTargetVertex();
			while (v2 != v0) {
				// Counter-clockwise triangulated voronoi shard mesh faces (v0-v1-v2) and edges here...
				btScalar vol = convexHC->vertices[v0].triple(convexHC->vertices[v1], convexHC->vertices[v2]);
				volume += vol;
				com += vol * (convexHC->vertices[v0] + convexHC->vertices[v1] + convexHC->vertices[v2]);
				edge = edge->getNextEdgeOfFace();
				v1 = v2;
				v2 = edge->getTargetVertex();
			}
		}
		com /= volume * btScalar(4.);
		volume /= btScalar(6.);

		// Shift all vertices relative to center of mass
		int numVerts = convexHC->vertices.size();
		for (j=0; j < numVerts; j++)
		{
			convexHC->vertices[j] -= com;
		}

		// Note:
		// At this point convex hulls contained in convexHC should be accurate (line up flush with other pieces, no cracks),
		// ...however Bullet Physics rigid bodies demo visualizations appear to produce some visible cracks.
		// Use the mesh in convexHC for visual display or to perform boolean operations with.

		// Create Bullet Physics rigid body shards
		btCollisionShape* shardShape = new btConvexHullShape(&(convexHC->vertices[0].getX()), convexHC->vertices.size());
		shardShape->setMargin(0.); // for this demo; note convexHC has optional margin parameter for this
		m_collisionShapes.push_back(shardShape);
		btTransform shardTransform;
		shardTransform.setIdentity();
		shardTransform.setOrigin(curVoronoiPoint + com); // Shard's adjusted location
		btDefaultMotionState* shardMotionState = new btDefaultMotionState(shardTransform);
		btScalar shardMass(volume * matDensity);
		btVector3 shardInertia(0.,0.,0.);
		shardShape->calculateLocalInertia(shardMass, shardInertia);

		for (int i = 0; i < m_collisionShapes.size(); i++){
		shards.push_back( new ofxBulletCustomShape() );
		shards[i]->addShape(m_collisionShapes[i], ofVec3f(0.0f,0.0f,0.0f));
		shards[i]->create( world.world, shardTransform, shardMass );
		//shards[i]->add();

		}

		//btRigidBody::btRigidBodyConstructionInfo shardRBInfo(shardMass, shardMotionState, shardShape, shardInertia);
		//btRigidBody* shardBody = new btRigidBody(shardRBInfo);
		//m_dynamicsWorld->addRigidBody(shardBody);

		cellnum ++;

	}
    printf("Generated %d voronoi btRigidBody shards\n", cellnum);
}
*/
void testApp::createVoro(){
// ==> Voronoi Shatter Basic Demo: Random Cuboid
	{
	// Random size cuboid (defined by bounding box max and min)
	btVector3 bbmax(btScalar(rand() / btScalar(RAND_MAX)) * 6. +0.5, btScalar(rand() / btScalar(RAND_MAX)) * 6. +0.5, btScalar(rand() / btScalar(RAND_MAX)) * 6. +0.5);
	btVector3 bbmin = -bbmax;
	// Place it 10 units above ground
	btVector3 bbt(0,-20,0);
	// Use an arbitrary material density for shards (should be consitent/relative with/to rest of RBs in world)
	btScalar matDensity = 100.;
	// Using random rotation
	btQuaternion bbq(btScalar(rand() / btScalar(RAND_MAX)) * 2. -1.,btScalar(rand() / btScalar(RAND_MAX)) * 2. -1.,btScalar(rand() / btScalar(RAND_MAX)) * 2. -1.,btScalar(rand() / btScalar(RAND_MAX)) * 2. -1.);
	bbq.normalize();
	// Generate random points for voronoi cells
	btAlignedObjectArray<btVector3> points;
	btVector3 point;
	btVector3 diff = bbmax - bbmin;
	//-------------------------------------------------------------------------
        typedef enum {
                rock=0,
                wood=1,		// for chopping/splintering  (depends on random ratio)
                marble=2,	// for planar shards (marble the material, not the ball!)
                numCutTypes
        } CutType;
        const CutType cut = (const CutType)  ((int) (btScalar(rand() / btScalar(RAND_MAX))*3.f));
	if (cut!=rock)	{
		const btScalar lowCoefficient(.01);	// in [0,very low number<<1]
		if (cut == wood || cut == marble) diff.setX(diff.x()*lowCoefficient);
		if (cut == marble) diff.setZ(diff.z()*lowCoefficient);
	}
	//--------------------------------------------------------------------------
	for (int i=0; i < VORONOIPOINTS; i++) {
		// Place points within box area (points are in world coordinates)
		point = quatRotate(bbq, btVector3(btScalar(rand() / btScalar(RAND_MAX)) * diff.x() -diff.x()/2., btScalar(rand() / btScalar(RAND_MAX)) * diff.y() -diff.y()/2., btScalar(rand() / btScalar(RAND_MAX)) * diff.z() -diff.z()/2.)) + bbt;
		points.push_back(point);
	}
	m_perfmTimer.reset();
	voronoiBBShatter(points, bbmin, bbmax, bbq, bbt, matDensity);
        printf("Total Time: %f seconds (cuboid)\n", m_perfmTimer.getTimeMilliseconds()/1000.);
	}




}

//--------------------------------------------------------------
void testApp::setup() {
	ofSetFrameRate(60);
	//ofSetVerticalSync(true);
	ofBackground( 10, 10, 10);

	camera.setPosition(ofVec3f(0, -3.f, -40.f));
	camera.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, -1, 0));

	//camera.cacheMatrices(true);

	world.setup();
	world.enableGrabbing();
	world.setCamera(&camera);
	world.setGravity( ofVec3f(0, 25., 0) );

	ofVec3f startLoc;
	ofPoint dimens;
	boundsWidth = 30.;
	float hwidth = boundsWidth*.5;
	float depth = 2.;
	float hdepth = depth*.5;
	boundsShape = new ofxBulletCustomShape();
	boundsShape->create(world.world, ofVec3f(0, 0, 0), 10.);

	for(int i = 0; i < 6; i++) {
		bounds.push_back( new ofxBulletBox() );
		if(i == 0) { // ground //
			startLoc.set( 0., hwidth+hdepth, 0. );
			dimens.set(boundsWidth, depth, boundsWidth);
			}
		btBoxShape* boxShape = ofBtGetBoxCollisionShape( dimens.x, dimens.y, dimens.z );
		boundsShape->addShape( boxShape, startLoc );

		bounds[i]->create( world.world, startLoc, 0., dimens.x, dimens.y, dimens.z );
		bounds[i]->setProperties(.25, .95);
		bounds[i]->add();
	}

	bDropBox	= false;
	bDrawDebug	= false;


    createVoro();
	// let's make an object for the light to follow //
	shapes.push_back( new ofxBulletSphere() );
	((ofxBulletSphere*)shapes[0])->create(world.world, ofVec3f(0, -hwidth+5, -5), .15f, 2.);
	((ofxBulletSphere*)shapes[0])->setSphereResolution( 10 );
	((ofxBulletSphere*)shapes[0])->setActivationState( DISABLE_DEACTIVATION );
	shapes[0]->add();

	ofSetSmoothLighting(true);
	light.setAmbientColor(ofColor(.0, .0, .0));
	light.setDiffuseColor(ofColor(.0, .0, .0));
	light.setSpecularColor(ofColor(255, .1, .1));

	logoMat.setAmbientColor(ofFloatColor(0, 0, 0));
	logoMat.setDiffuseColor(ofFloatColor(150, 0, 150));
	logoMat.setSpecularColor(ofFloatColor(220, 0, 220));
	logoMat.setShininess(40);

	boundsMat.setAmbientColor(ofFloatColor(10, 9, 10));
	boundsMat.setDiffuseColor(ofFloatColor(12, 10, 12));
	boundsMat.setSpecularColor(ofFloatColor(1, 1, 1));
	boundsMat.setShininess(10);

	shapesMat.setShininess(80);
}


//--------------------------------------------------------------
void testApp::update() {

	if(bDropBox && bounds.size() > 0) {
		for (int i = 0; i < bounds.size(); i++) {
			delete bounds[i];
		}
		bounds.clear();
		boundsShape->add();
	}

	if(bDropBox) {
		ofVec3f diff = ofVec3f(0, -5, 0) - boundsShape->getPosition();
		diff *= 200.f;
		boundsShape->applyCentralForce(diff);
	}

	world.update();
	ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));

}

//--------------------------------------------------------------
void testApp::draw() {
	glEnable( GL_DEPTH_TEST );

	camera.begin();

	ofSetLineWidth(1.f);
	if(bDrawDebug) world.drawDebug();

	ofEnableLighting();
	light.enable();
	light.setPosition(shapes[0]->getPosition());
	ofSetColor(255, 255, 255);
	shapes[0]->draw();

    boundsMat.begin();
	for(int i = 0; i < m_collisionShapes.size(); i++) {
	shards[i]->draw();
	}
    boundsMat.end();

	ofSetColor(100., 100., 100.);

	if(!bDropBox) {
		boundsMat.begin();
		for(int i = 0; i < bounds.size()-1; i++) {
			bounds[i]->draw();

		}
		boundsMat.end();
	} else {
		ofNoFill();
		btScalar	m[16];
		ofGetOpenGLMatrixFromRigidBody( boundsShape->getRigidBody(), m );
		glPushMatrix();
		glMultMatrixf( m );
		ofBox(ofVec3f(0, 0,0), boundsWidth);
		glPopMatrix();
		ofFill();
	}


	ofDisableAlphaBlending();
	ofDisableBlendMode();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
    glEnable(GL_NORMALIZE);
    glDisable(GL_CULL_FACE);
	ofPoint scale		= assimpModel.getScale();

	ofSetColor(0, 0, 0);
	logoMat.begin();
	for(int i = 0; i < logos.size(); i++) {
		btScalar	m[16];
		ofGetOpenGLMatrixFromRigidBody( logos[i]->getRigidBody(), m );
		glPushMatrix();
		glMultMatrixf( m );
		glTranslatef(-logos[i]->getCentroid().x, -logos[i]->getCentroid().y, -logos[i]->getCentroid().z);
		ofScale(scale.x,scale.y,scale.z);
		assimpModel.getMesh(0).drawFaces();
		glPopMatrix();
	}
	glPopAttrib();
	logoMat.end();

	ofSetColor(15,197,138);
	ofPushStyle();
	shapesMat.begin();
	for(int i = 0; i < shapes.size(); i++) {
		shapes[i]->draw();

	}
	shapesMat.end();
	ofPopStyle();

	light.disable();
	ofDisableLighting();

	camera.end();
	glDisable(GL_DEPTH_TEST);

	int totalShapes = shapes.size() + logos.size();
	ofVec3f gravity = world.getGravity();
	stringstream ss;
	ss << "Draw Debug (d): " << bDrawDebug << endl;
	ss << "Total Shapes: " << totalShapes << endl;
	ss << "Add logos(o)" << endl;
	ss << "add spherers (s)" << endl;
	ss << "add boxes (b)" << endl;
	ss << "Gravity(up/down/left/right): x=" << gravity.x << " y= " << gravity.y << " z= " << gravity.z << endl;
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(ss.str().c_str(), 20, 20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key) {
	ofVec3f mouseLoc = camera.screenToWorld( ofVec3f((float)ofGetMouseX(), (float)ofGetMouseY(), 0) );
	ofVec3f gravity = world.getGravity();
	ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
	float rsize = ofRandom(.3, 1.8);
	mouseLoc.z += 15;
	switch (key) {
		case 'd':
			bDrawDebug = !bDrawDebug;
			break;
		case 'o':
			logos.push_back( new ofxBulletCustomShape() );
			logos[logos.size()-1]->init( (btCompoundShape*)logos[0]->getCollisionShape(), logos[0]->getCentroid() );
			logos[logos.size()-1]->create( world.world, mouseLoc, startRot, 3. );
			logos[logos.size()-1]->add();
			break;
		case 's':
			shapes.push_back( new ofxBulletSphere() );
			((ofxBulletSphere*)shapes[shapes.size()-1])->create( world.world, mouseLoc, rsize*.2, rsize );
			((ofxBulletSphere*)shapes[shapes.size()-1])->setSphereResolution( 16 );
			((ofxBulletSphere*)shapes[shapes.size()-1])->setActivationState( DISABLE_DEACTIVATION );
			shapes[shapes.size()-1]->add();
			break;
		case 'b':
			shapes.push_back( new ofxBulletBox() );
			((ofxBulletBox*)shapes[shapes.size()-1])->create( world.world, mouseLoc, rsize*.2, rsize*2, rsize*2, rsize*2 );
			((ofxBulletBox*)shapes[shapes.size()-1])->setActivationState( DISABLE_DEACTIVATION );
			shapes[shapes.size()-1]->add();
			break;
		case ' ':
			bDropBox = true;
			break;
		case OF_KEY_UP:
			gravity.y -= 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_DOWN:
			gravity.y += 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_RIGHT:
			gravity.x += 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_LEFT:
			gravity.x -= 5.;
			world.setGravity( gravity );
			break;
		default:
			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo) {

}
