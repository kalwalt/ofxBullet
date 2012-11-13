#pragma once

#include "ofMain.h"
#include "ofxBullet.h"
#include "ofxAssimpModelLoader.h"
#include <set>

class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void getVerticesInsidePlanes(const btAlignedObjectArray<btVector3>& planes, btAlignedObjectArray<btVector3>& verticesOut, std::set<int>& planeIndicesOut);
	void voronoiBBShatter(const btAlignedObjectArray<btVector3>& points, const btVector3& bbmin, const btVector3& bbmax, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity);
	void voronoiConvexHullShatter(const btAlignedObjectArray<btVector3>& points, const btAlignedObjectArray<btVector3>& verts, const btQuaternion& bbq, const btVector3& bbt, btScalar matDensity);
    void createVoro();

	ofxBulletWorldRigid			world;
	vector <ofxBulletBox*>		bounds;
	ofxBulletCustomShape*		boundsShape;
	ofMaterial					boundsMat;
	float						boundsWidth;
	bool bDropBox;

	vector<ofxBulletCustomShape*>	logos;
	ofMaterial						logoMat;
	vector<ofxBulletCustomShape*>	shards;
	vector<ofxBulletBaseShape*>		shapes;
	ofMaterial						shapesMat;

	bool bDrawDebug;

    vector<ofMesh*>              shardsMeshes;
	//ofMesh						mesh;
	//ofEasyCam					camera;
	ofCamera					camera;
	ofLight						light;

	ofxAssimpModelLoader		assimpModel;

};
