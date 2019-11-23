#include "ofApp.h"

void ofApp::setup() {

	shader.load("shader/shader");
	ofSetVerticalSync(false);

	deferred.init();

	auto bg = deferred.createPass<ofxDeferred::BgPass>();
	bg->begin();
	ofClear(239, 244, 255, 255);
	bg->end();

	auto e = deferred.createPass<ofxDeferred::EdgePass>();
	e->setEdgeColor(ofFloatColor(0.6));
	e->setUseReadColor(true);

	auto s = deferred.createPass<ofxDeferred::SsaoPass>();
	s->setOcculusionRadius(5.);
	s->setDarkness(.5);

	sh = deferred.createPass<ofxDeferred::ShadowLightPass>();
	sh->setPosition(glm::vec3(300, 500, - 400));
	sh->lookAt(glm::vec3(0));
	sh->setAmbientColor(ofFloatColor(0.86, 0.86, 0.83));
	sh->setDiffuseColor(ofFloatColor(0.95));
	sh->setDarkness(.6);

	auto d = deferred.createPass<ofxDeferred::DofPass>();
	d->setEndPointsCoC(glm::vec2(0.05, .3));
	d->setFoculRange(glm::vec2(0.04, .3));

	cam.setNearClip(0.);
	cam.setFarClip(3000.);

	rs.enableFlags(ofxRealSenseUtil::USE_DEPTH_MESH_POLYGON);
	rs.start();

	panel.setup();
	panel.add(rs.getParameters());

}

void ofApp::update() {
	rs.update();
}

void ofApp::draw() {
	
	auto drawFunc = [&](float lds, bool isShadow) {
		shader.begin();
		shader.setUniform1f("lds", lds);
		shader.setUniform1f("isShadow", isShadow ? 1 : 0);
		
		ofPushMatrix();
		ofScale(500.);
		ofTranslate(0, 0, 1.);
		rs.getPolygonMesh().draw();
		ofPopMatrix();

		shader.end();
	};


	sh->beginShadowMap(cam, true);
	drawFunc(sh->getLinearScalar(), true);
	sh->endShadowMap();

	deferred.begin(cam, true);
	drawFunc(1. / (cam.getFarClip() - cam.getNearClip()), false);
	deferred.end();

	//sh->debugDraw();
	panel.draw();
	//ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), 10, 20);
}

void ofApp::exit() {
	rs.stop();
}

