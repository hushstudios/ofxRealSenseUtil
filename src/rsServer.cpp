#include "rsServer.h"

using namespace ofxRealSenseUtil;

Server::Server(const std::string& name) : bPlaying(false), bNewFrame(false) {
	uid = name; 

	rsParams.setName("Realsense " + name);
	rsParams.add(filters.getParameters());
	depthMeshParams.setName("depthMeshParams");
	depthMeshParams.add(useColorTexture.set("useColorTexture", true));
	depthMeshParams.add(useDepthTexture.set("useDepthTexture", false));
	depthMeshParams.add(usePointCloud.set("usePointCloud", false));
	depthMeshParams.add(usePolygonMesh.set("usePolygonMesh", true));
	depthMeshParams.add(depthPixelSize.set("pixelSize", 10, 1, 100));
	depthMeshParams.add(isClip.set("enableClip", false));
	depthMeshParams.add(p0.set("clip_p0", glm::vec2(0), glm::vec2(0), glm::vec2(640, 480)));
	depthMeshParams.add(p1.set("clip_p1", glm::vec2(1280, 720), glm::vec2(0), glm::vec2(1280, 720)));
	depthMeshParams.add(z_bounds.set("z_bounds", glm::vec2(0, 3), glm::vec2(0), glm::vec2(4, 4)));
	depthMeshParams.add(color_range.set("color_range", glm::vec2(0, 3), glm::vec2(0, 0), glm::vec2(4, 4)));
	depthMeshParams.add(one_color.set("one_color", false));

	transforms.setName("Transforms & Offsets");
	transforms.add(offset.set("offset", glm::vec3(0), glm::vec3(-5, -5, -5), glm::vec3(5, 5, 5)));
	transforms.add(theta.set("theta", glm::vec3(0), glm::vec3(-PI, -PI, -PI), glm::vec3(PI, PI, PI)));

	rsParams.add(depthMeshParams);
	rsParams.add(transforms); 
}

Server::~Server() {}

void Server::start() {
	if (!bOpen) {
		ofLogError(__FUNCTION__) << "Not opened yet.";
		return;
	}
	request = std::make_shared<ofThreadChannel<bool>>();
	response = std::make_shared<ofThreadChannel<FrameData>>();

	try {
		device = pipe->start(config).get_device();
		ofLogNotice(__FUNCTION__) << "start: " << device.get_info(RS2_CAMERA_INFO_NAME);
	} catch (const rs2::error& e) {
		ofLogError(__FUNCTION__) << e.what();
	}

	startThread();
	bPlaying = true;
}

void Server::stop() {
	bPlaying = false;
	request->close();
	response->close();
	
	waitForThread(true);
	
	pipe->stop();

	ofLogNotice(__FUNCTION__) << "stop: " << device.get_info(RS2_CAMERA_INFO_NAME);
}

void Server::update() {



	if (bPlaying) {
		bool r = true;
		request->send(r);
	}
	
	bNewFrame = false;

	while (response->tryReceive(fd)) {
		bNewFrame = true;
	}

	if (bNewFrame) {
		if (useColorTexture) {
			if (!colorTex.isAllocated()) {
				colorTex.allocate(fd.colorPix.getWidth(), fd.colorPix.getHeight(), GL_RGB8);
			}
			colorTex.loadData(fd.colorPix);
		} 
		if (useDepthTexture) {
			if (!depthTex.isAllocated()) {
				depthTex.allocate(fd.depthPix.getWidth(), fd.depthPix.getHeight(), GL_RGB32F);
			}
			depthTex.loadData(fd.depthPix);
		}
		if (usePointCloud) {
			meshPointCloud.clear();
			meshPointCloud = std::move(fd.meshPointCloud);
		}
		if (usePolygonMesh) meshPolygon = std::move(fd.meshPolygon);
		
	}

}

void Server::threadedFunction() {


	bool r = true;
	while (request->receive(r)) {

		FrameData newFd;
		
		rs2::frameset frames;
		if (!pipe->poll_for_frames(&frames)) continue;
		
		auto& depth = frames.get_depth_frame();
		auto& color = frames.get_color_frame();
		

		pc.map_to(color);
		filters.filter(depth);

		if (useColorTexture) {
			newFd.colorPix.setFromPixels(
				(unsigned char*)color.get_data(),
				color.get_width(), color.get_height(), OF_IMAGE_COLOR
			);
		}

		glm::ivec2 depthRes(depth.get_width(), depth.get_height());
		
		auto& points = pc.calculate(depth);


		if (useDepthTexture) {
			newFd.depthPix.setFromPixels(
				(float*)points.get_vertices(),
				depthRes.x, depthRes.y, OF_IMAGE_COLOR
			);
		}
		
		if (usePointCloud) {
			createPointCloud(newFd.meshPointCloud, points, depthRes, depthPixelSize.get());
		}
		if (usePolygonMesh) {
			createMesh(newFd.meshPolygon, points, depthRes, depthPixelSize.get());
		}

		response->send(std::move(newFd));
		
	}

}

void Server::createPointCloud(ofMesh& mesh, const rs2::points& ps, const glm::ivec2& res, int pixelSize) {

	if (!ps) return;

	mesh.clear();
	
	const rs2::vertex * vs = ps.get_vertices();
	const rs2::texture_coordinate * texCoords = ps.get_texture_coordinates();

	

	glm::ivec2 start(0, 0), end(res);
	if (isClip) {
		start = glm::max(glm::ivec2(p0.get()), glm::ivec2(0));
		end = glm::min(glm::ivec2(p1.get()), res);
	}
	
	
	rs2::vertex* td = const_cast<rs2::vertex*>(vs);

	for (int y = start.y + pixelSize; y < end.y; y += pixelSize) {
		for (int x = start.x + pixelSize; x < end.x; x += pixelSize) {
			int i = y * res.x + x;

			//! offset
			td[i].x = td[i].x + offset.get().x;
			td[i].y = td[i].y + offset.get().y;
			td[i].z = td[i].z + offset.get().z;
			
			//! rotate
			glm::vec3 v(td[i].x, td[i].y, td[i].z);
			v = rotateXAxis(theta.get().x, v);
			v = rotateYAxis(theta.get().y, v);
			v = rotateZAxis(theta.get().z, v);
			
			const auto& uv = texCoords[i];

			if (!v.z) continue;

			if (v.z > z_bounds.get().x && v.z < z_bounds.get().y) {
				mesh.addVertex(glm::vec3(v.x, v.y, v.z));
				mesh.addTexCoord(glm::vec2(uv.u, uv.v));
				
				if (one_color.get()) {
					if (uid == "0") {
						mesh.addColor(ofColor(255, 0, 0));
					}
					else if (uid == "1") {
						mesh.addColor(ofColor(0, 255, 0));
					}
					else if (uid == "2") {
						mesh.addColor(ofColor(0, 0, 255));
					}
					else {
						mesh.addColor(ofColor(ofMap(v.z, color_range.get().x, color_range.get().y, 255, 0)));
					}
				}
				else {
					mesh.addColor(ofColor(ofMap(v.z, color_range.get().x, color_range.get().y, 255, 0)));
				}
			}
		}
	}

}

glm::vec3 Server::rotateXAxis(float theta, glm::vec3 v) {

	//transfer back to the origin 
	glm::vec3 origin; 
	origin = v - offset.get(); 

	glm::mat3 mat;
	mat[0][0] = 1;
	mat[1][0] = 0;
	mat[2][0] = 0;
	mat[0][1] = 0;
	mat[1][1] = cos(theta);
	mat[2][1] = sin(theta);
	mat[0][2] = 0;
	mat[1][2] = -sin(theta);
	mat[2][2] = cos(theta);

	glm::vec3 rot = origin * mat; 
	glm::vec3 orig = rot + offset.get(); 

	return (orig);
}

glm::vec3 Server::rotateYAxis(float theta, glm::vec3 v) {
	//transfer back to the origin 
	glm::vec3 origin;
	origin = v - offset.get();

	glm::mat3 mat;
	mat[0][0] = cos(theta);
	mat[1][0] = -sin(theta);
	mat[2][0] = 0;
	mat[0][1] = sin(theta);
	mat[1][1] = cos(theta);
	mat[2][1] = 0;
	mat[0][2] = 0;
	mat[1][2] = 0;
	mat[2][2] = 1;

	glm::vec3 rot = origin * mat;
	glm::vec3 orig = rot + offset.get();

	return (orig);
}

glm::vec3 Server::rotateZAxis(float theta, glm::vec3 v) {
	//transfer back to the origin 
	glm::vec3 origin;
	origin = v - offset.get();

	glm::mat3 mat;
	mat[0][0] = cos(theta);
	mat[1][0] = 0;
	mat[2][0] = -sin(theta);
	mat[0][1] = 0;
	mat[1][1] = 1;
	mat[2][1] = 0;
	mat[0][2] = sin(theta);
	mat[1][2] = 0;
	mat[2][2] = cos(theta);

	glm::vec3 rot = origin * mat;
	glm::vec3 orig = rot + offset.get();

	return (orig);

}

void Server::createMesh(ofMesh& mesh, const rs2::points& ps, const glm::ivec2& res, int pixelSize) {

	if (!ps) return;
	
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);

	const rs2::vertex * vs = ps.get_vertices();
	const rs2::texture_coordinate * texCoords = ps.get_texture_coordinates();

	glm::ivec2 start(0, 0), end(res);
	if (isClip) {
		start = glm::max(glm::ivec2(p0.get()), glm::ivec2(0));
		end = glm::min(glm::ivec2(p1.get()), res);
	}
	const glm::ivec2 numTexel = glm::abs(start - end) / pixelSize;

	// list of index of depth map(x-y) - vNum
	std::unordered_map<int, int> vMap;

	int indexCount = -1;
	for (int y = start.y + pixelSize; y < end.y - pixelSize; y += pixelSize) {
		for (int x = start.x + pixelSize; x < end.x - pixelSize; x += pixelSize) {

			int index[4] = {
				y * res.x + x,
				y * res.x + (x + pixelSize),
				(y + pixelSize) * res.x + x,
				(y + pixelSize) * res.x + (x + pixelSize)
			};
			glm::vec3 pos[4];
			glm::vec2 uv[4];
			int eraseCount = 0;
			bool eraseFlag[4]{ false, false, false, false };

			for (int i = 0; i < 4; i++) {
				const auto& v = vs[index[i]];
				const auto& t = texCoords[index[i]];
				if (v.z) {
					pos[i] = glm::vec3(v.x, -v.y, -v.z);
					uv[i] = glm::vec2(t.u, t.v);
				} else {
					eraseFlag[i] = true;
					eraseCount++;
				}
			}

			// try to check if possible to make square
			if (eraseCount >= 2) continue;
			else if (eraseCount == 1) {
				for (int i = 0; i < 4; i++) {
					if (!eraseFlag[i]) {
						// avoid double count
						if (vMap.count(index[i]) == 0) {
							vMap[index[i]] = ++indexCount;
							mesh.addVertex(pos[i]);
							mesh.addTexCoord(uv[i]);
						}
						mesh.addIndex(vMap[index[i]]);
					}
				}
			}
			else if (eraseCount == 0) {
				for (int i = 0; i < 4; i++) {
					if (vMap.count(index[i]) == 0) {
						vMap[index[i]] = ++indexCount;
						mesh.addVertex(pos[i]);
						mesh.addTexCoord(uv[i]);
					}
				}
				mesh.addIndex(vMap[index[0]]);
				mesh.addIndex(vMap[index[1]]);
				mesh.addIndex(vMap[index[2]]);

				mesh.addIndex(vMap[index[2]]);
				mesh.addIndex(vMap[index[1]]);
				mesh.addIndex(vMap[index[3]]);
			}
		}
	}
	float endTime = ofGetElapsedTimef();
}

const ofTexture& Server::getColorTex() const {
	if (!useColorTexture) {
		ofLogError(__FUNCTION__) << "Target flag is disabled!";
	}
	return colorTex;
}
const ofTexture& Server::getDepthTex() const {
	if (!useDepthTexture) {
		ofLogError(__FUNCTION__) << "Target flag is disabled!";
	}
	return depthTex;
}
const ofVboMesh& Server::getPointCloud() const {
	if (!usePointCloud) {
		ofLogError(__FUNCTION__) << "Target flag is disabled!";
	}
	return meshPointCloud;
}
const ofVboMesh& Server::getPolygonMesh() const {
	if (!usePolygonMesh) {
		ofLogError(__FUNCTION__) << "Target flag is disabled!";
	}
	return meshPolygon;
}