#include "rendering.h"
#include "teapot.h"

using namespace std;

Renderer::Renderer(int w, int h, const char* title) {
	videoWidth  = w;
	videoHeight = h;
	caption = title;

	preview = true;
	spin = 0.0;
	requestedFrameNumber = 0;
	freeview = true;
	drawWorldcoordinates = ALL_WORLDCOORDINATES;

	//object positioning
	objectMovement = false;
	objectMoveLeft = false;
	objectMoveRight = false;
	objectMoveUp = false;
	objectMoveDown = false;
	objectMoveForward = false;
	objectMoveBackward = false;
	objectEnlarge = false;
	objectShrink = false;

	//light positioning
	lightMovement = false;
	lightMoveLeft = false;
	lightMoveRight = false;
	lightMoveUp = false;
	lightMoveDown = false;
	lightMoveForward = false;
	lightMoveBackward = false;

	//Keep in mind: reverse y- and z-axis as the light is in the OpenGL coordinate system, but the camera isn't
	lightPosition[0] = 0.0;
	lightPosition[1] = 0.0;
	lightPosition[2] = 0.0;
	lightPosition[3] = 1.0;
	renderShadow = true;

	//free view camera
	cameraMoveLeft = false;
	cameraMoveRight = false;
	cameraMoveUp = false;
	cameraMoveDown = false;
	cameraMoveForward = false;
	cameraMoveBackward = false;
	camera = new Camera(); //x, y, z, pitch, yaw, roll
	speedFactor = 3.0;

	//mouse movement
	mouseSensitivity = 0.05;
	mouseLeftButtonDown = false;
	mouseRightButtonDown = false;
	mouseMiddleButtonDown = false;
	mouseDragging = false;

	//arcball rotation
	arcball = new Arcball();

	initRendering();
}

Renderer::~Renderer(){ }


bool Renderer::initRendering(void) {
	//init SDL
	if(SDL_Init(SDL_INIT_VIDEO) != 0) {
		cout << "Error initializing SDL: " << SDL_GetError() << endl;
		return false;
	}

	const SDL_VideoInfo* videoInfo = SDL_GetVideoInfo();
	windowWidth = videoInfo->current_w*0.8; //current screen resolution
	windowHeight = videoInfo->current_h*0.8;
	arcball->setArcballSize(windowWidth, windowHeight);

	//activate stencil buffer
	SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

	//SDL_SetVideoMode(windowWidth, windowHeight, 32, SDL_OPENGL | SDL_FULLSCREEN|SDL_NOFRAME);
	if(SDL_SetVideoMode(windowWidth, windowHeight, 32, SDL_OPENGL | SDL_RESIZABLE) == NULL) { //create window
		cerr << "Error creating SDL window!" << endl;
		return false;
	}

	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_WM_SetCaption(caption, NULL);

	//Antialiasing (SDL multisampling)
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4); //16x

	//init OpenGL
	glEnable(GL_MULTISAMPLE); //antialising
	glPolygonOffset(-2.0, -1.0); //prevent depth buffer flickering

	glEnable(GL_DEPTH_TEST); //Z-buffer
	glEnable(GL_NORMALIZE); //normalize normals automatically

	//Quality of implementation-specific hints
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); //quality of color, texture coordinate, and fog coordinate interpolation
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST); //sampling quality of antialiased lines
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); //sampling quality of antialiased points
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);  //sampling quality of antialiased polygons.
 	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	glShadeModel(GL_SMOOTH); //enable smooth shading (opposed to flat shading)
	glClearColor(0.0, 0.0, 0.0, 0.0); //background color of the scene

	//Light
	GLfloat ambientLight[] = {0.35, 0.31, 0.17}; //define light properties
	//GLfloat diffuseLight[] = {0.6, 0.6, 0.6}; //white
	//GLfloat specularLight[] = {0.9, 0.9, 0.9}; //white
	GLfloat diffuseLight[] = {0.69, 0.66, 0.6};
	GLfloat specularLight[] = {1.0, 0.94, 0.81};


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight); //load light properties
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	glEnable(GL_LIGHTING);			//enable OpenGL lighting
	glEnable(GL_LIGHT0);			//enable light source

	//Material
	GLfloat ambientMaterial[] = {0.4, 0.35, 0.32, 1.0};
	//	GLfloat diffuseMaterial[] = {0.4, 0.4, 0.4, 1.0}; //white
	//	GLfloat specularMaterial[] = {0.8, 0.8, 0.8, 1.0}; //white
	GLfloat diffuseMaterial[] = {0.39, 0.37, 0.28, 1.0};
	GLfloat specularMaterial[] = {0.98, 0.85, 0.72, 1.0};
	GLfloat materialShininess = 20.0;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientMaterial);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseMaterial);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specularMaterial);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, materialShininess);

	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE); //let vertex color override material properties
	glEnable(GL_COLOR_MATERIAL);

	return true;
}


void Renderer::setViewFrustum(cv::Mat& K) {
	glViewport(0, 0, windowWidth, windowHeight);
	glMatrixMode(GL_PROJECTION); //select projection matrix stack
	glLoadIdentity(); //reset projection matrix

	//create new projection matrix
	double f_x = K.at<double>(0, 0); //focal length
	double f_y = K.at<double>(1, 1);

	double c_x = K.at<double>(0, 2); //principal point
	double c_y = K.at<double>(1, 2);

	double w = static_cast<double>(videoWidth);
	double h = static_cast<double>(videoHeight);
	double aspectRatio = (w / h) * (f_y / f_x);

	double fovy = 2*atan(h / (2.0 * f_y)); //vertical field of view (Radiant)

	double nearPlane = 0.1;
	double farPlane = 1000.0;

	double viewFrustumHeight = tan(fovy/2.0) * nearPlane;
	double viewFrustumWidth = aspectRatio * viewFrustumHeight;

	offset_x = (w / 2 - c_x) / w * viewFrustumWidth * 2; //shifted principal point
	offset_y = (h / 2 - c_y) / h * viewFrustumHeight * 2;

	//define view frustum i.e. the projection matrix
	//note: the frustum is asymmetric, as the principal points are shifted
	glFrustum( -viewFrustumWidth  - offset_x, //left
				viewFrustumWidth  - offset_x, //right
			   -viewFrustumHeight - offset_y, //bottom
				viewFrustumHeight - offset_y, //top
				nearPlane, farPlane);

	//easier, but doesn't consider shifted principal points
	//gluPerspective(fovy, aspectRatio, 0.1, 1000.0); //frustum: fovy, aspect ratio, zNear, zFar
}


void Renderer::setScaleFactor(float scale) {
	scaleFactor = scale;
	objectScalefactor = scale * 8.0;
}


void Renderer::setObjectPosition(cv::Point3f& position, std::vector<Points3D>& coordinates) {
	objectPosition = position;
	worldCoordinates = coordinates;

	//select color for triangulated world coordinates
	getRandomColors(worldCoordinates.size(), worldCoordinateColors);

	//reverse y- and z-axis as the light is in the OpenGL coordinate system, but the camera isn't ;-)
	lightPosition[0] = objectPosition.x;
	lightPosition[1] = objectPosition.y - fabs(objectPosition.z/2); //up direction -> negative values
	lightPosition[2] = objectPosition.z - fabs(objectPosition.z/2); //out of the screen -> negative values
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}


void Renderer::setCameraTrack(std::vector<cv::Mat>& r, std::vector<cv::Mat>& t) {
	rotations = r;
	translations = t;
}


void Renderer::setCameraPose(cv::Mat& R, cv::Mat& t) {
	//construct modelview matrix from rotation and translation (reversed y and z axis from OpenCV to OpenGL)
	// Left  Up  Fwd  Translation
	//   |   |   |    |
	//   v   v   v    v
	//   R   R   R |  t
	//  -R  -R  -R | -t
	//  -R  -R  -R | -t
	//   0   0   0 |  1

	//column major order:
	//first column
	modelViewMatrixRecoveredCamera[0] =  R.at<double>(0,0);
	modelViewMatrixRecoveredCamera[1] = -R.at<double>(1,0);
	modelViewMatrixRecoveredCamera[2] = -R.at<double>(2,0);
	modelViewMatrixRecoveredCamera[3] =  0.0;
	//second column
	modelViewMatrixRecoveredCamera[4] =  R.at<double>(0,1);
	modelViewMatrixRecoveredCamera[5] = -R.at<double>(1,1);
	modelViewMatrixRecoveredCamera[6] = -R.at<double>(2,1);
	modelViewMatrixRecoveredCamera[7] =  0.0;
	//third column
	modelViewMatrixRecoveredCamera[8] =   R.at<double>(0,2);
	modelViewMatrixRecoveredCamera[9] =  -R.at<double>(1,2);
	modelViewMatrixRecoveredCamera[10] = -R.at<double>(2,2);
	modelViewMatrixRecoveredCamera[11] =  0.0;
	//fourth column
	modelViewMatrixRecoveredCamera[12] =  t.at<double>(0,0);
	modelViewMatrixRecoveredCamera[13] = -t.at<double>(1,0);
	modelViewMatrixRecoveredCamera[14] = -t.at<double>(2,0);
	modelViewMatrixRecoveredCamera[15] =  1.0;

}


void Renderer::setFreeViewCameraPose(cv::Mat& R, cv::Mat& t, cv::Mat& pos, float pitch, float yaw, float roll) {

	  glMatrixMode(GL_MODELVIEW); //select transformation matrix stack
	  glLoadIdentity(); //reset transformation matrix

	  //construct modelview matrix from rotation and translation (reversed y and z axis from OpenCV to OpenGL)
	  // Left  Up  Fwd  Translation
	  //   |   |   |    |
	  //   v   v   v    v
	  //   R   R   R |  t
	  //  -R  -R  -R | -t
	  //  -R  -R  -R | -t
	  //   0   0   0 |  1

	  //column major order:
	  //first column
	  modelViewMatrixFreeViewCamera[0] =  R.at<double>(0,0);
	  modelViewMatrixFreeViewCamera[1] = -R.at<double>(1,0);
	  modelViewMatrixFreeViewCamera[2] = -R.at<double>(2,0);
	  modelViewMatrixFreeViewCamera[3] =  0.0;
	  //second column
	  modelViewMatrixFreeViewCamera[4] =  R.at<double>(0,1);
	  modelViewMatrixFreeViewCamera[5] = -R.at<double>(1,1);
	  modelViewMatrixFreeViewCamera[6] = -R.at<double>(2,1);
	  modelViewMatrixFreeViewCamera[7] =  0.0;
	  //third column
	  modelViewMatrixFreeViewCamera[8] =   R.at<double>(0,2);
	  modelViewMatrixFreeViewCamera[9] =  -R.at<double>(1,2);
	  modelViewMatrixFreeViewCamera[10] = -R.at<double>(2,2);
	  modelViewMatrixFreeViewCamera[11] =  0.0;
	  //fourth column
	  modelViewMatrixFreeViewCamera[12] =  t.at<double>(0,0);
	  modelViewMatrixFreeViewCamera[13] = -t.at<double>(1,0);
	  modelViewMatrixFreeViewCamera[14] = -t.at<double>(2,0);
	  modelViewMatrixFreeViewCamera[15] =  1.0;

	 glLoadMatrixd(modelViewMatrixFreeViewCamera); //replace the current with the new modelview matrix

	 //update modelview matrix
	 camera->setPosition(pos);
	 camera->rotateX(pitch);
	 camera->rotateY(yaw);
	 camera->rotateZ(roll);

}


void Renderer::render(cv::Mat& background) {
	//reset color-, z- and stencil buffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glLoadMatrixd(modelViewMatrixFreeViewCamera); //free view camera model-view matrix is now in use

	if(preview) {
		processInput(); //handle SDL events
		updateCamera(); //changes model-view matrix of free view camera
		updateObject(); //react to teapot movement
		updateLight(); //react to light movement
	}

	/////////////////////////////////////
	////  draw scene for free view camera
	/////////////////////////////////////
	freeview = true;
	//draw tracked points as spinning cubes in worldspace
	for(uint w = 0; w < worldCoordinates.size(); w++) {
		Points3D& currentWorldCoords = worldCoordinates.at(w);
		glColor3f(worldCoordinateColors[w][0], worldCoordinateColors[w][1], worldCoordinateColors[w][2]);
		for(unsigned int i = 0; i < currentWorldCoords.size(); i++) {
			drawSpinningCube(currentWorldCoords[i], scaleFactor/2);
		}
	}
	drawLightSource(); //visualize position of light source
	drawGrid(0, 0, 0, scaleFactor); //draw ground plate
	drawCameraTrack(); //visualize camera position of keyframes
    drawTeapotWithShadow(); //draw the actual teapot and it's shadow (and the plane if in preview mode)
    freeview = false;

	//////////////////////////////////////////
	////  draw rendering preview (bottom left)
	//////////////////////////////////////////
	glPushMatrix(); //save model-view matrix
    glViewport(0, 0, videoWidth, videoHeight); //shrink viewport to video size
	//draw the background video frame
	setOrthographicProjection();
	//note: the following solution might be slower than using the background image as a texture and sticking it to a quad
	//but it is a one-liner...
	glDrawPixels(videoWidth, videoHeight, GL_BGR, GL_UNSIGNED_BYTE, background.data);
	resetPerspectiveProjection();
	glLoadMatrixd(modelViewMatrixRecoveredCamera); //replace the current modelview matrix
	updateLight(); //the OpenGL light position has to be transformed by the new model-view matrix
	if(preview)	{ //draw tracked points as spinning cubes in worldspace
		if(drawWorldcoordinates == ALL_WORLDCOORDINATES) {
			for(uint w = 0; w < worldCoordinates.size(); w++) {
				Points3D& currentWorldCoords = worldCoordinates.at(w);
				glColor3f(worldCoordinateColors[w][0], worldCoordinateColors[w][1], worldCoordinateColors[w][2]);
				for(unsigned int i = 0; i < currentWorldCoords.size(); i++) {
					drawSpinningCube(currentWorldCoords[i], scaleFactor/2);
				}
			}
		} else if(drawWorldcoordinates == CURRENT_WORLDCOORDINATES) {
			uint limit = (requestedFrameNumber < worldCoordinates.size()) ? requestedFrameNumber : worldCoordinates.size()-1;
			Points3D& currentWorldCoords = worldCoordinates.at(limit);
			glColor3f(worldCoordinateColors[limit][0], worldCoordinateColors[limit][1], worldCoordinateColors[limit][2]);
			for(unsigned int i = 0; i < currentWorldCoords.size(); i++) {
				drawSpinningCube(currentWorldCoords[i], scaleFactor/2);
			}
		} else if(drawWorldcoordinates == NO_WORLDCOORDINATES) {
			//noop
		}
	}
	drawTeapotWithShadow(); //draw the actual teapot and it's shadow (and the plane if in preview mode)

	glViewport(0, 0, windowWidth, windowHeight); //reset viewport
    glPopMatrix(); //reset model-view matrix


	//update screen (double buffering)
	SDL_GL_SwapBuffers();
}


void Renderer::processInput() {
	//process SDL message loop
	while(SDL_PollEvent(&event)) { //there is an SDL event to handle
		if(event.type == SDL_QUIT) {
			SDL_Quit();
			exit(0);
		}

		if(event.type == SDL_VIDEORESIZE ) { //resize the screen
			windowWidth = event.resize.w;
			windowHeight = event.resize.h;
			SDL_SetVideoMode(windowWidth, windowHeight, 32, SDL_OPENGL | SDL_RESIZABLE);
			arcball->setArcballSize(windowWidth, windowHeight);
		}

		//process user input for the rendered object's position
		if(preview) {
			if(event.type == SDL_KEYDOWN ) { //key pressed

				if(event.key.keysym.sym == SDLK_RETURN || event.key.keysym.sym == SDLK_KP_ENTER) { //accept current object position and start rendering frames
					preview = false;
				}

				if(event.key.keysym.sym == SDLK_x) { //frame selection (backward)
					requestedFrameNumber = (requestedFrameNumber-1 + translations.size()) % translations.size(); //don't get negative
				}
				if(event.key.keysym.sym == SDLK_c) { //frame selection (forward)
					requestedFrameNumber = (requestedFrameNumber+1) % translations.size();
				}
				if(event.key.keysym.sym == SDLK_v) { //drawing mode of worldcoordinates (disabled, current camera, all)
					drawWorldcoordinates = (drawWorldcoordinates+1) % 3;
				}

				if(event.key.keysym.sym == SDLK_LCTRL) { //keep left CTRL pressed to move object
					objectMovement = true;
				}

				if(event.key.keysym.sym == SDLK_LSHIFT) { //keep left SHIFT pressed to move light
					lightMovement = true;
				}


				if(objectMovement) { //object positioning
					if(event.key.keysym.sym == SDLK_LEFT) { //x-axis
						objectMoveLeft = true;
					}
					if(event.key.keysym.sym == SDLK_RIGHT) {
						objectMoveRight = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEUP) { //y-axis
						objectMoveUp = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEDOWN) {
						objectMoveDown = true;
					}
					if(event.key.keysym.sym == SDLK_UP) { //z-axis
						objectMoveForward = true;
					}
					if(event.key.keysym.sym == SDLK_DOWN) {
						objectMoveBackward = true;
					}
					if(event.key.keysym.sym == SDLK_KP_PLUS ) {
						objectEnlarge = true;
					}
					if(event.key.keysym.sym == SDLK_KP_MINUS  ) {
						objectShrink = true;
					}

				} else if(lightMovement) { //light positioning
					if(event.key.keysym.sym == SDLK_LEFT) { //x-axis
						lightMoveLeft = true;
					}
					if(event.key.keysym.sym == SDLK_RIGHT) {
						lightMoveRight = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEUP) { //y-axis
						lightMoveUp = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEDOWN) {
						lightMoveDown = true;
					}
					if(event.key.keysym.sym == SDLK_UP) { //z-axis
						lightMoveForward = true;
					}
					if(event.key.keysym.sym == SDLK_DOWN) {
						lightMoveBackward = true;
					}
				} else { //free view camera
					if(event.key.keysym.sym == SDLK_LEFT) { //x-axis
						cameraMoveLeft = true;
					}
					if(event.key.keysym.sym == SDLK_RIGHT) {
						cameraMoveRight = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEUP) { //y-axis
						cameraMoveUp = true;
					}
					if(event.key.keysym.sym == SDLK_PAGEDOWN) {
						cameraMoveDown = true;
					}
					if(event.key.keysym.sym == SDLK_UP) { //z-axis
						cameraMoveForward = true;
					}
					if(event.key.keysym.sym == SDLK_DOWN) {
						cameraMoveBackward = true;
					}
				}


			}


			if(event.type == SDL_KEYUP ) { //key released
				if(event.key.keysym.sym == SDLK_LCTRL) {
					objectMovement = false;
				}

				if(event.key.keysym.sym == SDLK_LSHIFT) {
					lightMovement = false;
				}

				//object positioning and free view camera
				if(event.key.keysym.sym == SDLK_LEFT) { //x-axis
					objectMoveLeft = false;
					cameraMoveLeft = false;
					lightMoveLeft = false;
				}
				if(event.key.keysym.sym == SDLK_RIGHT) {
					objectMoveRight = false;
					cameraMoveRight = false;
					lightMoveRight = false;
				}
				if(event.key.keysym.sym == SDLK_PAGEUP) { //y-axis
					objectMoveUp = false;
					cameraMoveUp = false;
					lightMoveUp = false;
				}
				if(event.key.keysym.sym == SDLK_PAGEDOWN) {
					objectMoveDown = false;
					cameraMoveDown = false;
					lightMoveDown = false;
				}
				if(event.key.keysym.sym == SDLK_UP) { //z-axis
					objectMoveForward = false;
					cameraMoveForward = false;
					lightMoveForward = false;
				}
				if(event.key.keysym.sym == SDLK_DOWN) {
					objectMoveBackward = false;
					cameraMoveBackward = false;
					lightMoveBackward = false;
				}
				if(event.key.keysym.sym == SDLK_KP_PLUS ) {
					objectEnlarge = false;
				}
				if(event.key.keysym.sym == SDLK_KP_MINUS  ) {
					objectShrink = false;
				}

			}

			if(event.type == SDL_MOUSEMOTION) { //current mouse position
				mousePosition.x = (float)event.motion.x;
				mousePosition.y = (float)event.motion.y;
			}

			if(event.type == SDL_MOUSEBUTTONDOWN) {
				mousePressed.x = event.button.x; //initialize the starting coordinates for further mouse movement
				mousePressed.y = event.button.y;
				if(event.button.button == SDL_BUTTON_LEFT) {
					mouseLeftButtonDown = true;
				}
				if(event.button.button == SDL_BUTTON_RIGHT) {
					mouseRightButtonDown = true;
				}
				if(event.button.button == SDL_BUTTON_MIDDLE) {
					mouseMiddleButtonDown = true;

				}
			}
			if(event.type == SDL_MOUSEBUTTONUP) {
				if(event.button.button == SDL_BUTTON_LEFT) {
					mouseLeftButtonDown = false;
				}
				if(event.button.button == SDL_BUTTON_RIGHT) {
					mouseRightButtonDown = false;
				}
				if(event.button.button == SDL_BUTTON_MIDDLE) {
					mouseMiddleButtonDown = false;
				}
			}

		}
	} //end of message loop

}


void Renderer::updateObject() {

	if(objectMovement) {
		rotateObject();
		float velocityX = 0;
		float velocityY = 0;
		float velocityZ = 0;
		float velocityScale = 0;

		if(objectMoveLeft) {
			velocityX -= scaleFactor*speedFactor;
		}
		if(objectMoveRight) {
			velocityX += scaleFactor;
		}
		if(objectMoveUp) {
			velocityY -= scaleFactor;
		}
		if(objectMoveDown) {
			velocityY += scaleFactor;
		}
		if(objectMoveForward) {
			velocityZ += scaleFactor;
		}
		if(objectMoveBackward) {
			velocityZ -= scaleFactor;
		}
		if(objectEnlarge) {
			velocityScale += scaleFactor*(2.0/speedFactor);
		}
		if(objectShrink) {
			velocityScale -= scaleFactor*(2.0/speedFactor);
		}

		objectPosition.x += velocityX;
		objectPosition.y += velocityY;
		objectPosition.z += velocityZ;
		objectScalefactor = fmax(0.0, objectScalefactor + velocityScale); //don't get negative

	}

	//place teapot:
	//find selected world point by setting up a ray from the eye plane through the frustum.
	//points are then tested for distance to the ray.
	if(mouseMiddleButtonDown) { //picking an object with the mouse
		double currentModelViewMatrix[16]; //we do already know this matrix, but it doesn't hurt if we create a local copy of it
		double projectionMatrix[16];
		int viewport[4];
		double X, Y, Z;

		cv::Mat rayPointNear, rayPointFar; //the two world coordinates define a ray through the view-frustum

		//get current opengl viewport and matrices
		glGetIntegerv(GL_VIEWPORT, viewport);
		glGetDoublev(GL_MODELVIEW_MATRIX, currentModelViewMatrix);
		glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);

		//determine world coordinates of given 2D mouse coordinate
		//first on the near plane with z = 0.0
		//then on the far plane with z = 1.0
		//Note: (0,0) is bottom left in OpenGL but top left in mouse-coordinates
		gluUnProject ((double) mousePressed.x, (double)(windowHeight - mousePressed.y), 0.0, currentModelViewMatrix, projectionMatrix, viewport, &X, &Y, &Z);
		rayPointNear = (cv::Mat_<double>(3,1) <<  X, Y, Z); //point on near plane

		gluUnProject ((double) mousePressed.x, (double)(windowHeight - mousePressed.y), 1.0, currentModelViewMatrix, projectionMatrix, viewport, &X, &Y, &Z);
		rayPointFar = (cv::Mat_<double>(3,1) <<  X, Y, Z); //point on far plane

		//determine ray from point on near plane to point on far plane
		cv::Mat nearToFarRay = cv::Mat_<double>(3,1);
		nearToFarRay = rayPointFar - rayPointNear;
		cv::normalize(nearToFarRay, nearToFarRay);

		//test all points in all sets of world coordinates for distance to the ray
		double minDistance = 1.0;
		int setSelection = 0;
		int coordinateSelection = 0;
		for(unsigned int i = 0; i < worldCoordinates.size(); i++) {
			for(unsigned int j = 0; j < worldCoordinates[i].size(); j++) {
				cv::Mat worldPoint;
				worldPoint = (cv::Mat_<double>(3,1) << (double)worldCoordinates[i][j].x, (double)worldCoordinates[i][j].y, (double)worldCoordinates[i][j].z);

				cv::Mat pointRay = worldPoint - rayPointNear; //ray from point on near plane to world point
				cv::normalize(pointRay, pointRay);

				cv::Mat diff = nearToFarRay - pointRay; //distance vector between the (endpoints of the) two rays
				double distance = cv::norm(diff, cv::NORM_L2); //euclidean distance

				if(distance < minDistance) {
					minDistance = distance;
					setSelection = i;
					coordinateSelection = j;
				}
			}
		}
		if(coordinateSelection >= 0) {
			objectPosition = worldCoordinates[setSelection][coordinateSelection];
		}

	}
}


void Renderer::updateLight() {
		float velocityX = 0;
		float velocityY = 0;
		float velocityZ = 0;

		if(lightMoveLeft) {
			velocityX -= scaleFactor*speedFactor;
		}
		if(lightMoveRight) {
			velocityX += scaleFactor*speedFactor;
		}
		if(lightMoveUp) {
			velocityY -= scaleFactor*speedFactor;
		}
		if(lightMoveDown) {
			velocityY += scaleFactor*speedFactor;
		}
		if(lightMoveForward) {
			velocityZ += scaleFactor*speedFactor;
		}
		if(lightMoveBackward) {
			velocityZ -= scaleFactor*speedFactor;
		}

		lightPosition[0] += velocityX;
		lightPosition[1] += velocityY;
		lightPosition[2] += velocityZ;
		glLightfv(GL_LIGHT0, GL_POSITION, lightPosition); //tell openGL about the new light position. Position is transformed by current modelview matrix
}


void Renderer::updateCamera() {

	if(!objectMovement) {
		rotateCamera();
	}

	//camera translation
	if(cameraMoveForward) {
		camera->moveZ(-scaleFactor*speedFactor);
	}
	if(cameraMoveBackward) {
		camera->moveZ(scaleFactor*speedFactor);
	}
	if(cameraMoveLeft) {
		camera->moveX(-scaleFactor*speedFactor);
	}
	if(cameraMoveRight) {
		camera->moveX(scaleFactor*speedFactor);
	}
	if(cameraMoveUp) {
		camera->moveY(-scaleFactor*speedFactor);
	}
	if(cameraMoveDown) {
		camera->moveY(scaleFactor*speedFactor);
	}

	//camera rotation
	float deltaY = camera->lastYrot - camera->yrot;
	float deltaX = camera->lastXrot - camera->xrot;
	if(deltaY != 0) {
		camera->rotateY(deltaY); //rotate left/right (around Up-Vector)
	}
	if(deltaX != 0) {
		camera->rotateX(deltaX); //rotate up/down (around Right-Vector)
	}
	camera->lastYrot = camera->yrot;
	camera->lastXrot = camera->xrot;

	//perform actual change
	camera->setup();
}


void Renderer::rotateCamera() {
	int deltaX, deltaY; //mouse position deltas
	deltaX = mousePosition.x - mousePressed.x;
	deltaY = mousePosition.y - mousePressed.y;
	mousePressed.x = mousePosition.x; //overwrite the coordinate with current x position
	mousePressed.y = mousePosition.y;

	//rotation (left mouse button)
	if(mouseLeftButtonDown) {
		camera->xrot -= (float)deltaY * mouseSensitivity;	// rotate upwards/downwards
		camera->yrot += (float)deltaX * mouseSensitivity;	//rotate left/right
	}

	//forward/backward (right mouse button)
	if(mouseRightButtonDown){
		if(deltaY < 0){ //forwards
			camera->moveZ(-scaleFactor*speedFactor);
		}
		else if(deltaY > 0){ //backwards
			camera->moveZ(scaleFactor*speedFactor);
		}
	}
}


void Renderer::rotateObject() {
    if (!mouseDragging) {

        if (mouseLeftButtonDown) { //first click - prepare for future movement
        	mouseDragging = true;
        	arcball->click(mousePressed);
        }
    } else {
        if (mouseLeftButtonDown) { //still holding the mouse button
        	arcball->drag(mousePosition);
        }
        else { //released the mouse button
            mouseDragging = false;
        }
    }
}


void Renderer::drawSpinningCube(cv::Point3f position, float scale) {
	glPushMatrix();
	glTranslatef(position.x, position.y, position.z); //translate cube
	glScalef(scale, scale, scale);	//scale cube
	glRotatef( fmod((position.x+position.y+position.z)*1000000.0f, 360.0f) + spin, 1, 1, 0);  //rotate cube individually around x and y axis (based on its position)
	spin = fmod(spin, 360.0f) + 0.1; //don't let it escape ;-)

	glBegin(GL_QUADS);
		//front
		//glColor3f(0.7, 0.0, 0.0); //red
		glNormal3f(0, 0, -1);
		glVertex3f( 1, 1, -1);
		glVertex3f( 1,-1, -1);
		glVertex3f(-1,-1, -1);
		glVertex3f(-1, 1, -1);

		//back
		//glColor3f(0, 1, 1); //aqua
		glNormal3f(0, 0, 1);
		glVertex3f( 1,  1,  1);
		glVertex3f(-1,  1,  1);
		glVertex3f(-1, -1,  1);
		glVertex3f( 1, -1,  1);

		//left
		//glColor3f(1, 0, 1); //pink
		glNormal3f(-1, 0, 0);
		glVertex3f(-1, -1, -1);
		glVertex3f(-1, -1,  1);
		glVertex3f(-1,  1,  1);
		glVertex3f(-1,  1, -1);

		//right
		//glColor3f(0, 0, 1); //blue
		glNormal3f(1, 0, 0);
		glVertex3f( 1,  1, -1);
		glVertex3f( 1,  1,  1);
		glVertex3f( 1, -1,  1);
		glVertex3f( 1, -1, -1);

		//top;
		//glColor3f(0, 1, 0);//green
		glNormal3f(0, -1, 0);
		glVertex3f( 1, -1, -1);
		glVertex3f( 1, -1,  1);
		glVertex3f(-1, -1,  1);
		glVertex3f(-1, -1, -1);

		//bottom
		//glColor3f(1, 1, 0);//yellow
		glNormal3f(0, 1, 0);
		glVertex3f( 1,  1,  1);
		glVertex3f( 1,  1, -1);
		glVertex3f(-1,  1, -1);
		glVertex3f(-1,  1,  1);
	glEnd();

	glPopMatrix();
}


void Renderer::drawCube(float scale) {
	glScalef(scale, scale, scale);	//scale cube
	glBegin(GL_QUADS);
		//front
		glColor3f(0.7, 0.0, 0.0); //red
		glNormal3f(0, 0, -1);
		glVertex3f( 1, 1, -1);
		glVertex3f( 1,-1, -1);
		glVertex3f(-1,-1, -1);
		glVertex3f(-1, 1, -1);

		//back
		glColor3f(0, 1, 1); //aqua
		glNormal3f(0, 0, 1);
		glVertex3f( 1,  1,  1);
		glVertex3f(-1,  1,  1);
		glVertex3f(-1, -1,  1);
		glVertex3f( 1, -1,  1);

		//left
		glColor3f(1, 0, 1); //pink
		glNormal3f(-1, 0, 0);
		glVertex3f(-1, -1, -1);
		glVertex3f(-1, -1,  1);
		glVertex3f(-1,  1,  1);
		glVertex3f(-1,  1, -1);

		//right
		glColor3f(0, 0, 1); //blue
		glNormal3f(1, 0, 0);
		glVertex3f( 1,  1, -1);
		glVertex3f( 1,  1,  1);
		glVertex3f( 1, -1,  1);
		glVertex3f( 1, -1, -1);

		//top
		glColor3f(1, 1, 0);//yellow
		glNormal3f(0, 1, 0);
		glVertex3f( 1,  1,  1);
		glVertex3f( 1,  1, -1);
		glVertex3f(-1,  1, -1);
		glVertex3f(-1,  1,  1);

		//bottom;
		glColor3f(0, 1, 0);//green
		glNormal3f(0, -1, 0);
		glVertex3f( 1, -1, -1);
		glVertex3f( 1, -1,  1);
		glVertex3f(-1, -1,  1);
		glVertex3f(-1, -1, -1);

	glEnd();

	glPopMatrix();
}


void Renderer::drawGrid(float x, float y, float z, float scale) {
	glDisable(GL_LIGHTING); //disable lighting for uniform grid color

	int nLinesX = 15;
	int nLinesZ = 15;
	float lowerX = -200*scale;
	float upperX =  200*scale;
	float lowerZ = 0;
	float upperZ =  400*scale;
	float Y = 50*scale;

	float stepX = fabs(lowerX - upperX) / (nLinesX-1);
	float stepZ = fabs(lowerZ - upperZ) / (nLinesZ-1);


	//transparent base plane
	glColor4f(1.0, 1.0, 1.0, 0.2);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glBegin(GL_QUADS);
		glVertex3f(lowerX, Y, lowerZ);
		glVertex3f(lowerX, Y, upperZ);
		glVertex3f(upperX, Y, upperZ);
		glVertex3f(upperX, Y, lowerZ);
	glEnd();


	//white lines
	glLineWidth(2.0);
	glBegin(GL_LINES);
	for(int z = 0; z < nLinesZ; z++) { //draw x-axis parallel lines
		glVertex3f(lowerX, Y, lowerZ + z*stepZ);
		glVertex3f(upperX, Y, lowerZ + z*stepZ);
	}
	for(int x = 0; x < nLinesX; x++) { //draw z-axis parallel lines
		glVertex3f(lowerX + x*stepX, Y, lowerZ);
		glVertex3f(lowerX + x*stepX, Y, upperZ);
	}
	glEnd();
	glDisable(GL_BLEND);

	glEnable(GL_LIGHTING);
}


void Renderer::setOrthographicProjection() {
	glDisable(GL_DEPTH_TEST); //not needed in 2d
	glMatrixMode(GL_PROJECTION); //select projection matrix stack
	glPushMatrix(); //save previous projection matrix
	glLoadIdentity();
	gluOrtho2D(0, videoWidth, videoHeight, 0);  //set ortho view
	glMatrixMode(GL_MODELVIEW);
}


void Renderer::resetPerspectiveProjection() {
	glMatrixMode(GL_PROJECTION);
	glPopMatrix(); //restore previous projective matrix
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
	glClear(GL_DEPTH_BUFFER_BIT);
}


void Renderer::drawLightSource() {
	int stacks = 16; //latitude
	int slices = 16; //longitude

	glDisable(GL_LIGHTING);
	glPushMatrix();
		glTranslatef(lightPosition[0], lightPosition[1], lightPosition[2]);
		glColor3f(1.0, 1.0, 0.0); //yellow
		GLUquadricObj *sphere;
		sphere = gluNewQuadric();
		gluSphere(sphere, 5*scaleFactor, slices, stacks); //sphere made of stacks and slices
		gluDeleteQuadric(sphere);
	glPopMatrix();
	glEnable(GL_LIGHTING);
}


void Renderer::drawCameraTrack() {
	glDisable(GL_LIGHTING); //turn off the light so we can use glColor()

	float length = 3.0*scaleFactor;

	//camera frustum (not to scale)
	float aspectRatio = videoWidth/videoHeight;
	float nearHeight = 1.0*scaleFactor;
	float nearWidth = 1.0*scaleFactor*aspectRatio;
	float farHeight = 2.0*scaleFactor;
	float farWidth = 2.0*scaleFactor*aspectRatio;

	for(unsigned int i = 0; i < rotations.size(); i++) { //for each camera of the track
		cv::Mat nearCenter = (cv::Mat_<float>(3,1) <<  -translations[i].at<double>(0,0), -translations[i].at<double>(1,0), translations[i].at<double>(2,0));
		cv::Mat farCenter = (cv::Mat_<float>(3,1) <<  -translations[i].at<double>(0,0)-rotations[i].at<double>(0,2)*length, -translations[i].at<double>(1,0)-rotations[i].at<double>(1,2)*length, translations[i].at<double>(2,0)+rotations[i].at<double>(2,2)*length);
		cv::Mat rightVector = (cv::Mat_<float>(3,1) <<  rotations[i].at<double>(0,0), -rotations[i].at<double>(1,0), -rotations[i].at<double>(2,0));
		cv::Mat upVector = (cv::Mat_<float>(3,1) <<  rotations[i].at<double>(0,1), -rotations[i].at<double>(1,1), rotations[i].at<double>(2,1));

		//compute the 4 corners of the frustum on the near plane
		cv::Mat nearTopLeft = nearCenter + upVector * nearHeight - rightVector * nearWidth;
		cv::Mat nearTopRight = nearCenter + upVector * nearHeight + rightVector * nearWidth;
		cv::Mat nearBottomLeft = nearCenter - upVector * nearHeight - rightVector * nearWidth;
		cv::Mat nearBottomRight = nearCenter - upVector * nearHeight + rightVector * nearWidth;

		//compute the 4 corners of the frustum on the far plane
		cv::Mat farTopLeft = farCenter + upVector * farHeight - rightVector * farWidth;
		cv::Mat farTopRight = farCenter + upVector * farHeight + rightVector * farWidth;
		cv::Mat farBottomLeft = farCenter - upVector * farHeight - rightVector * farWidth;
		cv::Mat farBottomRight = farCenter - upVector * farHeight + rightVector * farWidth;


		//camera frustum planes
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		if((unsigned int)requestedFrameNumber == i) {
				glColor4f(1.0, 1.0, 0.1, 0.5); //yellow
		} else {
			glColor4f(0.9, 0.9, 0.9, 0.5); //white
		}
		glBegin(GL_QUADS);
			//near plane
			glVertex3f(nearTopLeft.at<float>(0), nearTopLeft.at<float>(1), nearTopLeft.at<float>(2));
			glVertex3f(nearTopRight.at<float>(0), nearTopRight.at<float>(1), nearTopRight.at<float>(2));
			glVertex3f(nearBottomRight.at<float>(0), nearBottomRight.at<float>(1), nearBottomRight.at<float>(2));
			glVertex3f(nearBottomLeft.at<float>(0), nearBottomLeft.at<float>(1), nearBottomLeft.at<float>(2));

			//far plane
			glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));
			glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));
			glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));
			glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));

			//bottom plane
			glVertex3f(nearBottomLeft.at<float>(0),nearBottomLeft.at<float>(1),nearBottomLeft.at<float>(2));
			glVertex3f(nearBottomRight.at<float>(0),nearBottomRight.at<float>(1),nearBottomRight.at<float>(2));
			glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));
			glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));

			//top plane
			glVertex3f(nearTopRight.at<float>(0),nearTopRight.at<float>(1),nearTopRight.at<float>(2));
			glVertex3f(nearTopLeft.at<float>(0),nearTopLeft.at<float>(1),nearTopLeft.at<float>(2));
			glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));
			glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));

			//left plane
			glVertex3f(nearTopLeft.at<float>(0),nearTopLeft.at<float>(1),nearTopLeft.at<float>(2));
			glVertex3f(nearBottomLeft.at<float>(0),nearBottomLeft.at<float>(1),nearBottomLeft.at<float>(2));
			glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));
			glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));

			// right plane
			glVertex3f(nearBottomRight.at<float>(0),nearBottomRight.at<float>(1),nearBottomRight.at<float>(2));
			glVertex3f(nearTopRight.at<float>(0),nearTopRight.at<float>(1),nearTopRight.at<float>(2));
			glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));
			glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));
			glEnd();
		glDisable(GL_BLEND);


		//camera frustum outlines
		glLineWidth(1.0);
		glColor3f(0.5, 0.5, 0.5);

		//near plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(nearTopLeft.at<float>(0), nearTopLeft.at<float>(1), nearTopLeft.at<float>(2));
		glVertex3f(nearTopRight.at<float>(0), nearTopRight.at<float>(1), nearTopRight.at<float>(2));
		glVertex3f(nearBottomRight.at<float>(0), nearBottomRight.at<float>(1), nearBottomRight.at<float>(2));
		glVertex3f(nearBottomLeft.at<float>(0), nearBottomLeft.at<float>(1), nearBottomLeft.at<float>(2));
		glEnd();

		//far plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));
		glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));
		glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));
		glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));
		glEnd();

		//left plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(nearTopLeft.at<float>(0),nearTopLeft.at<float>(1),nearTopLeft.at<float>(2));
		glVertex3f(nearBottomLeft.at<float>(0),nearBottomLeft.at<float>(1),nearBottomLeft.at<float>(2));
		glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));
		glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));
		glEnd();

		// right plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(nearBottomRight.at<float>(0),nearBottomRight.at<float>(1),nearBottomRight.at<float>(2));
		glVertex3f(nearTopRight.at<float>(0),nearTopRight.at<float>(1),nearTopRight.at<float>(2));
		glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));
		glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));
		glEnd();

		//top plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(nearTopRight.at<float>(0),nearTopRight.at<float>(1),nearTopRight.at<float>(2));
		glVertex3f(nearTopLeft.at<float>(0),nearTopLeft.at<float>(1),nearTopLeft.at<float>(2));
		glVertex3f(farTopLeft.at<float>(0),farTopLeft.at<float>(1),farTopLeft.at<float>(2));
		glVertex3f(farTopRight.at<float>(0),farTopRight.at<float>(1),farTopRight.at<float>(2));
		glEnd();

		//bottom plane
		glBegin(GL_LINE_LOOP);
		glVertex3f(farBottomRight.at<float>(0),farBottomRight.at<float>(1),farBottomRight.at<float>(2));
		glVertex3f(farBottomLeft.at<float>(0),farBottomLeft.at<float>(1),farBottomLeft.at<float>(2));
		glVertex3f(nearBottomLeft.at<float>(0),nearBottomLeft.at<float>(1),nearBottomLeft.at<float>(2));
		glVertex3f(nearBottomRight.at<float>(0),nearBottomRight.at<float>(1),nearBottomRight.at<float>(2));
		glEnd();

	}

	glEnable(GL_LIGHTING);
}


void Renderer::defineShadowPlane() {
	//define the (invisible) floor for the shadow (in homogeneous coordinates)
	planeVertex0_h = (cv::Mat_<float>(4,1) << - 50*scaleFactor, 0,   50*scaleFactor, 1.0);
	planeVertex1_h = (cv::Mat_<float>(4,1) <<   50*scaleFactor, 0,   50*scaleFactor, 1.0);
	planeVertex2_h = (cv::Mat_<float>(4,1) <<   50*scaleFactor, 0, - 50*scaleFactor, 1.0);
	planeVertex3_h = (cv::Mat_<float>(4,1) << - 50*scaleFactor, 0, - 50*scaleFactor, 1.0);

	//multiply virtual floor with transformation matrix (the 3d-Ball transformation)
	planeVertex0_h = arcball->getTansformationMatrix() * planeVertex0_h;
	planeVertex1_h = arcball->getTansformationMatrix() * planeVertex1_h;
	planeVertex2_h = arcball->getTansformationMatrix() * planeVertex2_h;
	planeVertex3_h = arcball->getTansformationMatrix() * planeVertex3_h;

	//convert homogeneous to cartesian coordinates
	//TODO: write a separate function to do the homogeneous transformation and vice versa. the corresponding OpenCV function has bugs...
	planeVertex0 = cv::Mat_<float>(3,1);
	planeVertex1 = cv::Mat_<float>(3,1);
	planeVertex2 = cv::Mat_<float>(3,1);
	planeVertex3 = cv::Mat_<float>(3,1);

	float w = static_cast<float>(planeVertex0_h.at<float>(3,0));
	planeVertex0.at<float>(0,0) = static_cast<float>(planeVertex0_h.at<float>(0,0) / w); //x = x/w
	planeVertex0.at<float>(1,0) = static_cast<float>(planeVertex0_h.at<float>(1,0) / w); //y = y/w
	planeVertex0.at<float>(2,0) = static_cast<float>(planeVertex0_h.at<float>(2,0) / w); //z = z/w

	w = static_cast<float>(planeVertex1_h.at<float>(3,0));
	planeVertex1.at<float>(0,0) = static_cast<float>(planeVertex1_h.at<float>(0,0) / w); //x = x/w
	planeVertex1.at<float>(1,0) = static_cast<float>(planeVertex1_h.at<float>(1,0) / w); //y = y/w
	planeVertex1.at<float>(2,0) = static_cast<float>(planeVertex1_h.at<float>(2,0) / w); //z = z/w

	w = static_cast<float>(planeVertex2_h.at<float>(3,0));
	planeVertex2.at<float>(0,0) = static_cast<float>(planeVertex2_h.at<float>(0,0) / w); //x = x/w
	planeVertex2.at<float>(1,0) = static_cast<float>(planeVertex2_h.at<float>(1,0) / w); //y = y/w
	planeVertex2.at<float>(2,0) = static_cast<float>(planeVertex2_h.at<float>(2,0) / w); //z = z/w

	w = static_cast<float>(planeVertex3_h.at<float>(3,0));
	planeVertex3.at<float>(0,0) = static_cast<float>(planeVertex3_h.at<float>(0,0) / w); //x = x/w
	planeVertex3.at<float>(1,0) = static_cast<float>(planeVertex3_h.at<float>(1,0) / w); //y = y/w
	planeVertex3.at<float>(2,0) = static_cast<float>(planeVertex3_h.at<float>(2,0) / w); //z = z/w

	//now translate the transformed plane to its position
	cv::Mat translation = (cv::Mat_<float>(3,1) << objectPosition.x, objectPosition.y, objectPosition.z);

	planeVertex0 += translation;
	planeVertex1 += translation;
	planeVertex2 += translation;
	planeVertex3 += translation;
}


void Renderer::findShadowPlaneEquation(cv::Mat& v0, cv::Mat& v1, cv::Mat& v2, cv::Mat& floorPlane) {

	cv::Mat u = cv::Mat_<float>(3,1);
	cv::Mat v = cv::Mat_<float>(3,1);
	cv::Mat normal = cv::Mat_<float>(3,1);
	float d;
	cv::Mat result;

	u = v0 - v1;
	v = v2 - v1;

	normal = v.cross(u);
	cv::normalize(normal, normal);
	d = -(normal.dot(v1));

	floorPlane.at<float>(0) = normal.at<float>(0);
	floorPlane.at<float>(1) = normal.at<float>(1);
	floorPlane.at<float>(2) = normal.at<float>(2);
	floorPlane.at<float>(3) = d;
}


void Renderer::calculateShadowProjectionMatrix(cv::Mat& groundplane) {
	float angle;

	//dot product between light position and shadow plane normal describes the angle
	angle = groundplane.at<float>(0) * -lightPosition[0] +
			groundplane.at<float>(1) * -lightPosition[1] +
			groundplane.at<float>(2) * -lightPosition[2] +
			groundplane.at<float>(3) * -lightPosition[3];

	//first column (x-axis)
	shadowProjectionMatrix[0][0] = angle + lightPosition[0] * groundplane.at<float>(0);
	shadowProjectionMatrix[1][0] =   0.0 + lightPosition[0] * groundplane.at<float>(1);
	shadowProjectionMatrix[2][0] =   0.0 + lightPosition[0] * groundplane.at<float>(2);
	shadowProjectionMatrix[3][0] =   0.0 + lightPosition[0] * groundplane.at<float>(3);
	//second column (y-axis)
	shadowProjectionMatrix[0][1] =   0.0 + lightPosition[1] * groundplane.at<float>(0);
	shadowProjectionMatrix[1][1] = angle + lightPosition[1] * groundplane.at<float>(1);
	shadowProjectionMatrix[2][1] =   0.0 + lightPosition[1] * groundplane.at<float>(2);
	shadowProjectionMatrix[3][1] =   0.0 + lightPosition[1] * groundplane.at<float>(3);
	//third column (z-axis)
	shadowProjectionMatrix[0][2] =   0.0 + lightPosition[2] * groundplane.at<float>(0);
	shadowProjectionMatrix[1][2] =   0.0 + lightPosition[2] * groundplane.at<float>(1);
	shadowProjectionMatrix[2][2] = angle + lightPosition[2] * groundplane.at<float>(2);
	shadowProjectionMatrix[3][2] =   0.0 + lightPosition[2] * groundplane.at<float>(3);
	//fourth column
	shadowProjectionMatrix[0][3] =   0.0 + lightPosition[3] * groundplane.at<float>(0);
	shadowProjectionMatrix[1][3] =   0.0 + lightPosition[3] * groundplane.at<float>(1);
	shadowProjectionMatrix[2][3] =   0.0 + lightPosition[3] * groundplane.at<float>(2);
	shadowProjectionMatrix[3][3] = angle + lightPosition[3] * groundplane.at<float>(3);
}


void Renderer::drawShadowPlane(void) {
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if(preview && freeview) {
		glColor4f(0.5, 1.0, 0.5, 0.2); //visible
	} else {
		glColor4f(0.9, 0.9, 0.9, 0.0); //invisible
	}

	glDisable(GL_LIGHTING);
	glPushMatrix();
		glBegin(GL_QUADS);
			glVertex3f(planeVertex0.at<float>(0), planeVertex0.at<float>(1), planeVertex0.at<float>(2));
			glVertex3f(planeVertex1.at<float>(0), planeVertex1.at<float>(1), planeVertex1.at<float>(2));
			glVertex3f(planeVertex2.at<float>(0), planeVertex2.at<float>(1), planeVertex2.at<float>(2));
			glVertex3f(planeVertex3.at<float>(0), planeVertex3.at<float>(1), planeVertex3.at<float>(2));
		glEnd();
	glPopMatrix();

	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);
}


void Renderer::drawTeapotWithShadow() {

	//set up the plane where the teapot is placed on
	defineShadowPlane();
	//define floor plane to project shadow on
	cv::Mat planeEquation = cv::Mat_<float>(4,1);
	findShadowPlaneEquation(planeVertex0, planeVertex1, planeVertex2, planeEquation);

	//calculate shadow projection matrix
	calculateShadowProjectionMatrix(planeEquation);

	//draw Utah Teapot
	glPushMatrix();
		float s = objectScalefactor;
		float sy = 1.33333 * s; //distort teapot in y-direction (glutTeapot is only 0.75 * original height)
		//glColor4f(1.0, 1.0, 1.0, 1.0);
		glColor4f(0.98, 0.9, 0.81, 1.0);
		glTranslatef(objectPosition.x, objectPosition.y-s, objectPosition.z); //translate object
		glMultMatrixf(arcball->getTansformationMatrixGL());
		glScalef(s, -sy, s); //scale object
		glutSolidTeapot(1.0);
	glPopMatrix();

	glPushMatrix();
	if(renderShadow) {
		//enable the stencil buffer
		//the plane fragments, that are drawn next, will have a stencil buffer value of 3
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 3, 0xffffffff);
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
	}

	glPushMatrix();
		drawShadowPlane();
	glPopMatrix();


	if(renderShadow) {
		//render the projected teapot onto the plane where the stencil value is 3
		//set stencil buffer to 2 in order to prevent multiple projection to the same pixel
		glStencilFunc(GL_LESS, 2, 0xffffffff);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

		glEnable(GL_POLYGON_OFFSET_FILL); //prevent z-fighting of the shadow and the plane

		//render semi-transparent black shadow
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_LIGHTING);
		glColor4f(0.0, 0.0, 0.0, 0.5);

		//project the teapot shadow
		glPushMatrix();
			glMultMatrixf((float*) shadowProjectionMatrix);
			//draw Utah Teapot
			glPushMatrix();
				float s = objectScalefactor;
				float sy = 1.33333 * s; //distort teapot in y-direction (glutTeapot is only 0.75 * original height)
				glTranslatef(objectPosition.x, objectPosition.y-s, objectPosition.z); //the bottom of the teapot is 's' units below the center
				glMultMatrixf(arcball->getTansformationMatrixGL());
				glScalef(s, -sy, s); //scale object
				glutSolidTeapot(1.0);
			glPopMatrix();
		glPopMatrix();

		glDisable(GL_BLEND);
		glEnable(GL_LIGHTING);

		glDisable(GL_POLYGON_OFFSET_FILL); //disable polygon offset
		glDisable(GL_STENCIL_TEST);
	}

	//in selection mode: draw sphere around teapot
	if(objectMovement && freeview) {
		glPushMatrix();
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glTranslatef(objectPosition.x, objectPosition.y-s, objectPosition.z); //translate sphere to teapot center
			glColor4f(1.0, 1.0, 1.0, 0.3);
			GLUquadricObj *sphere;
			sphere = gluNewQuadric();
			gluSphere(sphere, 5*s, 32, 32); //radius of 5s, 32 slices and stacks
			gluDeleteQuadric(sphere);
		glPopMatrix();
	}

	glPopMatrix();

}


void Renderer::getRandomColors(uint n, std::vector<std::vector<float> >& colors) {

	int j;
	float p, q, t, f, fR, fG, fB;

	//distribute hues in golden ration
	float goldenRatio = 1.618033989;

	float h = generate_random_float(0.0, 6.0); //start value for hue

	//choose saturated and bright colors of all hues
	for(uint i = 0; i < n; i++) {
		std::vector<float> color;
		h += 1.0/goldenRatio;
		h = fmod(static_cast<float>(h), 6.0f);
		float s = generate_random_float(0.5, 1.0); //saturation
		float v = generate_random_float(0.5, 1.0); //value = brightness


		j = (int) h;
		f = h - j;
		p = v*(1. - s);
		q = v*(1. - s*f);
		t = v*(1. - s*(1-f));
		switch (j) {
		case 0: fR = v; fG = t; fB = p; break;
		case 1: fR = q; fG = v; fB = p; break;
		case 2: fR = p; fG = v; fB = t; break;
		case 3: fR = p; fG = q; fB = v; break;
		case 4: fR = t; fG = p; fB = v; break;
		case 5: fR = v; fG = p; fB = q; break;
		default: fR = 1.0, fG = 1.0, fB = 1.0; //white
		break;
		}

		color.push_back(fR); //r
		color.push_back(fG); //g
		color.push_back(fB); //b

		colors.push_back(color);
	}
}


float Renderer::generate_random_float(float min, float max) {
	static int first_call = 1;
	if (first_call == 1) { //make sure seed is set only once. This is ensured by declaring "first_call" static */
		srand((unsigned)time(NULL)); //seed
		first_call = 0;
	}

    if (min>max) {
        return (rand()/((float)(RAND_MAX+1.0)))*(min-max)+max;
    }
    else {
    	return (rand()/((float)(RAND_MAX+1.0)))*(max-min)+min;
    }
}

