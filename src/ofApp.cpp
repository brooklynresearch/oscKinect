#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    currentKinect = 0;
    // number of Kinect devices; used to create array
    nKinects = ofxKinect::numTotalDevices();
    ofxKinect::listDevices();
    
//    kinects = new ofxKinect*[nKinects];
//    // number of kinect objects to instantiate
//    for(int i = 0; i < nKinects; i++){
//        kinects[i] = new ofxKinect();
//        kinects[i]->init(false, false);
//        kinects[i]->open();
//    }
    
    
    // enable depth->video image calibration
//    kinect.setRegistration(true);
    
//    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    kinect.init(false, false); // disable video image (faster fps)      
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    
    
//    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 230;
    farThreshold = 70;
    bThreshWithOpenCV = true;
    
    blobMinSize = 20;
    blobMaxSize = 3000;
    maxBlobs = 5;
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
    
    
    // second kinect
    kinect2.init(false, false);
    kinect2.open();
    
    // print the intrinsic IR sensor values
    if(kinect2.isConnected()) {        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect2.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect2.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect2.getZeroPlaneDistance() << "mm";
    }

    
    grayImage2.allocate(kinect2.width, kinect2.height);
    grayThreshNear2.allocate(kinect2.width, kinect2.height);
    grayThreshFar2.allocate(kinect2.width, kinect2.height);
    
    nearThreshold2 = 230;
    farThreshold2 = 70;
    bThreshWithOpenCV2 = true;
    
    blobMinSize2 = 20;
    blobMaxSize2 = 3000;
    maxBlobs2 = 5;
    
    // zero the tilt on startup
    angle2 = 0;
    kinect2.setCameraTiltAngle(angle2);
    
    // start from the front
    bDrawPointCloud2 = false;
    
    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void ofApp::update(){
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage.getPixels();
            
            int numPixels = grayImage.getWidth() * grayImage.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, maxBlobs, blobMaxSize, blobMinSize, false);
    }
    
    kinect2.update();
    if(kinect2.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV2) {
            grayThreshNear2 = grayImage2;
            grayThreshFar2 = grayImage2;
            grayThreshNear2.threshold(nearThreshold2, true);
            grayThreshFar2.threshold(farThreshold2);
            cvAnd(grayThreshNear2.getCvImage(), grayThreshFar2.getCvImage(), grayImage2.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage2.getPixels();
            
            int numPixels = grayImage2.getWidth() * grayImage2.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold2 && pix[i] > farThreshold2) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage2.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder2.findContours(grayImage2, maxBlobs2, blobMaxSize2, blobMinSize2, false);
    }
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    ofSetColor(255, 255, 255);
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        // draw from the live kinect
        kinect.drawDepth(10, 10, 400, 300);
        //kinect.draw(420, 10, 400, 300);
        
        grayImage.draw(10, 320, 400, 300);
        contourFinder.draw(10, 320, 400, 300);
        
//#ifdef USE_TWO_KINECTS
        //kinect2.draw(420, 320, 400, 300);
        kinect2.drawDepth(420, 10, 400, 300);
        grayImage2.draw(420, 320, 400, 300);
        contourFinder2.draw(420, 320, 400, 300);
        
//#endif
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    
    reportStream << "Currently controlling Kinect: " << ofToString(currentKinect) << " out of: " << ofToString(nKinects) << endl;
    
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    // kinect 2
    if(kinect2.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect2.getMksAccel().x, 2) << " / "
        << ofToString(kinect2.getMksAccel().y, 2) << " / "
        << ofToString(kinect2.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV2 <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold2 << " (press: + -)" << endl
    << "set far threshold " << farThreshold2 << " (press: < >) num blobs found " << contourFinder2.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect2.isConnected() << endl;
    
    if(kinect2.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle2 << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    
    ofDrawBitmapString(reportStream.str(), 20, 652);
    
}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();

    kinect2.setCameraTiltAngle(0);
    kinect2.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            if(currentKinect == 0){
                bThreshWithOpenCV = !bThreshWithOpenCV;
            }
            else{
                bThreshWithOpenCV2 = !bThreshWithOpenCV2;
            }
            break;
            
        case'p':
            if(currentKinect == 0){
                bDrawPointCloud = !bDrawPointCloud;
            }
            else{
                bDrawPointCloud = !bDrawPointCloud;
            }
            break;
            
        case '>':
        case '.':
            if(currentKinect == 0){
                farThreshold ++;
                if (farThreshold > 255) farThreshold = 255;
            }
            else{
                farThreshold2 ++;
                if (farThreshold2 > 255) farThreshold2 = 255;
            }
            break;
            
        case '<':
        case ',':
            if(currentKinect == 0){
                farThreshold --;
                if (farThreshold < 0) farThreshold = 0;
            }
            else{
                farThreshold2 --;
                if (farThreshold2 < 0) farThreshold2 = 0;
            }
            break;
            
        case '+':
        case '=':
            if(currentKinect == 0){
                nearThreshold ++;
                if (nearThreshold > 255) nearThreshold = 255;
            }
            else{
                nearThreshold2 ++;
                if (nearThreshold2 > 255) nearThreshold2 = 255;
            }
            
            break;
            
        case '-':
            if(currentKinect == 0){
                nearThreshold --;
                if (nearThreshold < 0) nearThreshold = 0;
            }
            else{
                nearThreshold2 --;
                if (nearThreshold2 < 0) nearThreshold2 = 0;
            }
            break;
            
        case 'w':
            if(currentKinect == 0){
                kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            }
            else{
                kinect2.enableDepthNearValueWhite(!kinect2.isDepthNearValueWhite());
            }
            break;
            
        case 'o':
            if(currentKinect == 0){
                kinect.setCameraTiltAngle(angle); // go back to prev tilt
                kinect.open();
            }
            else{
                kinect2.setCameraTiltAngle(angle2); // go back to prev tilt
                kinect2.open();
            }
            break;
            
        case 'c':
            if(currentKinect == 0){
                kinect.setCameraTiltAngle(0); // zero the tilt
                kinect.close();
            }
            else{
                kinect.setCameraTiltAngle(0); // zero the tilt
                kinect.close();
            }
            break;
            
        case '1':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_GREEN);
            }
            else{
                kinect2.setLed(ofxKinect::LED_GREEN);
                
            }
            break;
            
        case '2':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_YELLOW);
            }
            else{
                kinect2.setLed(ofxKinect::LED_YELLOW);
            }
            break;
            
        case '3':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_RED);
            }
            else{
                kinect2.setLed(ofxKinect::LED_RED);
            }
            break;
            
        case '4':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            }
            else{
                kinect2.setLed(ofxKinect::LED_BLINK_GREEN);
            }
            break;
            
        case '5':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            }
            else{
                kinect2.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            }
            break;
            
        case '0':
            if(currentKinect == 0){
                kinect.setLed(ofxKinect::LED_OFF);
            }
            else{
                kinect2.setLed(ofxKinect::LED_OFF);
            }
            break;
            
        case OF_KEY_UP:
            if(currentKinect == 0){
                angle++;
                if(angle>30) angle=30;
                kinect.setCameraTiltAngle(angle);
            }
            else{
                angle2++;
                if(angle2>30) angle2=30;
                kinect2.setCameraTiltAngle(angle2);
            }
            break;
            
        case OF_KEY_DOWN:
            if(currentKinect == 0){
                angle--;
                if(angle<-30) angle=-30;
                kinect.setCameraTiltAngle(angle);
            }
            else{
                angle2--;
                if(angle2<-30) angle2=-30;
                kinect2.setCameraTiltAngle(angle2);
            }
            break;
            
        case OF_KEY_LEFT:
            currentKinect--;
            if(currentKinect<0) currentKinect=0;
            
            break;
            
        case OF_KEY_RIGHT:
            currentKinect++;
            if(currentKinect >= nKinects) currentKinect=nKinects-1;
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
