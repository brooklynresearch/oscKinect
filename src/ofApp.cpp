#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    if( XML.load("settings.xml") ){
        cout << "WE GOT IT" << endl;
    } else {
        cout << "WE DONT GOT IT" <<endl;
    }
    // load up parameters
    loadParameters(0);
    sender.setup(HOSTIP, PORTNUM);
    
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
//    kinect.open(kinectID);
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
    

    bThreshWithOpenCV = false;
    
    
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
    
    
    // second kinect
    kinect2.init(false, false);
    kinect2.open();
//    kinect2.open(kinectID2);
    
    // print the intrinsic IR sensor values
    if(kinect2.isConnected()) {        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect2.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect2.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect2.getZeroPlaneDistance() << "mm";
    }

    
    grayImage2.allocate(kinect2.width, kinect2.height);
    grayThreshNear2.allocate(kinect2.width, kinect2.height);
    grayThreshFar2.allocate(kinect2.width, kinect2.height);
    

    bThreshWithOpenCV2 = false;
    
    // load up parameters for second kinect
    kinect2.setCameraTiltAngle(angle2);
    
    // start from the front
    bDrawPointCloud2 = false;
    
    ofSetFrameRate(30);
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
                    // bottom crop factor
                    if(i > (kinect.height - botCrop)*grayImage.width){
                      pix[i] = 0;
                    }
                    // top crop factor
                    else if(i < topCrop*grayImage.width){
                        pix[i] = 0;
                    }
                    // left crop factor
                    else if(i % grayImage.width < leftCrop){
                        pix[i] = 0;
                    }
                    // right crop factor
                    else if(i % grayImage.width > kinect.width - rightCrop){
                        pix[i] = 0;
                    }
                    else{
                      pix[i] = 255;
                    }
                    
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        // findContours(grayImage, minArea, maxArea, nConsidered, findHoles
        contourFinder.findContours(grayImage, blobMinSize, blobMaxSize, maxBlobs, false);
        
        // go through blobs and send message, can be compartmentalized elsewhere
        for(int i = 0; i < contourFinder.nBlobs; i++){
            ofxCvBlob blob = contourFinder.blobs.at(i);
            int rawX = blob.centroid.x;
            int rawY = blob.centroid.y;
            // possible dumb method for calculating use the below
            // and then do a proportional scale to predetermined thing
//            double tanMath = tan(0.4066176);
            
            // do calculations for each
            // make and send OSC signal?
            // constrained to to 0 and 1.0 on all
            float sendX = ofMap(rawX, 0, grayImage.width, 0, 1.0);
            float sendY = ofMap(rawY, 0, grayImage.height, 0.2*sectorID, 0.2*(sectorID+1));
            
            sendX = findRealXPos(sendX, 0);
            sendY = findRealYPos(sendY, 0);
            
            sendOSCPosition(sectorID, i, sendX, sendY);
        }
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
                    // bottom crop factor
                    if(i > (kinect2.height - botCrop2)*grayImage2.width){
                        pix[i] = 0;
                    }
                    // top crop factor
                    else if(i < topCrop2*grayImage2.width){
                        pix[i] = 0;
                    }
                    // left crop factor
                    else if(i % grayImage2.width < leftCrop2){
                        pix[i] = 0;
                    }
                    // right crop factor
                    else if(i % grayImage2.width > kinect2.width - rightCrop2){
                        pix[i] = 0;
                    }
                    else{
                        pix[i] = 255;
                    }
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage2.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder2.findContours(grayImage2, blobMinSize2, blobMaxSize2, maxBlobs2, false);
        
        // do the same as first kinect to calculate x and y position then send,
        // but possibly functionalize everything later
        
        // go through blobs and send message, can be compartmentalized elsewhere
        for(int i = 0; i < contourFinder2.nBlobs; i++){
            ofxCvBlob blob = contourFinder2.blobs.at(i);
            int rawX = blob.centroid.x;
            int rawY = blob.centroid.y;
            // possible dumb method for calculating use the below
            // and then do a proportional scale to predetermined thing
//            double tanMath = tan(0.4066176);
            
            // do calculations for each
            // make and send OSC signal?
            float sendX = ofMap(rawX, 0, grayImage2.width, 0, 1.0);
            float sendY = ofMap(rawY, 0, grayImage2.height, 0.2*sectorID2, 0.2*(sectorID2+1));
            
            sendX = findRealXPos(sendX, 1);
            sendY = findRealYPos(sendY, 1);
            sendOSCPosition(sectorID2, i, sendX, sendY);
        }
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
    stringstream blobReport;
    stringstream blobReport2;
    
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
    // cropping stats
    reportStream << "top, bottom, left, and right crop for 640x480: " << ofToString(topCrop) << ", "
    << ofToString(botCrop) << ", " << ofToString(leftCrop) << ", " << ofToString(rightCrop) << endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    // blob stats
    blobReport << "BLOB numbers for Kinect 1" << endl;
    
    for(int i = 0; i < contourFinder.nBlobs; i++){
        ofxCvBlob blob = contourFinder.blobs.at(i);
        int rawX = blob.centroid.x;
        int rawY = blob.centroid.y;
        double tanMath = tan(0.4066176);
        float sendX = ofMap(rawX, 0, grayImage.width, 0, 1.0);
        float sendY = ofMap(rawY, 0, grayImage.height,  0.2*sectorID, 0.2*(sectorID+1));
        blobReport << "estimated raw x, y for blob " << ofToString(i) << ": " << ofToString(rawX) << " "
            << ofToString(rawY) << " " << ofToString(sendX) << " " << ofToString(sendY) << endl;
        
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
    << endl;
    
    // cropping stats
    reportStream << "top, bottom, left, and right crop for 640x480: " << ofToString(topCrop2) << ", "
    << ofToString(botCrop2) << ", " << ofToString(leftCrop2) << ", " << ofToString(rightCrop2) << endl;
    
    
    if(kinect2.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle2 << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    // blob stats
    blobReport2 << "BLOB numbers for Kinect 2" << endl;
    
    for(int i = 0; i < contourFinder2.nBlobs; i++){
        ofxCvBlob blob = contourFinder2.blobs.at(i);
        int rawX = blob.centroid.x;
        int rawY = blob.centroid.y;
        double tanMath = tan(0.4066176);
        float sendX = ofMap(rawX, 0, grayImage2.width, 0, 1.0);
        float sendY = ofMap(rawY, 0, grayImage2.height,  0.2*sectorID2, 0.2*(sectorID2+1));
        blobReport2 << "estimated raw x, y for blob " << ofToString(i) << ": " << ofToString(rawX) << " "
        << ofToString(rawY) << " " << ofToString(sendX) << " " << ofToString(sendY) << endl;
        
    }
    
    ofDrawBitmapString(reportStream.str(), 20, 652);
    ofDrawBitmapString(blobReport.str(), 830, 10);
    ofDrawBitmapString(blobReport2.str(), 830, 512);
    displayKinectParameters(20, 975);
    
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
                //kinect0Params[NEAR][1] = nearThreshold;
            }
            else{
                nearThreshold2 ++;
                if (nearThreshold2 > 255) nearThreshold2 = 255;
                //kinect1Params[NEAR][1] = nearThreshold2;
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
        
        // crop variables
            
        // top crop
        // lower top
        case 't':
            if(currentKinect == 0){
                topCrop--;
                if(topCrop < 0) topCrop = 0;
            }
            else{
                topCrop2--;
                if(topCrop2 < 0) topCrop2 = 0;
            }
            break;
        
        case 'g':
            if(currentKinect == 0){
                topCrop++;
            }
            else{
                topCrop2++;
            }
            break;
        
        // bottom crop
        // raise bottom crop
        case 'y':
            if(currentKinect == 0){
                botCrop++;
            }
            else{
                botCrop2++;
            }
            break;
        // lower bottom crop
        case 'h':
            if(currentKinect == 0){
                botCrop--;
                if(botCrop < 0) botCrop = 0;
            }
            else{
                botCrop2--;
                if(botCrop2 < 0) botCrop2 = 0;
            }
            break;
        
        // left crop
        // decrease left crop
        case 'u':
            if(currentKinect == 0){
                leftCrop--;
                if(leftCrop < 0) leftCrop = 0;
            }
            else{
                leftCrop2--;
                if(leftCrop2 < 0) leftCrop2 = 0;
            }
            break;
        // increase left crop
        case 'i':
            if(currentKinect == 0){
                leftCrop++;
            }
            else{
                leftCrop2++;
            }
            break;
        
        // right crop
        // increase right crop
        case 'j':
            if(currentKinect == 0){
                rightCrop++;
            }
            else{
                rightCrop2++;
            }
            break;
        // decrease right crop
        case 'k':
            if(currentKinect == 0){
                rightCrop--;
                if(rightCrop--) rightCrop = 0;
            }
            else{
                rightCrop2--;
                if(rightCrop2--) rightCrop2 = 0;
            }
            break;
            
        // change kinect angle
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
            
        // change current Kinect being adjusted
        case OF_KEY_LEFT:
            currentKinect--;
            if(currentKinect<0) currentKinect=0;
            
            break;
            
        case OF_KEY_RIGHT:
            currentKinect++;
            if(currentKinect >= nKinects) currentKinect=nKinects-1;
            break;
            
        case 's':
            // save variables here
            //saveParameters(currentKinect);
            saveParameters(0);
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


// load variables
void ofApp::loadParameters(int loadFor){

    
        XML.setTo("KINECT[" + ofToString(loadFor) + "]");
    
    if(XML.exists("HOSTIP")) {
        HOSTIP	= XML.getValue<string>("HOSTIP");
    } else {
        HOSTIP = "localhost";
    }
    if(XML.exists("PORTNUM")) {
        PORTNUM	= XML.getValue<int>("PORTNUM");
    } else {
        PORTNUM = 12345;
    }
    if(XML.exists("KINECTID")) {
        kinectID	= XML.getValue<string>("KINECTID");
    } else {
        kinectID = 	ofxKinect::nextAvailableSerial();
    }
    if(XML.exists("KINECTID2")) {
        kinectID2	= XML.getValue<string>("KINECTID2");
    } else {
        kinectID2 = ofxKinect::nextAvailableSerial()	;
    }
    if(XML.exists("SECTORID")) {
        sectorID	= XML.getValue<int>("SECTORID");
    } else {
        sectorID = 0;
    }
    if(XML.exists("SECTORID2")) {
        sectorID2	= XML.getValue<int>("SECTORID2");
    } else {
        sectorID2 = 1;
    }
        if(XML.exists("XLEFT")) {
            leftCrop	= XML.getValue<int>("XLEFT");
        } else {
            leftCrop = 0;
        }
        if(XML.exists("XRIGHT")) {
            rightCrop	= XML.getValue<int>("XRIGHT");
        } else {
            rightCrop = kinect.width;
        }
        if(XML.exists("YTOP")) {
            topCrop	= XML.getValue<int>("YTOP");
        } else {
            topCrop = 0;
        }
        if(XML.exists("YBOT")) {
            botCrop	= XML.getValue<int>("YBOT");
        } else {
            botCrop = kinect.height;
        }
        if(XML.exists("NEAR")) {
            nearThreshold	= XML.getValue<int>("NEAR");
        } else {
            nearThreshold = 0;
        }
        if(XML.exists("FAR")) {
            farThreshold	= XML.getValue<int>("FAR");
        } else {
            farThreshold = 200;
        }
        if(XML.exists("MINSIZE")) {
            blobMinSize	= XML.getValue<int>("MINSIZE");
        } else {
            blobMinSize = 20;
        }
        if(XML.exists("MAXSIZE")) {
            blobMaxSize	= XML.getValue<int>("MAXSIZE");
        } else {
            blobMaxSize = 20000;
        }
        if(XML.exists("BLOBNUM")) {
            maxBlobs = XML.getValue<int>("BLOBNUM");
        } else {
            maxBlobs = 5;
        }
        if(XML.exists("XMULT")) {
            xMult	= XML.getValue<float>("XMULT");
        } else {
            xMult = 1;
        }
        if(XML.exists("YMULT")) {
            yMult	= XML.getValue<float>("YMULT");
        } else {
            yMult = 1;
        }
        if(XML.exists("XADD")) {
            xAdd	= XML.getValue<float>("XADD");
        } else {
            xAdd = 0;
        }
        if(XML.exists("YADD")) {
            yAdd	= XML.getValue<float>("YADD");
        } else {
            yAdd = 0;
        }
        if(XML.exists("ANGLE")) {
            angle	= XML.getValue<int>("ANGLE");
        } else {
            angle = 0;
        }
        if(XML.exists("XLEFT2")) {
            leftCrop2	= XML.getValue<int>("XLEFT2");
        } else {
            leftCrop2 = 0;
        }
        if(XML.exists("XRIGHT2")) {
            rightCrop2	= XML.getValue<int>("XRIGHT2");
        } else {
            rightCrop2 = kinect.width;
        }
        if(XML.exists("YTOP2")) {
            topCrop2	= XML.getValue<int>("YTOP2");
        } else {
            topCrop2 = 0;
        }
        if(XML.exists("YBOT2")) {
            botCrop2	= XML.getValue<int>("YBOT2");
        } else {
            botCrop2 = kinect.height;
        }
        if(XML.exists("NEAR2")) {
            nearThreshold2	= XML.getValue<int>("NEAR2");
        } else {
            nearThreshold2 = 0;
        }
        if(XML.exists("FAR2")) {
            farThreshold2	= XML.getValue<int>("FAR2");
        } else {
            farThreshold2 = 200;
        }
        if(XML.exists("MINSIZE2")) {
            blobMinSize2	= XML.getValue<int>("MINSIZE2");
        } else {
            blobMinSize2 = 20;
        }
        if(XML.exists("MAXSIZE2")) {
            blobMaxSize2	= XML.getValue<int>("MAXSIZE2");
        } else {
            blobMaxSize2 = 20000;
        }
        if(XML.exists("BLOBNUM2")) {
            maxBlobs2 = XML.getValue<int>("BLOBNUM2");
        } else {
            maxBlobs2 = 5;
        }
        if(XML.exists("XMULT2")) {
            xMult2	= XML.getValue<float>("XMULT2");
        } else {
            xMult2 = 1;
        }
        if(XML.exists("YMULT2")) {
            yMult2	= XML.getValue<float>("YMULT2");
        } else {
            yMult2 = 1;
        }
        if(XML.exists("XADD2")) {
            xAdd2	= XML.getValue<float>("XADD2");
        } else {
            xAdd2 = 0;
        }
        if(XML.exists("YADD2")) {
            yAdd2	= XML.getValue<float>("YADD2");
        } else {
            yAdd2 = 0;
        }
        if(XML.exists("ANGLE2")) {
            angle2	= XML.getValue<int>("ANGLE2");
        } else {
            angle2 = 0;
        }
}

// save variables
void ofApp::saveParameters(int saveFor){
    XML.clear();
    XML.addChild("PARAMS");
    XML.setTo("//PARAMS");
    XML.addChild("KINECT");
    XML.setTo("//KINECT");
    XML.addValue("HOSTIP",HOSTIP);
    XML.addValue("PORTNUM",ofToString(PORTNUM));
    XML.addValue("PORTNUM",ofToString(sectorID));
    XML.addValue("PORTNUM",ofToString(sectorID2));
    XML.addValue("KINECTID",kinectID);
    XML.addValue("KINECTID2",kinectID2);
    XML.addValue("XLEFT",ofToString(leftCrop));
    XML.addValue("XRIGHT",ofToString(rightCrop));
    XML.addValue("YTOP", ofToString(topCrop));
    XML.addValue("YBOT", ofToString(botCrop));
    XML.addValue("NEAR", ofToString(nearThreshold));
    XML.addValue("FAR", ofToString(farThreshold));
    XML.addValue("MINSIZE", ofToString(blobMinSize));
    XML.addValue("MAXSIZE", ofToString(blobMaxSize));
    XML.addValue("BLOBNUM", ofToString(maxBlobs));
    XML.addValue("XMULT", ofToString(xMult));
    XML.addValue("YMULT", ofToString(yMult));
    XML.addValue("XADD", ofToString(xAdd));
    XML.addValue("YADD", ofToString(yAdd));
    XML.addValue("ANGLE", ofToString(angle));
    XML.addValue("XLEFT2",ofToString(leftCrop2));
    XML.addValue("XRIGHT2",ofToString(rightCrop2));
    XML.addValue("YTOP2", ofToString(topCrop2));
    XML.addValue("YBOT2", ofToString(botCrop2));
    XML.addValue("NEAR2", ofToString(nearThreshold2));
    XML.addValue("FAR2", ofToString(farThreshold2));
    XML.addValue("MINSIZE2", ofToString(blobMinSize2));
    XML.addValue("MAXSIZE2", ofToString(blobMaxSize2));
    XML.addValue("BLOBNUM2", ofToString(maxBlobs2));
    XML.addValue("XMULT2", ofToString(xMult2));
    XML.addValue("YMULT2", ofToString(yMult2));
    XML.addValue("XADD2", ofToString(xAdd2));
    XML.addValue("YADD2", ofToString(yAdd2));
    XML.addValue("ANGLE2", ofToString(angle2));
    
    XML.save("settings.xml");
}

void ofApp::setKinectParameters(string &idName, string &value){
    XML.setValue(idName, value);
}

// calculating real positions to send
float ofApp::findRealXPos(float modX, int calcFor){
    float sendX = 0;
    
    if(calcFor == 0){
        sendX = modX*xMult + xAdd;
    }
    else{
        sendX = modX*xMult2+ xAdd2;
    }
    
    return sendX;
}

// calculating real positions to send
float ofApp::findRealYPos(float modY, int calcFor){
    float sendY = 0;
    
    if(calcFor == 0){
        sendY = modY*yMult + yAdd;
    }
    else{
        sendY = modY*yMult2+ yAdd2;
    }
    
    return sendY;
}

// sending x and y OSC positions.
void ofApp::sendOSCPosition(int currentSector, int currentBlob, float xPos, float yPos){
    ofxOscMessage m;
    m.setAddress("/Blob/Pos");
    m.addIntArg(currentSector);
    m.addIntArg(currentBlob);
    m.addFloatArg(xPos);
    m.addFloatArg(yPos);
    sender.sendMessage(m);
}

void ofApp::displayKinectParameters(int xPos, int yPos){
    stringstream kinectParamStream0;
    stringstream kinectParamStream1;
    kinectParamStream0 << "ID: " << "0" << "   Serial #: " << kinect.getSerial() << endl;
    kinectParamStream0 << "Left: " << ofToString(leftCrop) << "   Right: " << ofToString(rightCrop) << "   Top: " << ofToString(topCrop) << "   Bottom: " << ofToString(botCrop) << "   Near: " << ofToString(nearThreshold) << "   Far: " << ofToString(farThreshold) << endl;
    kinectParamStream0 << "Min Blob Size: " << ofToString(blobMinSize) << "   Max Blob Size: " << ofToString(blobMaxSize) << "   Max Number of Blobs: " << ofToString(maxBlobs) << endl;
    kinectParamStream0 << "X Multiplier: " << ofToString(xMult) << "   Y Multiplier: " << ofToString(yMult) << "   X Addition: " << ofToString(xAdd) << "   Y Addition " << ofToString(yAdd) << endl;
    ofDrawBitmapString(kinectParamStream0.str(), xPos , yPos);
    
    kinectParamStream1 << "ID: " << "1" << "   Serial #: " << kinect2.getSerial() << endl;
    kinectParamStream1 << "Left: " << ofToString(leftCrop2) << "   Right: " << ofToString(rightCrop2) << "   Top: " << ofToString(topCrop2) << "   Bottom: " << ofToString(botCrop2) << "   Near: " << ofToString(nearThreshold2) << "   Far: " << ofToString(farThreshold2) << endl;
    kinectParamStream1 << "Min Blob Size: " << ofToString(blobMinSize2) << "   Max Blob Size: " << ofToString(blobMaxSize2) << "   Max Number of Blobs: " << ofToString(maxBlobs2) << endl;
    kinectParamStream1 << "X Multiplier: " << ofToString(xMult2) << "   Y Multiplier: " << ofToString(yMult2) << "   X Addition: " << ofToString(xAdd2) << "   Y Addition " << ofToString(yAdd2) << endl;
    ofDrawBitmapString(kinectParamStream1.str(), (xPos + 600) , yPos);
}
