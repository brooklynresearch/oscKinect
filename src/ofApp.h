#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        void exit();
        
        void drawPointCloud();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void loadParameters(int loadFor);
        int findRealXPos();
        int findRealYPos();
    
        ofxKinect kinect;
        ofxKinect kinect2;
        ofxKinect** kinects;
        int nKinects;
        int currentKinect;
    
        ofxCvColorImage colorImg;
        
        ofxCvGrayscaleImage grayImage; // grayscale depth image
        ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
        ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
        
        ofxCvContourFinder contourFinder;
    
    
        bool bThreshWithOpenCV;
        bool bDrawPointCloud;
        
        int nearThreshold;
        int farThreshold;
    
        int blobMinSize;
        int blobMaxSize;
        int maxBlobs;
    
        int topCrop;
        int botCrop;
        int leftCrop;
        int rightCrop;
        
    
        int angle;
    
        // second Kinect stuff
        ofxCvColorImage colorImg2;
        
        ofxCvGrayscaleImage grayImage2; // grayscale depth image
        ofxCvGrayscaleImage grayThreshNear2; // the near thresholded image
        ofxCvGrayscaleImage grayThreshFar2; // the far thresholded image
        
        ofxCvContourFinder contourFinder2;
    
        bool bThreshWithOpenCV2;
        bool bDrawPointCloud2;
        
        int nearThreshold2;
        int farThreshold2;
    
        int blobMinSize2;
        int blobMaxSize2;
        int maxBlobs2;
        
        int topCrop2;
        int botCrop2;
        int leftCrop2;
        int rightCrop2;
        
    
        int angle2;
        
    
    
        // used for viewing the point cloud
        ofEasyCam easyCam;
		
};
