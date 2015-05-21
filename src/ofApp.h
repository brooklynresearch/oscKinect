#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#define HOST "localhost"
//#define HOST "192.168.0.126"
#define PORT 12345
#define NUM_PARAMS 17

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
        void saveParameters(int saveFor);
        float findRealXPos(float modX, int calcFor);
        float findRealYPos(float modY, int calcFor);
        void sendOSCPosition(int currentSector, int currentBlob, float xPos, float yPos);
        void setKinectParameters(string &idName, string &value);
        void displayKinectParameters(int xPos, int yPos);
    
        string HOSTIP;
        int PORTNUM;
        string kinectID;
        string kinectID2;
        int sectorID;
        int sectorID2;
    
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
    
        ofxOscSender sender;
    
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
    
        float xAdd;
        float yAdd;
        float xMult;
        float yMult;
    
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
        
        float xAdd2;
        float yAdd2;
        float xMult2;
        float yMult2;
    
        int angle2;
    
        ofXml XML;
    
    
        /*
        string kinect0Params[14][2] = {{"XLEFT","0"},{"XRIGHT","640"},{"YTOP","0"},{"YBOT","480"},{"NEAR","0"},{"FAR","200"},{"SERIAL","123"},{"MINSIZE","20"},{"MAXSIZE","2000"},{"BLOBNUM","5"},{"XMULT","1"},{"YMULT","1"},{"XADD","0"},{"YADD","0"}};
        string kinect1Params[14][2] = {{"XLEFT","0"},{"XRIGHT","640"},{"YTOP","0"},{"YBOT","480"},{"NEAR","0"},{"FAR","200"},{"SERIAL","123"},{"MINSIZE","20"},{"MAXSIZE","2000"},{"BLOBNUM","5"},{"XMULT","1"},{"YMULT","1"},{"XADD","0"},{"YADD","0"}};
    
    
        enum parameterNum{
            XLEFT,
            XRIGHT,
            YTOP,
            YBOT,
            NEAR,
            FAR,
            SERIAL,
            MINSIZE,
            MAXSIZE,
            BLOBNUM,
            XMULT,
            YMULT,
            XADD,
            YADD,
            MAX_PARAMS
        };
    
        string xmlStructure;
        */
    
        // used for viewing the point cloud
        ofEasyCam easyCam;
		
};
