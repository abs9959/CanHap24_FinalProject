
/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  

/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 

/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
int               hardwareVersion                     = 3;
/* end device block definition *****************************************************************************************/

/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 400.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;

/* forces and radial params */
PVector           actingForce                         = new PVector(0, 0);

float             min_x                               = -0.07;
float             max_x                               = 0.055;
float             min_y                               = 0.0;
float             max_y                               = 0.1;

//scaling factors for tangential force application for height based forces
float scalingFactorX = 2;
float scalingFactorY = 2;

// Define the coefficients of the polynomial function (e.g., ax^2 + bx + c)
float a = 1.0;
float b = 2.0;
float c = 3.0;
 
// Define the distance threshold for applying force
double threshold;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
FWorld            world;
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;

/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;

/* Lab3 variables */
PVector lastPosEE = new PVector(0,0);
enum FFNUM {
  ONE,
  TWO,
  THREE,
  FOUR,
};

FFNUM ffNum = FFNUM.ONE;
PFont f;

//Image objects
PImage flower;
PImage painting2;

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  /* screen size definition */
  size(1000, 700);
  
  /* set font type and size */
  f = createFont("Arial", 16, true);
  
  //haplyBoard          = new Board(this, Serial.list()[0], 0);
  haplyBoard          = new Board(this, "/dev/cu.usbmodem2101", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);

  widgetOne.device_set_parameters();
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
  
  flower = loadImage("images/flower.jpg");
  painting2 = loadImage("images/painting2.png");
 
}
/* end setup section ***************************************************************************************************/
  
/* draw section ********************************************************************************************************/
void draw(){
  if(!renderingForce) {
    
    background(255); 
    if(ffNum == FFNUM.ONE) {
      text("First mode", 500, 300);
      text("*Haptic Experience Window*", 100, 75);
      fill(#000000);
      text("Instructions:\nEach of the 3 experiences below represents a texture. \nThe textures are as follows:\n1. Instructions \n2. The Flower \n3. second painting name \n4. third painting name \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress '1' for the first experience. \nPress '2' for the second experience and so on... ", 100, 100);
      fill(#000000);
      text("Current mode:", 300, 300);
      fill(#000000);
    } else if(ffNum == FFNUM.TWO) {
      text("THE FLOWER", 670, 80);
    } else if(ffNum == FFNUM.THREE) {
      text("ABSTRACT BLUE N.01", 800, 175);
    } else if(ffNum == FFNUM.FOUR) {
      fill(0);
      text("Fourth mode", 500, 300);
    }
    
    if (ffNum == FFNUM.TWO) {
      float scaleFactor = 0.6; // Adjust as needed
      
      // Calculate the scaled width and height of the image
      float scaledWidth = flower.width * scaleFactor;
      float scaledHeight = flower.height * scaleFactor;
      
      imageMode(CORNER);
      image(flower, map(-0.05,-0.079, 0.071, 0, 1000), map(0.01, 0, 0.1, 0, 700), scaledWidth, scaledHeight);

      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
      stroke(0); // Set stroke color to black
      strokeWeight(2); // Set stroke weight
    }
    
    if (ffNum == FFNUM.THREE) {
       //background(255); // Clear the screen
  
      // Calculate the scaled dimensions
      float scaleFactor = min(width / (float)painting2.width, height / (float)painting2.height);
      int scaledWidth = int(painting2.width * 0.3);
      int scaledHeight = int(painting2.height * 0.3);
      
      // Draw the scaled image at the center of the screen
      imageMode(CENTER);
      image(painting2, width / 2 - 55, height / 2 + 50, scaledWidth, scaledHeight);
      
      fill(0);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
      //stroke(0); 
      strokeWeight(2);    
    }
  }
}
/* end draw section ****************************************************************************************************/
 
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{ 
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      lastPosEE = posEE.copy();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 

      /* haptic wall force calculation */
      actingForce.set(0, 0);
     

//PAINTING 1*********************************************************************************************************************
      if(ffNum == FFNUM.TWO) {
        if(posEE.y > 0.0283 && posEE.y <= 0.0301){
            if(posEE.x > -0.0185 && posEE.x <= -0.004){
              if(posEE.y > 0.0282 && posEE.y <= 0.0295) {
              fEE.set(0,-3);
              }
            }
          }
          
        else if(posEE.y > 0.0363 && posEE.y <= 0.0383){
          if(posEE.x > -0.0102 && posEE.x <= 0.002){
            if(posEE.y > 0.0363 && posEE.y <= 0.0372) {
            fEE.set(0,-3);
            }
          }
        }
          
        else if(posEE.y > 0.0499 && posEE.y <= 0.0520){
          if(posEE.x > -0.0047 && posEE.x <= 0.0031){
            if(posEE.y > 0.0499 && posEE.y <= 0.0507) {
            fEE.set(0,3);
            }
          }
        }
          
        else if(posEE.y > 0.0635 && posEE.y <= 0.0652){
          if(posEE.x > -0.0177 && posEE.x <= -0.0041){
            if(posEE.y > 0.0635 && posEE.y <= 0.0642) {
            fEE.set(0,3);
            }
          }
        }
        
        else if(posEE.y > 0.0590 && posEE.y <= 0.0605){
          if(posEE.x > -0.0261 && posEE.x <= -0.0133){
            if(posEE.y > 0.0590 && posEE.y <= 0.0595) {
            fEE.set(0,3);
            }
          }
        }
        
        else if(posEE.y > 0.0440 && posEE.y <= 0.0461){
          if(posEE.x > -0.0257 && posEE.x <= -0.0130){
            if(posEE.y > 0.0440 && posEE.y <= 0.0453) {
            fEE.set(0,-3);
            }
          }
        }
        
        else if(posEE.y > 0.0320 && posEE.y <= 0.0337){
          if(posEE.x > -0.0284 && posEE.x <= -0.0179){
            if(posEE.y > 0.0320 && posEE.y <= 0.0329) {
            fEE.set(0,-3);
            }
          }
        }
        
        else if(posEE.y > 0.0419 && posEE.y <= 0.0436){
          if(posEE.x > -0.0174 && posEE.x <= -0.0095){
            if(posEE.y > 0.0419 && posEE.y <= 0.0427) {
            fEE.set(0,3);
            }
          }
        }
        
        else if(posEE.y > 0.0489 && posEE.y <= 0.0506){
          if(posEE.x > -0.0132 && posEE.x <= -0.0050){
            if(posEE.y > 0.0489 && posEE.y <= 0.0497) {
            fEE.set(0,-3);
            }
          }
        }

        else if(posEE.y > 0.0780){
          double threshold = 0.00025; // Threshold for distance from multiples of 0.1
          double interval = 0.001;    // Interval between multiples of 0.1
          
          // Calculate the nearest multiple of 0.1 to posEE.x
          double nearestMultipleX = Math.round(posEE.x / interval) * interval;
          
          // Calculate the distance between posEE.x and the nearest multiple of 0.1
          double distanceX = Math.abs(posEE.x - nearestMultipleX);
          
          // Check if the distance is within the threshold
          if (distanceX <= threshold) {
              fEE.set(75, 0);
          } else {
              fEE.set(0, 0);
          }
          
          // Calculate the nearest multiple of 0.1 to posEE.x
          double nearestMultipleY = Math.round(posEE.y / interval) * interval;
          
          // Calculate the distance between posEE.x and the nearest multiple of 0.1
          double distanceY = Math.abs(posEE.y - nearestMultipleY);
          
          // Check if the distance is within the threshold
          if (distanceY <= threshold) {
              fEE.set(0, 75);
          } else {
              fEE.set(0, 0);
          } 
        }
        
        else {
          fEE.set(0, 0);
        }
        
        lastPosEE = posEE.copy();
      }
      
//PAINTING 2*********************************************************************************************************************     
      else if (ffNum == FFNUM.THREE)
      {
        //[rectLeft, rectRight, rectTop, rectBottom]
        float[] rect1 = new float[]{-0.0519, -0.039, 0.0500, 0.0743};
        float[] rect2 = new float[]{-0.0519, 0.00625, 0.0777, 0.0977};
        float[] rect3 = new float[]{-0.0325, -0.015, 0.0238, 0.0743};
        float[] rect4 = new float[]{-0.0113, 0.0119, 0.0238, 0.0354};
        float[] rect5 = new float[]{-0.0113, 0.0013, 0.0408, 0.0743};
        float[] rect6 = new float[]{0.0063, 0.0260, 0.0538, 0.0738};
        float forceMagnitude = 2; // Adjust as needed
       
// rectangle 1 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        if (posEE.y > rect1[2] && posEE.y < rect1[2] + 0.001 && posEE.x > rect1[0] && posEE.x < rect1[1] + 0.005) {
          fEE.add(0, -1); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect1[3] && posEE.y > rect1[3] - 0.001 && posEE.x > rect1[0] && posEE.x < rect1[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        }
        //Left wall
        else if (posEE.x < rect1[0] && posEE.x > rect1[0] - 0.002 && posEE.y > rect1[2] && posEE.y < rect1[3]) {
          fEE.add(10,0.05); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect1[1] && posEE.x < rect1[1] + 0.002 && posEE.y > rect1[2] && posEE.y < rect1[3]) {
          fEE.add(10,0.05); // Apply force upwards
        }
        else if (posEE.x > rect1[0] && posEE.x < rect1[1] && posEE.y > rect1[2] && posEE.y < rect1[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.x)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
          
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(1,0);
          }
          
          else{
            fEE.set(0,0);
          }
        }
        
// rectangle 2 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        else if (posEE.y > rect2[2] && posEE.y < rect2[2] + 0.001 && posEE.x > rect2[0] && posEE.x < rect2[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect2[3] && posEE.y > rect2[3] - 0.001 && posEE.x > rect2[0] && posEE.x < rect2[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        }
        //Left wall
        else if (posEE.x < rect2[0] && posEE.x > rect2[0] - 0.001 && posEE.y > rect2[2] && posEE.y < rect2[3]) {
          fEE.add(10,0.1); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect2[1] + 0.005 && posEE.x < rect2[1] + 0.01 && posEE.y > rect2[2] && posEE.y < rect2[3]) {
          fEE.add(10,0.1); // Apply force upwards
        }
        else if (posEE.x > rect2[0] && posEE.x < rect2[1] + 0.007 && posEE.y > rect2[2] && posEE.y < rect2[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.y)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
         
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(0,1.2);
          }
          
          else{
            fEE.set(0,0);
          }
        }

// rectangle 3 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        else if (posEE.y > rect3[2] && posEE.y < rect3[2] + 0.001 && posEE.x > rect3[0] && posEE.x < rect3[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect3[3] && posEE.y > rect3[3] - 0.001 && posEE.x > rect3[0] && posEE.x < rect3[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        }
        //Left wall
        else if (posEE.x < rect3[0] && posEE.x > rect3[0] - 0.0003 && posEE.y > rect3[2] && posEE.y < rect3[3]) {
          fEE.add(10,0.1); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect3[1] + 0.005 && posEE.x < rect3[1] + 0.006 && posEE.y > rect3[2] && posEE.y < rect3[3]) {
          fEE.add(10,0.01); // Apply force upwards
        }
        else if (posEE.x > rect3[0] + 0.002 && posEE.x < rect3[1] && posEE.y > rect3[2] && posEE.y < rect3[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.x)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
          
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(1.5,0);
          }
          
          else{
            fEE.set(0,0);
          }
        }
        
// rectangle 4 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        else if (posEE.y > rect4[2] && posEE.y < rect4[2] + 0.001 && posEE.x > rect4[0] && posEE.x < rect4[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect4[3] && posEE.y > rect4[3] - 0.001 && posEE.x > rect4[0] && posEE.x < rect4[1] + 0.005) {
          fEE.add(0, -0.2); // Apply force upwards
        }
        //Left wall
        else if (posEE.x < rect4[0] + 0.003 && posEE.x > rect4[0] - 0.006 && posEE.y > rect4[2] && posEE.y < rect4[3]) {
          fEE.add(-10,0.1); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect4[1] + 0.005 && posEE.x < rect4[1] + 0.007 && posEE.y > rect4[2] && posEE.y < rect4[3]) {
          fEE.add(10,0.01); // Apply force upwards
        }
        else if (posEE.x > rect4[0] + 0.005 && posEE.x < rect4[1] + 0.005 && posEE.y > rect4[2] && posEE.y < rect4[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.y)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
          
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(0,2);
          }
          
          else{
            fEE.set(0,0);
          }
        }

// rectangle 5 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        else if (posEE.y > rect5[2] && posEE.y < rect5[2] + 0.001 && posEE.x > rect5[0] && posEE.x < rect5[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect5[3] && posEE.y > rect5[3] - 0.001 && posEE.x > rect5[0] && posEE.x < rect5[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        }
        //Left wall
        else if (posEE.x > rect5[0] && posEE.x < rect5[0] + 0.001 && posEE.y > rect5[2] && posEE.y < rect5[3]) {
          fEE.add(-10,0.1); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect5[1] + 0.005 && posEE.x < rect5[1] + 0.007 && posEE.y > rect5[2] && posEE.y < rect5[3]) {
          fEE.add(10,0.01); // Apply force upwards
        }
        else if (posEE.x > rect5[0] && posEE.x < rect5[1] && posEE.y > rect5[2] && posEE.y < rect5[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.x)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
          
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(1.5,0);
          }
          
          else{
            fEE.set(0,0);
          }
        }

// rectangle 6 --------------------------------------------------------------------------------------------------------------------
        //Top wall
        else if (posEE.y > rect6[2] && posEE.y < rect6[2] + 0.001 && posEE.x > rect6[0] && posEE.x < rect6[1] + 0.005) {
          fEE.add(0, -1); // Apply force upwards
        } 
        //Bottom wall
        else if (posEE.y < rect6[3] && posEE.y > rect6[3] - 0.001 && posEE.x > rect6[0] && posEE.x < rect6[1] + 0.005) {
          fEE.add(0, -0.5); // Apply force upwards
        }
        //Left wall
        else if (posEE.x > rect6[0] && posEE.x < rect6[0] + 0.001 && posEE.y > rect6[2] && posEE.y < rect6[3]) {
          fEE.add(-10,0.1); // Apply force upwards
        }
        //Right wall
        else if (posEE.x > rect6[1] + 0.007 && posEE.x < rect6[1] + 0.008 && posEE.y > rect6[2] && posEE.y < rect6[3]) {
          fEE.add(10,0.01); // Apply force upwards
        }
        else if (posEE.x > rect6[0] + 0.007 && posEE.x < rect6[1] + 0.007 && posEE.y > rect6[2] && posEE.y < rect6[3]) {
          double roundedNumber = Double.parseDouble(String.format("%.4f", Math.abs(posEE.y)));
          double interval = Math.pow(10, -3);
          double distance = roundedNumber % interval;
          double roundedDist = Double.parseDouble(String.format("%.4f", distance));
          
          if (roundedDist == 0 || roundedDist == interval * Math.pow(10, -1) * 9 || roundedDist == interval * Math.pow(10, -1) ||  roundedDist == interval * Math.pow(10, -1) * 8 ||  roundedDist == interval * Math.pow(10, -1) * 2 ){
            fEE.set(0,1.2);
          }
          
          else{
            fEE.set(0,0);
          }
        }
        
        else {
          fEE.set(0, 0); 
        }
        
          
        lastPosEE = posEE.copy();
      }
   
 //END OF EXPERIENCES**********************************************************************************************************************     
        
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
    }
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/

/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}

PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void keyPressed() {
  if(key == '1') {
    ffNum = FFNUM.ONE;
  } else if(key == '2') {
    ffNum = FFNUM.TWO;
  } else if(key == '3') {
    ffNum = FFNUM.THREE;
  } else if(key == '4') {
    ffNum = FFNUM.FOUR;
  }
}

double[] calculateForce(float positionX, float positionY, double wavelength, double angle) {
    double triangleX = triangleWave(positionX, wavelength, angle);
    double triangleY = triangleWave(positionY, wavelength, angle);

    // Assuming linear scaling for simplicity
    double forceX = triangleX * scalingFactorX;
    double forceY = triangleY * scalingFactorY;
    return new double[]{forceX, forceY};
}

double triangleWave(double x, double wavelength, double angle) {
    double period = wavelength / 2.0;
    double phaseShift = Math.PI / 2.0; // Phase shift for triangle wave

    // Calculate phase offset based on angle
    double phaseOffset = Math.tan(angle);

    // Calculate triangle wave using phase offset
    double wave = Math.abs((x / period + phaseOffset) % 2 - 1);
    return wave;
}

float calculatePolynomial(float x) {
    //return (float)Math.sqrt(0.0005 - (float)Math.pow(x, 2)) + 0.03;
    return x+0.05;
  }
  
float calculatePolynomial2(float x) {
    //return (float)Math.sqrt(0.0005 - (float)Math.pow(x, 2)) + 0.03;
    return x+0.06;
  }

  // Calculate the derivative of the polynomial function at a given x
float calculatePolynomialDerivative(float x) {
    return -(x/(1+0.03)); // Derivative of a quadratic polynomial
  }
  
float mapX(float x) {
    // Scale the x-coordinate from the range (-0.07 to 0.055) to (0 to 1000)
    // Formula: newX = (x - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin
    return (x + 0.07) * (1000.0 / (0.066 + 0.07));
}

float mapY(float y) {
    // Scale the y-coordinate from the range (0 to 0.1) to (0 to 650)
    // Formula: newY = y * (newMax / oldMax)
    return y * (650.0 / 0.1);
}

/* end helper functions section ****************************************************************************************/
