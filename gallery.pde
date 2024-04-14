
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
PImage painting3;

/*Curve Setting*/
PShape bezierCurve;
PShape bezierCurve2;

float[][] vertices = new float[12][8];
float[][] vertices2 = new float[6][8];

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 700);
  
  /* set font type and size */

  f = createFont("Arial", 16, true);
  //haplyBoard          = new Board(this, "/dev/cu.usbmodem2101", 0);
  haplyBoard          = new Board(this, Serial.list()[1], 0);

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
  painting3 = loadImage("images/artworkcolor.png");
    
  // Create a PShape object for the bezier curve
  bezierCurve = createShape();
  bezierCurve.beginShape();
  
  float[][] vertices = {{mapX(-0.012), mapX(-0.002), mapX(0.0095), mapX(-0.005), mapY(0.031), mapY(0.013), mapY(0.026), mapY(0.0315)},
                        {mapX(-0.005), mapX(-0.009), mapX(-0.009), mapX(-0.005), mapY(0.0315), mapY(0.031), mapY(0.033), mapY(0.034)},
                        {mapX(-0.004), mapX(0.0043), mapX(-0.003), mapX(-0.007), mapY(0.034), mapY(0.038), mapY(0.05), mapY(0.04)},
                        {mapX(-0.008), mapX(-0.009), mapX(-0.013), mapX(-0.0108), mapY(0.038), mapY(0.037), mapY(0.037), mapY(0.04)},
                        {mapX(-0.0108), mapX(-0.0053), mapX(-0.019), mapX(-0.0145), mapY(0.04), mapY(0.057), mapY(0.055), mapY(0.043)},
                        {mapX(-0.0145), mapX(-0.0146), mapX(-0.014), mapX(-0.017), mapY(0.042),mapY(0.047), mapY(0.038), mapY(0.042)},
                        {mapX(-0.0169), mapX(-0.0175), mapX(-0.0345), mapX(-0.023), mapY(0.042), mapY(0.055),  mapY(0.051),  mapY(0.042)},
                        {mapX(-0.023), mapX(-0.018), mapX(-0.0255), mapX(-0.025), mapY(0.042), mapY(0.033),  mapY(0.0385),  mapY(0.039)},
                        {mapX(-0.025), mapX(-0.031), mapX(-0.036), mapX(-0.028), mapY(0.039), mapY(0.049),  mapY(0.039),  mapY(0.036)},
                        {mapX(-0.028), mapX(-0.023), mapX(-0.025), mapX(-0.028), mapY(0.036), mapY(0.035),  mapY(0.031),  mapY(0.033)},
                        {mapX(-0.028), mapX(-0.039), mapX(-0.034), mapX(-0.028), mapY(0.033), mapY(0.033),  mapY(0.024),  mapY(0.028)},
                        {mapX(-0.028), mapX(-0.026), mapX(-0.025), mapX(-0.03), mapY(0.028), mapY(0.028),  mapY(0.025),  mapY(0.023)}};

 
  // Approximate the bezier curve by adding vertices
  
  for (int i = 0; i<12; i++){
    for (float t = 0; t <= 1; t += 0.01) {
    float x = bezierPoint(vertices[i][0], vertices[i][1], vertices[i][2], vertices[i][3], t);
    float y = bezierPoint(vertices[i][4], vertices[i][5], vertices[i][6], vertices[i][7], t);
    bezierCurve.vertex(x, y);
    }
  }
  bezierCurve.endShape();
  
  bezierCurve2 = createShape();
  bezierCurve2.beginShape();
  
  float[][] vertices2 = {{mapX(0.02), mapX(0.015), mapX(0.007), mapX(0.014), mapY(0.062), mapY(0.047), mapY(0.062), mapY(0.066)},
                        {mapX(0.014), mapX(0.022), mapX(0.016), mapX(0.011), mapY(0.066), mapY(0.068), mapY(0.075), mapY(0.069)},
                        {mapX(0.011), mapX(0.0082), mapX(0.001), mapX(0.011), mapY(0.069), mapY(0.0648), mapY(0.072), mapY(0.075)},
                        {mapX(0.011), mapX(0.024), mapX(0.018), mapX(0.013), mapY(0.075), mapY(0.075), mapY(0.08), mapY(0.08)},
                        {mapX(0.013), mapX(0.009), mapX(0.002), mapX(0.013), mapY(0.08), mapY(0.08), mapY(0.086), mapY(0.0856)},
                        {mapX(0.013), mapX(0.023), mapX(0.02), mapX(0.015), mapY(0.0856), mapY(0.076), mapY(0.087), mapY(0.091)}}; 
  
   for (int i = 0; i<6; i++){
    for (float t = 0; t <= 1; t += 0.01) {
    float x = bezierPoint(vertices2[i][0], vertices2[i][1], vertices2[i][2], vertices2[i][3], t);
    float y = bezierPoint(vertices2[i][4], vertices2[i][5], vertices2[i][6], vertices2[i][7], t);
    bezierCurve2.vertex(x, y);
    }
  }
  bezierCurve2.endShape();
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
      text("Instructions:\nEach of the 3 experiences below represents a texture. \nThe textures are as follows:\n1. Instructions \n2. The Flower \n3. Abstract Blue N.01 \n4. third painting name \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress '1' for the first experience. \nPress '2' for the second experience and so on... ", 100, 100);
      fill(#000000);
      text("Current mode:", 300, 300);
      fill(#000000);
    } else if(ffNum == FFNUM.TWO) {
      text("THE FLOWER", 670, 80);
    } else if(ffNum == FFNUM.THREE) {
      text("ABSTRACT BLUE N.01", 800, 175);
    } else if(ffNum == FFNUM.FOUR) {
      fill(0);
      text(" ", 500, 300);
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
    
    if (ffNum == FFNUM.FOUR){
     float scaleFactor = 0.2;
     
     float scaledWidth = painting3.width * scaleFactor;
     float scaledHeight = painting3.height * scaleFactor;
     
     image(painting3, width / 2 - 265, map(0.02, 0, 0.1, 0, 700), scaledWidth, scaledHeight);

     shape(bezierCurve);
     bezierCurve.setVisible(false);
     
     shape(bezierCurve2);
     bezierCurve2.setVisible(false);
   
     fill(#000000);
     text("Current mode: Seventh mode", 10,30);
     ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    
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
/* ABBY FLOWER */
      if (ffNum == FFNUM.TWO) {
        if(posEE.y > 0.0283 && posEE.y <= 0.0301){
          if(posEE.x > -0.0185 && posEE.x <= -0.004){
            if(posEE.y > 0.0282 && posEE.y <= 0.0295) {
              fEE.set(0,-3);
              }
            }
          }
        //top of right    
        else if(posEE.y > 0.0363 && posEE.y <= 0.0383){
          if(posEE.x > -0.0102 && posEE.x <= 0.002){
            if(posEE.y > 0.0363 && posEE.y <= 0.0372) {
              fEE.set(0,-3);
            }
          }
        
          if(posEE.x > -0.0186 && posEE.x <= -0.0172){
            if(posEE.y > 0.0293 && posEE.y <= 0.0429){
              if(posEE.x > -0.0186 && posEE.x <= -0.0177) {
                fEE.set(-3,0);
              }
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
        
        //top of bottom left     
        else if(posEE.y > 0.0440 && posEE.y <= 0.0461){
          if(posEE.x > -0.0257 && posEE.x <= -0.0130){
            if(posEE.y > 0.0440 && posEE.y <= 0.0453) {
              fEE.set(0,-3);
            }
          }
        
          if(posEE.x > -0.0293 && posEE.x <= -0.0277){
            if(posEE.y > 0.0329 && posEE.y <= 0.0474){
              if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
                fEE.set(-3,0);
              }
            }
          }
      
          if(posEE.x > 0.0019 && posEE.x <= 0.0033){
            if(posEE.y > 0.0370 && posEE.y <= 0.0503){
              if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
                fEE.set(3,0);
              }
            }
          }
      
          if(posEE.x > -0.0102 && posEE.x <= -0.0086){
            if(posEE.y > 0.0374 && posEE.y <= 0.0499){
              if(posEE.x > -0.0102 && posEE.x <= -0.0094) {
                fEE.set(-3,0);
              }
            }
          }
        }
        
        //top of top left   
        else if(posEE.y > 0.0320 && posEE.y <= 0.0337){
          if(posEE.x > -0.0284 && posEE.x <= -0.0179){
            if(posEE.y > 0.0320 && posEE.y <= 0.0329) {
              fEE.set(0,-3);
            }
          }
        
          if(posEE.x > -0.0053 && posEE.x <= -0.0038){
            if(posEE.y > 0.0280 && posEE.y <= 0.0372){
              if(posEE.x > -0.0055 && posEE.x <= -0.0046) {
                fEE.set(3,0);
              }
            }
          } 
        }
        
       //bottom of top   
        else if(posEE.y > 0.0419 && posEE.y <= 0.0436){
          if(posEE.x > -0.0174 && posEE.x <= -0.0095){
            if(posEE.y > 0.0419 && posEE.y <= 0.0427) {
              fEE.set(0,3);
            }
          }
        
          if(posEE.x > -0.0293 && posEE.x <= -0.0277){
            if(posEE.y > 0.0329 && posEE.y <= 0.0474){
              if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
                fEE.set(-3,0);
              }
            }
          }
      
          if(posEE.x > 0.0019 && posEE.x <= 0.0033){
            if(posEE.y > 0.0370 && posEE.y <= 0.0503){
              if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
                fEE.set(3,0);
              }
            }
          }
        }
      
        else if(posEE.y > 0.0489 && posEE.y <= 0.0506){
          if(posEE.x > -0.0132 && posEE.x <= -0.0050){
            if(posEE.y > 0.0489 && posEE.y <= 0.0497) {
              fEE.set(0,-3);
            }
          }
        
          if(posEE.x > -0.0262 && posEE.x <= -0.0249){
            if(posEE.y > 0.0462 && posEE.y <= 0.0599){
              if(posEE.x > -0.0262 && posEE.x <= 0.0258) {
                fEE.set(-3,0);
              }
            }
          }
        }
        
        //right of top      
        else if(posEE.x > -0.0053 && posEE.x <= -0.0038){
          if(posEE.y > 0.0280 && posEE.y <= 0.0372){
            if(posEE.x > -0.0055 && posEE.x <= -0.0046) {
              fEE.set(3,0);
            }
          }
          if(posEE.y > 0.0488 && posEE.y <= 0.0646){
            if(posEE.x > -0.0053 && posEE.x <= -0.0045) {
              fEE.set(3,0);
            }
          }
        }
        
        //right of right   
        else if(posEE.x > 0.0019 && posEE.x <= 0.0033){
          if(posEE.y > 0.0370 && posEE.y <= 0.0503){
            if(posEE.x > 0.0019 && posEE.x <= 0.0026) {
              fEE.set(3,0);
            }
          }
        }
        
        //right of bottom left     
        else if(posEE.x > -0.0138 && posEE.x <= -0.0124){
          if(posEE.y > 0.0446 && posEE.y <= 0.0599){
            if(posEE.x > -0.01389 && posEE.x <= 0.0026) {
              fEE.set(3,0);
            }
          }
        }
        
        //left of bottom left     
        else if(posEE.x > -0.0262 && posEE.x <= -0.0249){
          if(posEE.y > 0.0462 && posEE.y <= 0.0599){
            if(posEE.x > -0.0262 && posEE.x <= 0.0258) {
              fEE.set(-3,0);
            }
          }
        }
        
        //left of left     
        else if(posEE.x > -0.0293 && posEE.x <= -0.0277){
          if(posEE.y > 0.0329 && posEE.y <= 0.0474){
            if(posEE.x > -0.0293 && posEE.x <= -0.0285) {
              fEE.set(-3,0);
            }
          }
        }
        
        // left of top    
        else if(posEE.x > -0.0186 && posEE.x <= -0.0172){
          if(posEE.y > 0.0293 && posEE.y <= 0.0429){
            if(posEE.x > -0.0186 && posEE.x <= -0.0177) {
              fEE.set(-3,0);
            }
          }
        }
      
        // left of right    
        else if(posEE.x > -0.0102 && posEE.x <= -0.0086){
          if(posEE.y > 0.0374 && posEE.y <= 0.0499){
            if(posEE.x > -0.0102 && posEE.x <= -0.0094) {
              fEE.set(-3,0);
            }
          }
        }

        else if(posEE.y > 0.0780 && posEE.y < 0.0975){
          if(posEE.x > -0.0438 && posEE.x < 0.0172){
            float scale = 10;
            float perlin_noise = noise(posEE.x, posEE.y);
            float magnitude = 5 + perlin_noise * scale + random(-2, 2);
            float direction = perlin_noise * TWO_PI + random(-PI/4, PI/4);
            
            // Create a force vector based on magnitude and direction obtained from the Perlin noise
            float forceX = magnitude * cos(direction);
            float forceY = magnitude * sin(direction);
            PVector forceVector = new PVector(forceX, forceY);
            fEE.set(forceVector);
          }
        }
        
        else {
          fEE.set(0, 0);
        }
        
        lastPosEE = posEE.copy();
      }
      
//PAINTING 2*********************************************************************************************************************     
/* CHARLOTTE ABSTRACT */    
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

//PAINTING 3*******************************************************************************************************   
/* GHAZALEH */
      else if(ffNum == FFNUM.FOUR)  {
        int total = bezierCurve.getVertexCount();
         int total2 = bezierCurve2.getVertexCount();

         double minDistance = 999999;
         float forceX = 0;
         float forceY = 0;
         int mag = -2;
         
         if (posEE.x >= -0.0346 && posEE.x <= 0.0026 && posEE.y <= 0.0517){
           for (int j = 0; j < total; j++) {
            PVector v = bezierCurve.getVertex(j);
            
            float ED = calculateDistance(mapX(posEE.x), mapY(posEE.y), v.x, v.y);

            if (ED < 200){
                if (ED <= minDistance){
                  minDistance = ED;
                  forceX = (v.x - mapX(posEE.x))/((float)Math.pow(ED, 2));
                  forceY = (v.y - mapY(posEE.y))/((float)Math.pow(ED, 2));
                }
              }
           }
         } 
         else if (posEE.x >= 0.005 && posEE.y >= 0.0576){
           for (int j = 0; j < total2; j++){
             PVector v1 = bezierCurve2.getVertex(j);
             float ED = calculateDistance(mapX(posEE.x), mapY(posEE.y), v1.x, v1.y);
            if (ED < 200){
                if (ED <= minDistance){
                  minDistance = ED;
                  forceX = (v1.x - mapX(posEE.x))/((float)Math.pow(ED, 2));
                  forceY = (v1.y - mapY(posEE.y))/((float)Math.pow(ED, 2));
                }
              }
           }
         }
         else{
              float scale = 1;
              float perlin_noise = noise(mapX(posEE.x), mapY(posEE.y));
              float magnitude = perlin_noise * scale;
              float direction = perlin_noise * TWO_PI + random(-PI/4, PI/4);
              forceX = magnitude * cos(direction);
              forceY = magnitude * sin(direction);
              PVector forceVector = new PVector(forceX, forceY);
              //println(forceVector);
              fEE.set(forceVector);
            }         
         fEE.set(forceX * mag, forceY * mag);
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

float calculateDistance(float x1, float y1, float x2, float y2){

  float distance = (float)Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
  
  return distance;
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
