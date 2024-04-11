
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

/* elements definition *************************************************************************************************/
HVirtualCoupling  s;

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
float             min_y                               = 0.;
float             max_y                               = 0.1;

//scaling factors for tangential force application for height based forces
float scalingFactorX = 2;
float scalingFactorY = 2;

//Centers of circles for large bumpy texture
PVector           center                              = new PVector(min_x + (max_x - min_x)/2, min_y + (max_y - min_y)/2);      
PVector           center2                             = new PVector(-0.05, 0.02);
PVector           center3                             = new PVector(0.05, 0.07);
PVector           center4                             = new PVector(-0.04, 0.04);
PVector           center5                             = new PVector(0.05, 0.07);
PVector           center6                             = new PVector(-0.02, 0.05);
PVector           center7                             = new PVector(0.08, 0.03);
PVector           center8                             = new PVector(0.04, 0.03);
PVector           center9                             = new PVector(-0.05, 0.07);

float[] xPositions = {-0.04, 0.0, 0.04}; // Adjust positions along x plane
float[] yPositions = {0.03, 0.05, 0.07}; // Adjust positions along y plane

// Define the coefficients of the polynomial function (e.g., ax^2 + bx + c)
float a = 1.0;
float b = 2.0;
float c = 3.0;

float endEffectorX;
float endEffectorY; 
 
// Define the distance threshold for applying force
double threshold;

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
  FIVE,
  SIX,
};

FFNUM ffNum = FFNUM.ONE;
PFont f;

//Image objects
PImage bumpImage;
PImage bumpImage2;
PImage pyramids;
PImage edges;
PImage flower;
PImage painting2;

float startXX = 100;
float endXX = 200;
float y = 400; // The y-coordinate remains constant for a horizontal line

/* end elements definition *********************************************************************************************/  

/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  /* screen size definition */
  size(1000, 700);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);
  
  /* device setup */
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  
  haplyBoard          = new Board(this, Serial.list()[0], 0);
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
  
  bumpImage = loadImage("images/bump.jpg"); // Replace "bump_image.jpg" with the actual path to your bump image
  bumpImage2 = loadImage("images/bump.jpg"); // Replace "bump_image.jpg" with the actual path to your bump image
  pyramids = loadImage("images/pyramids.JPG");
  edges = loadImage("images/edges.png");
  flower = loadImage("images/flower.jpg");
 
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
      text("Instructions:\nEach of the 5 experiences below represents a texture. \nThe textures are as follows:\n1. Rough \n2. Large bumps \n3. Harsh edges \n4. Rough (less granular) \n5. Initial exploration of tracing a curved line \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress '1' for the first experience. \nPress '2' for the second experience and so on... ", 100, 100);
      fill(#000000);
      text("Current mode:", 300, 300);
      fill(#000000);
    } else if(ffNum == FFNUM.TWO) {
      text("Second mode", 500, 300);
    } else if(ffNum == FFNUM.FOUR) {
      text("Fourth mode", 500, 300);
    } else if(ffNum == FFNUM.FIVE) {
      text("Fifth mode", 500, 300);
    } else if(ffNum == FFNUM.SIX) {
      text("Sixth mode", 500, 300);
      text("*Haptic Experience Window*", 100, 75);
      fill(#000000);
      text("Instructions:\nEach of the 5 experiences below represents a texture. \nThe textures are as follows:\n1. Rough \n2. Large bumps \n3. Harsh edges \n4. Rough (less granular) \n5. Initial exploration of tracing a curved line \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress '1' for the first experience. \nPress '2' for the second experience and so on... ", 100, 100);
      fill(#000000);
      text("Current mode:", 300, 300);
      fill(#000000);
    } else {
      text("Third mode", 500, 300);
    }
  
    // Show end effector marker and hide initial text when in bump environment
    if(ffNum == FFNUM.FOUR) {
      float scaleFactor = 0.3; // Adjust as needed
      
      // Calculate the scaled width and height of the image
      float scaledWidth = bumpImage.width * scaleFactor;
      float scaledHeight = bumpImage.height * scaleFactor;
      
      image(pyramids, map(-0.05,-0.079, 0.071, 0, 1000), map(0.02, 0, 0.1, 0, 700), scaledWidth, scaledHeight);
      fill(#000000);
      text("Current mode: Fourth mode", 100, 100);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    }
  
    if(ffNum == FFNUM.THREE) {
      float scaleFactor = 0.5; // Adjust as needed
      
      // Calculate the scaled width and height of the image
      float scaledWidth = bumpImage.width * scaleFactor;
      float scaledHeight = bumpImage.height * scaleFactor;
      
      //this make suse of an outdated mapping function, I wouldn't suggest using this one
      image(edges, mapX(-0.05), mapY(0.033), scaledWidth, scaledHeight);
      fill(#000000);
      text("Current mode: Third mode", 100, 100);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    }
  
    // Show end effector marker and hide initial text when in bump environment
    if(ffNum == FFNUM.TWO) {
  
      float scaleFactor = 0.18; // Adjust as needed
      
      // Calculate the scaled width and height of the image
      float scaledWidth = bumpImage.width * scaleFactor;
      float scaledHeight = bumpImage.height * scaleFactor;
      
      //image(bumpImage, mapX(-0.024), mapY(0.033), scaledWidth, scaledHeight);
      image(bumpImage, map(-0.029,-0.079, 0.071, 0, 1000), map(0.03, 0, 0.1, 0, 700), scaledWidth, scaledHeight); 
      println("BUMP");
      fill(#000000);
      text("Current mode: Second mode", 100, 100);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
    }
  
    if (ffNum == FFNUM.FIVE) {
      float scaleFactor = 0.6; // Adjust as needed
      
      // Calculate the scaled width and height of the image
      float scaledWidth = flower.width * scaleFactor;
      float scaledHeight = flower.height * scaleFactor;
      
      image(flower, map(-0.05,-0.079, 0.071, 0, 1000), map(0.01, 0, 0.1, 0, 700), scaledWidth, scaledHeight);
      ellipse(mapX(posEE.x), mapY(posEE.y), 10, 10);
      stroke(0); // Set stroke color to black
      strokeWeight(2); // Set stroke weight
      line(startXX, y+50, endXX, y+50);
      line(100, y, 200, y);
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
     
/* SNAG GRID  **************************************************************************************************/ 
      if (ffNum == FFNUM.ONE) {
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
      
/* Large bumps  **************************************************************************************************/      
      else if (ffNum == FFNUM.TWO) {
        println(posEE);
      
        // Define center points
        PVector[] centerPoints = {center};//, center2, center3, center4, center5, center6, center7, center8, center9};
        
        // Define distances
        float[] distances = new float[1];
        
        // Calculate distances from each center point
        for (int i = 0; i < centerPoints.length; i++) {
            PVector dist = PVector.sub(posEE, centerPoints[i]);
            distances[i] = dist.mag();
        }
    
        float rad = 0.017f; // Radius of the circular region
        float force_scale = 2.0f; // Scale factor for force
        boolean insideCircularRegion = false;
    
        // Iterate over distances and check if the object is inside any circular region
        for (int i = 0; i < distances.length; i++) {
            if (distances[i] <= rad) {
                println("INSIDE");
                // Inside the circular region
                fEE = PVector.mult(PVector.sub(posEE, centerPoints[i]).normalize(), force_scale);
                insideCircularRegion = true;
                break; // Exit the loop once inside a circular region
            }
          }
      
        // If the object is outside all circular regions, no force is exerted
        if (!insideCircularRegion) {
            fEE.set(0, 0);
        }
      }    
        
/* SNAG  **************************************************************************************************/      
      else if(ffNum == FFNUM.THREE) {
          if(posEE.x > -0.06 && posEE.x <= -0.05){
            if(posEE.x > -0.06 && posEE.x <= -0.055) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          else if(posEE.x > -0.04 && posEE.x <= -0.03){
            if(posEE.x > -0.04 && posEE.x <= -0.035) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          
           else if(posEE.x > -0.02 && posEE.x <= -0.01){
            if(posEE.x > -0.02 && posEE.x <= -0.015) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          
          else if(posEE.x > -0.01 && posEE.x <= 0){
            if(posEE.x > -0.01 && posEE.x <= -0.005) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          else if(posEE.x > 0.01 && posEE.x <= 0.02){
            if(posEE.x > 0.01 && posEE.x <= 0.015) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          else if(posEE.x > 0.03 && posEE.x <= 0.04){
            if(posEE.x > 0.03 && posEE.x <= 0.035) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          else if(posEE.x > 0.05 && posEE.x <= 0.06){
            if(posEE.x > 0.03 && posEE.x <= 0.055) {
            fEE.set(-700,0);
            }
            else{
            fEE.set(700,0);
            }
          }
          else{
          fEE.set(0,0);
          }
        } 
  
/* PYRAMID GRID  **************************************************************************************************/      
      else if(ffNum == FFNUM.FOUR)  { 
        boolean insideAnyPyramid = false;
      
        // Define the range of x and y coordinates for the grid
        float minX = -0.07f;
        float maxX = 0.05f;
        float minY = 0.0f;
        float maxY = 0.08f;
          
        // Define the spacing between pyramids in the x and y directions
        float spacingX = 0.01f;
        float spacingY = 0.01f;
          
        // Calculate the number of pyramids in the x and y directions
        int numPyramidsX = (int) ((maxX - minX) / spacingX) + 1;
        int numPyramidsY = (int) ((maxY - minY) / spacingY) + 1;
        
        // Initialize array to store center points
        PVector[] centerPoints = new PVector[numPyramidsX * numPyramidsY];
        
        // Calculate center points of pyramids
        int index = 0;
        for (float x = minX; x <= maxX; x += spacingX) {
            for (float y = minY; y <= maxY; y += spacingY) {
                centerPoints[index++] = new PVector(x, y);
            }
        }
          
        // Define the size of the square-based pyramid (half of the side length)
        float halfSideLength = 0.005f;
        
        // Scale factor for force
        float forceScale = 500.0f;
        
        // Initialize total force vector
        PVector totalForce = new PVector();
        
        // Check if the end-effector is inside any square-based pyramid
        for (int i = 0; i < centerPoints.length; i++) {
            if (Math.abs(centerPoints[i].x - posEE.x) <= halfSideLength && Math.abs(centerPoints[i].y - posEE.y) <= halfSideLength) {
                // Inside the square-based pyramid
                PVector direction = PVector.sub(posEE, centerPoints[i]);
                fEE.set(direction.x * forceScale, direction.y * forceScale);
                insideAnyPyramid = true;
                break; // Exit the loop once inside a pyramid
            }
        }
        if (!insideAnyPyramid) {
            fEE.set(0, 0);
        }
    }
  
//CONTOUR FOLLOWING*********************************************************************************************************************
      else if(ffNum == FFNUM.FIVE) {
        println(posEE);     
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
          fEE.set(0, 0); // No force if end-effector is not close to the polynomial function
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
  } else if(key == '5') {
    ffNum = FFNUM.FIVE;
  } else if(key == '6') {
    ffNum = FFNUM.SIX;
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
