/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



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
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

/* define maze blocks */
FBox              b1;
FBox              b2;
FBox              b3;
FBox              b4;
FBox              b5;
FBox              b6;
FBox              b7;
FBox              b8;
FBox              b9;
FBox              b10;
FBox              b11;
FBox              b12;
FBox              b13;
FBox              b14;
FBox              b15;
FBox              b16;
FBox              b17;
FBox              b18;
FBox              l1;

/* define start and stop button */
FCircle           c1;
FCircle           c2;

/* define game ball */
FCircle           g2;
FBox              g1;

/* define game start */
boolean           gameStart                           = false;

/* text font */
PFont             f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
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
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  /* Set maze barriers */
  b1                  = new FBox(3.0, 1.0);
  b1.setPosition(edgeTopLeftX+2.27, edgeTopLeftY+3.5); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
  
  b2                  = new FBox(1.0, 5.2);
  b2.setPosition(edgeTopLeftX+5.5, edgeTopLeftY+3.37); 
  b2.setFill(0);
  b2.setNoStroke();
  b2.setStaticBody(true);
  world.add(b2);
   
  b3                  = new FBox(3.0, 1.0);
  b3.setPosition(edgeTopLeftX+3.5, edgeTopLeftY+5.46); 
  b3.setFill(0);
  b3.setNoStroke();
  b3.setStaticBody(true);
  world.add(b3);
  
  b4                  = new FBox(5.0, 1.0);
  b4.setPosition(edgeTopLeftX+4.5, edgeTopLeftY+7.5); 
  b4.setFill(0);
  b4.setNoStroke();
  b4.setStaticBody(true);
  world.add(b4);
   
  b5                  = new FBox(1.0, 6.0);
  b5.setPosition(edgeTopLeftX+7.5, edgeTopLeftY+5);
  b5.setFill(0);
  b5.setNoStroke();
  b5.setStaticBody(true);
  world.add(b5);
  
  b6                  = new FBox(3.0, 1.0);
  b6.setPosition(edgeTopLeftX+9.5, edgeTopLeftY+2.5);
  b6.setFill(0);
  b6.setNoStroke();
  b6.setStaticBody(true);
  world.add(b6);
  
  b7                  = new FBox(4.5, 1.0);
  b7.setPosition(edgeTopLeftX+10.25, edgeTopLeftY+5);
  b7.setFill(0);
  b7.setNoStroke();
  b7.setStaticBody(true);
  world.add(b7);
  
  b8                  = new FBox(1.0, 2.5);
  b8.setPosition(edgeTopLeftX+10.5, edgeTopLeftY+6.5);
  b8.setFill(0);
  b8.setNoStroke();
  b8.setStaticBody(true);
  world.add(b8);
  
  b9                  = new FBox(1.0, 2.5);
  b9.setPosition(edgeTopLeftX+9.5, edgeTopLeftY+8);
  b9.setFill(0);
  b9.setNoStroke();
  b9.setStaticBody(true);
  world.add(b9);
  
  b10                 = new FBox(3.0, 1.0);
  b10.setPosition(edgeTopLeftX+14, edgeTopLeftY+2.5);
  b10.setFill(0);
  b10.setNoStroke();
  b10.setStaticBody(true);
  world.add(b10);
  
  b11                 = new FBox(1.0, 4.72);
  b11.setPosition(edgeTopLeftX+16, edgeTopLeftY+3.13); 
  b11.setFill(0);
  b11.setNoStroke();
  b11.setStaticBody(true);
  world.add(b11);
  
  b12                 = new FBox(3.0, 1.0);
  b12.setPosition(edgeTopLeftX+15, edgeTopLeftY+5);
  b12.setFill(0);
  b12.setNoStroke();
  b12.setStaticBody(true);
  world.add(b12);
  
  b13                 = new FBox(1.0, 2.5);
  b13.setPosition(edgeTopLeftX+14, edgeTopLeftY+6.5);
  b13.setFill(0);
  b13.setNoStroke();
  b13.setStaticBody(true);
  world.add(b13);
  
  b14                 = new FBox(7.5, 1.0);
  b14.setPosition(edgeTopLeftX+15.75, edgeTopLeftY+7.5); 
  b14.setFill(0);
  b14.setNoStroke();
  b14.setStaticBody(true);
  world.add(b14);
  
  b15                 = new FBox(1.0, 6.0);
  b15.setPosition(edgeTopLeftX+21.25, edgeTopLeftY+5);
  b15.setFill(0);
  b15.setNoStroke();
  b15.setStaticBody(true);
  world.add(b15);
  
  b16                = new FBox(1.0, 4.0);
  b16.setPosition(edgeTopLeftX+18.5, edgeTopLeftY+4);
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b17                 = new FBox(4.0, 1.0);
  b17.setPosition(edgeTopLeftX+21, edgeTopLeftY+2.5);
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b18                 = new FBox(3.0, 1.0);
  b18.setPosition(edgeTopLeftX+22.75, edgeTopLeftY+6.5);
  b18.setFill(0);
  b18.setNoStroke();
  b18.setStaticBody(true);
  world.add(b18);
  
  /* Start Button */
  c1                  = new FCircle(1.0); // diameter is 2
  c1.setPosition(edgeTopLeftX+2.5, edgeTopLeftY+worldHeight/2.0-3);
  c1.setFill(0, 255, 0);
  c1.setStaticBody(true);
  world.add(c1);
  
  /* Finish Button */
  c2                  = new FCircle(1.0);
  c2.setPosition(worldWidth-2.5, edgeTopLeftY+worldHeight/2.0);
  c2.setFill(200,0,0);
  c2.setStaticBody(true);
  c2.setSensor(true);
  world.add(c2);
  
  /* Game Box */
  g1                  = new FBox(1, 1);
  g1.setPosition(2, 4);
  g1.setDensity(80);
  g1.setFill(random(255),random(255),random(255));
  g1.setName("Widget");
  //world.add(g1);
  
  /* Game Ball */
  g2                  = new FCircle(1);
  g2.setPosition(3, 4);
  g2.setDensity(80);
  g2.setFill(random(255),random(255),random(255));
  g2.setName("Widget");
  //world.add(g2);
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  

 
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
 
    if(gameStart){
      fill(0, 0, 0);
    
      b1.setFill(0, 0, 0);
      b2.setFill(0, 0, 0);
      b3.setFill(0, 0, 0);
      b4.setFill(0, 0, 0);
      b5.setFill(0, 0, 0);
      b6.setFill(0, 0, 0);
      b7.setFill(0, 0, 0);
      b8.setFill(0, 0, 0);
      b9.setFill(0, 0, 0);
      b10.setFill(0, 0, 0);
      b11.setFill(0, 0, 0);
      b12.setFill(0, 0, 0);
      b13.setFill(0, 0, 0);
      b14.setFill(0, 0, 0);
      b15.setFill(0, 0, 0);
      b16.setFill(0, 0, 0);
      b17.setFill(0, 0, 0);
      b18.setFill(0, 0, 0);
    
    }
    else{
      fill(128, 128, 128);
      textAlign(CENTER);
      text("Touch the green circle", width/2, 60);
    
      b1.setFill(255, 255, 255);
      b2.setFill(255, 255, 255);
      b3.setFill(255, 255, 255);
      b4.setFill(255, 255, 255);
      b5.setFill(255, 255, 255);
      b6.setFill(255, 255, 255);
      b7.setFill(255, 255, 255);
      b8.setFill(255, 255, 255);
      b9.setFill(255, 255, 255);
      b10.setFill(255, 255, 255);
      b11.setFill(255, 255, 255);
      b12.setFill(255, 255, 255);
      b13.setFill(255, 255, 255);
      b14.setFill(255, 255, 255);
      b15.setFill(255, 255, 255);
      b16.setFill(255, 255, 255);
      b17.setFill(255, 255, 255);
      b18.setFill(255, 255, 255);
    }
  
    world.draw();
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
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (s.h_avatar.isTouchingBody(c1)){
      gameStart = true;
      c1.setSensor(true);
      g1.setPosition(2,8);
      g2.setPosition(3,8);
      s.h_avatar.setSensor(false);
    }
  
    if(g1.isTouchingBody(c2) || g2.isTouchingBody(c2)){
      gameStart = false;
      s.h_avatar.setSensor(true);
    }
  
  
  
    /* Viscous layer codes */
    if (s.h_avatar.isTouchingBody(l1)){
      s.h_avatar.setDamping(400);
    }
    else{
      s.h_avatar.setDamping(10); 
    }
  
    if(gameStart && g1.isTouchingBody(l1)){
      g1.setDamping(20);
    }
  
    if(gameStart && g2.isTouchingBody(l1)){
      g2.setDamping(20);
    }
  
  
    /* Bouyancy of fluid on avatar and gameball section */
    if (g1.isTouchingBody(l1)){
      float b_s;
      float bm_d = g1.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water
    
      if (bm_d + g1.getWidth()/2 >= g1.getWidth()) { //if whole ball or more is submerged
        b_s = g1.getWidth(); // amount of ball submerged is ball size
      }
      else { //if ball is partially submerged
        b_s = bm_d + g1.getWidth()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
      }
  
      g1.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
   
    }
  
    if (g2.isTouchingBody(l1)){
      float b_s;
      float bm_d = g2.getY()-l1.getY()+l1.getHeight()/2; // vertical distance between middle of ball and top of water
    
      if (bm_d + g2.getSize()/2 >= g2.getSize()) { //if whole ball or more is submerged
        b_s = g2.getSize(); // amount of ball submerged is ball size
      }
      else { //if ball is partially submerged
        b_s = bm_d + g2.getSize()/2; // amount of ball submerged is vertical distance between middle of ball and top of water + half of ball size
      }
  
      g2.addForce(0,l1.getDensity()*sq(b_s)*gravityAcceleration*-1); // 300 is gravity force
     
    }
    /* End Bouyancy of fluid on avatar and gameball section */
  
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* Alternate bouyancy of fluid on avatar and gameball helper functions, comment out
 * "Bouyancy of fluid on avatar and gameball section" in simulation and uncomment 
 * the helper functions below to test
 */
 
/*
void contactPersisted(FContact contact){
  float size;
  float b_s;
  float bm_d;
  
  if(contact.contains("Water", "Widget")){
    size = 2*sqrt(contact.getBody2().getMass()/contact.getBody2().getDensity()/3.1415);
    bm_d = contact.getBody2().getY()-contact.getBody1().getY()+l1.getHeight()/2;
    
    if(bm_d + size/2 >= size){
      b_s = size;
    }
    else{
      b_s = bm_d + size/2;
    }
    
    contact.getBody2().addForce(0, contact.getBody1().getDensity()*sq(b_s)*300*-1);
    contact.getBody2().setDamping(20);
  }
  
}


void contactEnded(FContact contact){
  if(contact.contains("Water", "Widget")){
    contact.getBody2().setDamping(0);
  }
}
*/

/* End Alternate Bouyancy of fluid on avatar and gameball helper functions */

/* end helper functions section ****************************************************************************************/
