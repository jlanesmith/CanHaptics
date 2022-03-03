/**
 **********************************************************************************************************************
 * @file       sketch_2_Hello_Wall.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       08-January-2021
 * @brief      Wall haptic example with programmed physics for a haptic wall 
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
import oscP5.*;
import netP5.*;
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
boolean           renderingForce                     = true;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;


/* end effector radius in meters */
float             rEE                                 = 0.006;

/* virtual wall parameter  */
PVector           fWall                               = new PVector(0, 0);


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
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;

/* Jonathan's stuff */
OscP5 oscP5;
NetAddress myRemoteLocation;

MusicString[] strings = new MusicString[34];
float pianoTop = 0.06;
float pianoMid = 0.095;
float pianoBottom = 0.12;
String pitchLetter = "";
float pitchMIDI = 60;
float pitchFreq = 0;
boolean continuousPitch = true; 

String[] topRowLetters = {"C", "C#", "D", "Eb", "E", "F", "F#", "G", "G#", "A", "Bb", "B", "C", "C#", "D", "Eb", "E"};
float topRowBounds[] = {-0.1, -0.087, -0.075, -0.065, -0.053, -0.04, -0.028, -0.016, -0.006, 0.006, 0.016, 0.028, 0.04, 0.053, 0.065, 0.075, 0.087, 0.1};
String[] bottomRowLetters = {"C", "D", "E", "F", "G", "A", "B", "C", "D", "E"};
float bottomRowBounds[] = {-0.1, -0.08, -0.06, -0.04, -0.02, 0, 0.02, 0.04, 0.06, 0.08, 0.1};
float[] bottomRowMidi = {60, 62, 64, 65, 67, 69, 71, 72, 74, 76};

/* end elements definition *********************************************************************************************/ 



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  
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
    
  
  /* visual elements setup */
  background(0);
  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create pantagraph graphics */
  create_pantagraph();
  
  /* create string graphics */

  strings[0] = new MusicString(true, pianoTop, pianoBottom, -0.1);
  strings[1] = new MusicString(true, pianoMid, pianoBottom, -0.08);
  strings[2] = new MusicString(true, pianoMid, pianoBottom, -0.06);
  strings[3] = new MusicString(true, pianoTop, pianoBottom, -0.04);
  strings[4] = new MusicString(true, pianoMid, pianoBottom, -0.02);
  strings[5] = new MusicString(true, pianoMid, pianoBottom, 0);
  strings[6] = new MusicString(true, pianoMid, pianoBottom, 0.02);
  strings[7] = new MusicString(true, pianoTop, pianoBottom, 0.04);
  strings[8] = new MusicString(true, pianoMid, pianoBottom, 0.06);
  strings[9] = new MusicString(true, pianoMid, pianoBottom, 0.08);
  strings[10] = new MusicString(true, pianoTop, pianoBottom, 0.1);
  strings[11] = new MusicString(false, -0.1, 0.1, pianoTop);
  strings[12] = new MusicString(false, -0.1, 0.1, pianoBottom);
  
  strings[13] = new MusicString(true, pianoTop, pianoMid, -0.087);
  strings[14] = new MusicString(true, pianoTop, pianoMid, -0.075);
  strings[15] = new MusicString(true, pianoTop, pianoMid, -0.065);
  strings[16] = new MusicString(true, pianoTop, pianoMid, -0.053);
  strings[17] = new MusicString(false, -0.087, -0.075, pianoMid);
  strings[18] = new MusicString(false, -0.065, -0.053, pianoMid);

  strings[19] = new MusicString(true, pianoTop, pianoMid, -0.028);
  strings[20] = new MusicString(true, pianoTop, pianoMid, -0.016);
  strings[21] = new MusicString(true, pianoTop, pianoMid, -0.006);
  strings[22] = new MusicString(true, pianoTop, pianoMid, 0.006);
  strings[23] = new MusicString(true, pianoTop, pianoMid, 0.016);
  strings[24] = new MusicString(true, pianoTop, pianoMid, 0.028);
  strings[25] = new MusicString(false, -0.028, -0.016, pianoMid);
  strings[26] = new MusicString(false, -0.006, 0.006, pianoMid);
  strings[27] = new MusicString(false, 0.016, 0.028, pianoMid);
  
  strings[28] = new MusicString(true, pianoTop, pianoMid, 0.087);
  strings[29] = new MusicString(true, pianoTop, pianoMid, 0.075);
  strings[30] = new MusicString(true, pianoTop, pianoMid, 0.065);
  strings[31] = new MusicString(true, pianoTop, pianoMid, 0.053);
  strings[32] = new MusicString(false, 0.087, 0.075, pianoMid);
  strings[33] = new MusicString(false, 0.065, 0.053, pianoMid);
  
  oscP5 = new OscP5(this, 57120);
  myRemoteLocation = new NetAddress("127.0.0.1", 57120);

  
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
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
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
      posEE.set(device_to_graphics(posEE)); 
            
      /* haptic wall force calculation */
      fWall.set(0, 0);
      
      boolean justMoved = false;
            
      /* String calculations */
      for (MusicString string : strings) {
        
        float mainPos = string.isVertical ? posEE.x : posEE.y;
        float lateralPos = string.isVertical ? posEE.y : posEE.x;
        
        // For switching between above and below string
        if (string.aboveString && mainPos > string.location + string.strumDistance/2) {
          string.aboveString = false;
          if (lateralPos > (string.lowEnd-0.0) && lateralPos < (string.highEnd+0.01)) { 
            // Adding the 0.01 makes it more reliable, but there are still sometimes bugs when switching to another note
            justMoved = true; 
          }
        } else if (!string.aboveString && mainPos < string.location - string.strumDistance/2) {
          string.aboveString = true;
          if (lateralPos > (string.lowEnd-0.01) && lateralPos < (string.highEnd+0.01)) { 
            justMoved = true; 
          }        
        }
        
        // For generating force
        float force = 0;
        if(string.aboveString && mainPos > string.location - string.strumDistance && lateralPos > string.lowEnd && lateralPos < string.highEnd){
          force = (mainPos - string.location + string.strumDistance) * string.thickness;  
        }
        else if (!string.aboveString && mainPos < string.location + string.strumDistance && lateralPos > string.lowEnd && lateralPos < string.highEnd) {
          force = (string.location + string.strumDistance - mainPos) * -string.thickness;  
        }
        fWall = fWall.add(new PVector(string.isVertical ? force : 0, string.isVertical ? 0 : force));
      }
 //<>//
      fEE = (fWall.copy()).mult(-1);
      fEE.set(graphics_to_device(fEE));
      /* end haptic wall force calculation */
      
      // Calculate pitch
      if (justMoved) { // If just moved across a boundary
        if (posEE.y < pianoTop || posEE.y > pianoBottom) { // If above or below the keyboard
          continuousPitch = true;
        } else {
          continuousPitch = false;
          if (posEE.y > pianoTop && posEE.y < pianoMid) { // Black and white keys
            int i = 0;
            while (posEE.x > topRowBounds[i]) {i++;}
            pitchLetter = topRowLetters[i-1];
            pitchMIDI = 60 + i - 1;
          } else if (posEE.y > pianoMid && posEE.y < pianoBottom) { // Just white keys
            int i = 0;
            while (posEE.x > bottomRowBounds[i]) {i++;}
            pitchLetter = bottomRowLetters[i-1];
            pitchMIDI = bottomRowMidi[i-1];
          }
        }
      }
      if (continuousPitch) {
        pitchLetter = "";
        pitchMIDI = 60.4 + (posEE.x + 0.09)/0.18*15.3;
      }
      pitchFreq = pow(2,(pitchMIDI-69)/12) * 440; // Convert MIDI note number into frequency
      OscMessage myMessage1 = new OscMessage("/hapstrument");
      myMessage1.add(pitchFreq); 
      oscP5.send(myMessage1, myRemoteLocation);
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_pantagraph(){
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  float rEEAni = pixelsPerMeter * rEE;
  
  pGraph = createShape();
  pGraph.beginShape();
  pGraph.fill(255);
  pGraph.stroke(0);
  pGraph.strokeWeight(2);
  
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.vertex(deviceOrigin.x, deviceOrigin.y);
  pGraph.endShape(CLOSE);
  
  joint = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, rEEAni, rEEAni);
  joint.setStroke(color(0));
  
  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y, 2*rEEAni, 2*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}

PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  
  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}


void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  
  float lAni = pixelsPerMeter * l;
  float LAni = pixelsPerMeter * L;
  
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  pGraph.setVertex(1, deviceOrigin.x + lAni*cos(th1), deviceOrigin.y + lAni*sin(th1));
  pGraph.setVertex(3, deviceOrigin.x + lAni*cos(th2), deviceOrigin.y + lAni*sin(th2));
  pGraph.setVertex(2, deviceOrigin.x + xE, deviceOrigin.y + yE);
  
  shape(pGraph);
  shape(joint);
  for (MusicString string : strings) {
    shape(string.wall);
  }
  
  textSize(70);
  fill(1);
  text(pitchLetter, 750, 150);
  textSize(50);
  text("Midi:", 80, 100);
  text("Freq:", 80, 150);
  text((int)pitchMIDI, 220, 100);
  text((int)pitchFreq, 220, 150);

  translate(xE, yE);
  shape(endEffector);
}


PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

/* end helper functions section ****************************************************************************************/



class MusicString {
  boolean aboveString = true;
  float strumDistance = 0.005;
  boolean isVertical = false;
  float lowEnd = -0.2;
  float highEnd = 0.2;
  int thickness = 400; // 200 for twang, 300 for more plucky
  float location;
  PShape wall;
 
  MusicString (boolean vertical, float stringLowEnd, float stringHighEnd, float stringLocation) {
    isVertical = vertical;
    lowEnd = stringLowEnd;
    highEnd = stringHighEnd;
    location = stringLocation;
    if (isVertical)
        wall = create_wall(location, lowEnd, location, highEnd);
    else
        wall = create_wall(lowEnd, location, highEnd, location);
 
    wall.setStroke(color(0));
  }
}
 
