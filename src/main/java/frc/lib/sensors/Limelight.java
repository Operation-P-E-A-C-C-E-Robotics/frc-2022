package frc.lib.sensors;

import org.opencv.core.Point;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
 *Simple class for interfacing with the limelight camera.
*/
public class Limelight {
  public NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private final double tHeight;
  private final double cHeight;
  private final double cAngle;

  private PointTracker raw = new PointTracker(5);
  private PointTracker smoothed = new PointTracker(5);

  /**
   * @param targetHeight the height of the target for calculating distance
   * @param cameraHeight the height of the camera for calculating distance
   */
  public Limelight(double targetHeight, double cameraHeight, double cameraAngle){
    tHeight = targetHeight;
    cHeight = cameraHeight;
    cAngle = cameraAngle;
  }

  public double getTargetHeight(){
    return tHeight;
  }

  public double getCameraHeight(){
    return cHeight;
  }

  public double getCameraAngle(){
    return cAngle;
  }

  /**
   * get a raw table entry from the limelight NetworkTables.
   * @param entry the name of the entry to get
   * @return a double with the value of the entry.
   */
  public double limelightTableValue(String entry){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0);
  }
  /**
   * set a raw limelight NetworkTables value.
   * @param entry the name of the entry
   * @param value the value to set it to.
   */
  public void writeTableValue(String entry, double value){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setNumber(value);
  }
  /**
   * get the distance to the target using the targets apparent angle from the camera and the height of the target and camera.
   * @return the distance in the units the camera height and target height were defined in.
   */
  public double getTargetDistance(){
    double targetOffsetY = getTargetOffsetY();
    double distance = 0;
    distance = (tHeight - cHeight) / Math.tan(Math.toRadians(cAngle) + Math.toRadians(targetOffsetY));
    return distance;
  }

  public void update(){
    // raw.xy(getTargetOffsetX(), getTargetOffsetY());
    // smoothed = raw.smooth(5, 0.1);
    //zoom levels with pipelines
    //TODO change values, X value should be changed so it will only trigger 
    //when the target is low enough
    // if(getTargetOffsetX() < 5 && getTargetOffsetY() < 5){
    //   setPipeline(3);
    // } else if (getTargetOffsetX() < 10 && getTargetOffsetY() < 10){
    //   setPipeline(2);
    // }  else {
    //   setPipeline(0);
    // }


  }
  /**
   * get the targets x offset from the camera
   * @return the angle as a double from -27 to 27 degrees
   */
  public double getTargetOffsetX(){
    return limelightTableValue("tx"); //-27 to 27 degrees
  }
  /**
   * get the targets y offset from the camera
   * @return the angle in degrees.
   */
  public double getTargetOffsetY(){
    return limelightTableValue("ty");
  }

  public double getTargetArea(){
    return limelightTableValue("ta");
  }
  /**
   * the rotation of the fitted bounding box.
   * @return the skew in degrees
   */
  public double skew(){
    return limelightTableValue("ts");
  }
  /**
   * get the length of the shortest side of the fitted bounding box.
   * @return the length in pixels.
   */
  public double getTargetShortSidelength(){ //pixels
    return limelightTableValue("tshort");
  }
  /**
   * get the length of the longest side of the fitted bounding box.
   * @return the length in pixels
   */
  public double getTargetLongSidelength(){ //pixels
    return limelightTableValue("tlong");
  }
  /**
   * get the width of the rough bounding box
   * @return the width in pixels from 0 - 320
   */
  public double getRoughWidth(){ //0-320 pixels
    return limelightTableValue("thor");
  }
  /**
   * get the height of the rough bounding box
   * @return the height in pixels from 0 - 320
   */
  public double getRoughHeight(){ //0 - 320 pixels
    return limelightTableValue("tvert");
  }
  /**
   * whether the limelight is tracking the target
   * @return boolean. true if the limelight has target.
   */
  public double hasTarget() {
    return limelightTableValue("tv");
  }
  /**
   * turn the led's on.
   */
  public void setLedOn(){
    writeTableValue("ledMode", 3);
  }
  /**
   * turn led's off.
   */
  public void setLedOff(){
    writeTableValue("ledMode", 1);
  }
  /**
   * set the led's to blink mode
   * (seizure mode)
   */
  public void setLedBlink(){
    writeTableValue("ledMode", 2);
  }
  /**
   * set the led's to the default value
   */
  public void setLedDefault(){
    writeTableValue("ledMode", 0);
  }
  /**
   * set the camera mode to vision.
   */
  public void setModeVision(){
    writeTableValue("camMode", 0);
  }
  /**
   * set the camera mode to drive. this increases exposure and disables vision tracking
   */
  public void setModeDrive(){
    writeTableValue("camMode", 1);
  }
  /**
   * set the pipeline to filter the image from.
   * @param pipe the pipeline number, from 0 - 9
   */
  public void setPipeline(int pipe){
    writeTableValue("pipeline", pipe);
  }
}