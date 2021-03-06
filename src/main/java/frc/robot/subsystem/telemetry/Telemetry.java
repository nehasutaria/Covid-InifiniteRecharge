
/**
 * two lidars
 * one boolean to see if the robot is parallel at a certain distance
 * 
 * boolean isSquare(double distance)
 * 
 */

package frc.robot.subsystem.telemetry;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.PortMan;
import java.util.logging.Logger;

import javax.lang.model.util.ElementScanner6;

public class Telemetry extends SubsystemBase{
    
    private LidarPWM frontLidar, rearLidar, backLidar;
    private double frontLidarDistance, rearLidarDistance, backLidarDistance;
    
    /*
    private double frontLidarOffset = 0;
    private double rearLidarOffset = 0;
    private double backLidarOffset = 0;
    */

    // "calibration" of lidars (based on margin of error)
    private double frontCalibration;
    private double rearCalibration;
    private double frontShort, frontMed;   //front ranges 
    private double rearShort, rearMed;   //rear ranges

    private static Logger logger = Logger.getLogger(Telemetry.class.getName());

    private double betweenLidarDistance = 0;
    private double lidarTolerance = 1.0;
    private double correction = Math.PI/180;
    private MedianFilter filterFront;
    private MedianFilter filterRear;
    private MedianFilter filterBack;

    private int horDirection = 0;
    private int verticalDirection = 0;
    private double rotationalSpeed = 0.1;
    private double translationalSpeed = 0.1;

    //targetDistance is the distance away from the wall
    private double horizontalTargetDistance = 100;
    private double verticalTargetDistance = 100;

    private String ballLocation; //left right or center
    private int ballDirection; //1 -1 or 0
    private double ballDistance;
    private double targetBallDistance;
    private boolean seeBall;

    public Telemetry() {
    }
    
    public void init(PortMan portMan) throws Exception{
        logger.entering(Telemetry.class.getName(), "init()");

        frontLidar = new LidarPWM(portMan.acquirePort(PortMan.digital1_label, "Telemetry.frontLidar"));
        rearLidar = new LidarPWM(portMan.acquirePort(PortMan.digital7_label, "Telemetry.rearLidar"));
        backLidar = new LidarPWM(portMan.acquirePort(PortMan.digital6_label, "Telemetry.backLidar"));
        filterFront = new MedianFilter(10);
        filterRear = new MedianFilter(10);
        filterBack = new MedianFilter(10);

        CameraServer.getInstance().startAutomaticCapture();
        //CameraServer.getInstance().startAutomaticCapture();

        logger.exiting(Telemetry.class.getName(), "init()");
    }

    public boolean isSquare(double tolerance){
        if (Math.abs(getFrontLidarDistance() - getRearLidarDistance()) <= tolerance)
            return true;
        else
            return false;
    }

    public int whereAmI(){
    {
        //multiplies speed by the value that is returned to set direction of rotation
        frontLidarDistance = Math.round(frontLidar.getDistance()) * 1.0;
        rearLidarDistance = Math.round(rearLidar.getDistance()) * 1.0;
            if (frontLidarDistance > rearLidarDistance && !isSquare(lidarTolerance)){
                //rotate left
                return 1;
            } else if(frontLidarDistance < rearLidarDistance && !isSquare(lidarTolerance)){
                //rotate right
                return -1;
            } else {
                //already square
                return 0;
            }
        }     
    }

    public int directionToGo(){
        frontLidarDistance = Math.round(frontLidar.getDistance()) * 1.0;
        if(Math.abs(frontLidarDistance - horizontalTargetDistance) < lidarTolerance){
            //already at target
            return 0;
        }
        else if(frontLidarDistance - horizontalTargetDistance > 0){
            //go left
            return 1;
        }
        else{
            //go right
            return -1;
        }
    }

    public int verticalDirectionToGo(){
        backLidarDistance = Math.round(backLidar.getDistance()) * 1.0;
        if(Math.abs(backLidarDistance - verticalTargetDistance) < lidarTolerance){
            //already at target
            return 0;
        }
        else if(backLidarDistance - verticalTargetDistance > 0){
            //go back
            return -1;
        }
        else{
            //go forward
            return 1;
        }
    }
    
    private void setFrontCal(){
        //returns value to be added to front lidar's calculated distance
        if(filterFront.calculate(frontLidar.getDistance()) <= frontShort){
            frontCalibration = -10.0;
        } 
        else if(filterFront.calculate(frontLidar.getDistance()) <= frontMed){
            frontCalibration = -9.0;
        } 
        else if(filterFront.calculate(frontLidar.getDistance()) > frontMed){
            frontCalibration = -8.0;
        }
        else{
            frontCalibration = -10000; //if all of those are false, something must be wrong, so this will let us know
        }

    }

    private void setRearCal(){
        //returns value to be added to rear lidar's calculated distance
        if(filterRear.calculate(rearLidar.getDistance()) <= rearShort){
            rearCalibration = -10.0;
        }
        else if(filterRear.calculate(rearLidar.getDistance()) <= rearMed){
            rearCalibration = -9.0;
        }
        else if(filterRear.calculate(rearLidar.getDistance()) > rearMed){
            rearCalibration = -8.0;
        }
        else{
            rearCalibration = -10000; //if all of those are false, something must be wrong, so this will let us know
        }
    }

    public double getFrontLidarDistance(){
        if (frontLidar == null)
            return 0.0;
        return Math.round(filterFront.calculate(frontLidar.getDistance())) * 1.0; // + frontCalibration
    }

    public double getRearLidarDistance(){
        if (rearLidar == null)
            return 0.0;
        return Math.round(filterRear.calculate(rearLidar.getDistance())) * 1.0; // + rearCalibration
    }

    public double getBackLidarDistance(){
        if(backLidar == null)
            return 0.0;
        return Math.round(filterBack.calculate(backLidar.getDistance())) * 1.0;
    }

    public int getBallDirection(){
        if(ballLocation.equals("left"))
            ballDirection = -1;
        else if(ballLocation.equals("right"))
            ballDirection = 1;
        else if(ballLocation.equals("center"))
            ballDirection = 0;

        return ballDirection;
    }

    public double getTolerance(){
        return lidarTolerance;
    }

    public void setTolerance(double tol){
        lidarTolerance = tol;
    }

    public double getBetweenLidar(){
        return betweenLidarDistance;
    }

    public void setHorDirection(int dir){
        horDirection = dir;
    }

    public void setHorizontalTargetDistance(double dis)
    {
        horizontalTargetDistance = dis;
    }

    public void setVerticalTargetDistance(double dist)
    {
        verticalTargetDistance = dist;
    }

    public void setVerticalDirection(int dire)
    {
        verticalDirection = dire;
    }

    public void setRotationalSpeed(double sp)
    {
        rotationalSpeed = - sp;
    }

    public double getRotationalSpeed()
    {
        if(ballDirection == 0)
            rotationalSpeed = 0;
        return rotationalSpeed;
    }

    public void setTranslationalSpeed(double speed)
    {
        translationalSpeed = speed;
    }

    public double getTranslationalSpeed()
    {
        if(ballDistance <= targetBallDistance)
            translationalSpeed = 0;
        return translationalSpeed;
    }

    public void setBallDirection(String bd)
    {
        ballLocation = bd;
    }

    public void setBallDistance(double bd)
    {
        ballDistance = bd;
    }

    public double getBallDistance() {
        return ballDistance;
    }

    public void setSeeBall(boolean sb)
    {
        seeBall = sb;
    }

    public boolean getSeeBall() {
        return seeBall;
    }

    public void setTargetBallDistance(double distance) {
        targetBallDistance = distance;
    }

    public double getTargetBallDistance() {
        return targetBallDistance;
    }
}