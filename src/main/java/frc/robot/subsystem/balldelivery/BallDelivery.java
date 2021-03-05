/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.balldelivery;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import frc.robot.subsystem.PortMan;

/**
 * Add your docs here.
 */
public class BallDelivery extends SubsystemBase{

    private static Logger logger = Logger.getLogger(BallDelivery.class.getName());
    private WPI_TalonFX shootingMotorLeft; //master
    private WPI_TalonFX shootingMotorRight; //follower
    private WPI_TalonSRX eatingMotor;
    private WPI_TalonSRX carouselMotor;
    private WPI_TalonSRX hoodMotor;

    private DigitalInput stopCarousel;
    private DigitalInput zeroShooter;

    private double pValue;
    private double iValue;
    private double dValue;

    private double targetCarouselVelocity;
    private double targetEatingVelocity;
    private double targetShootingVelocity;
    private double targetHoodPosition;

    private boolean atTargetEatingVel;
    private boolean atTargetShootingVel;

    public double eatingTol;
    public double shootingTol;
    
    public void init(final PortMan portMan) throws Exception {
        logger.info("init");
        shootingMotorLeft = new WPI_TalonFX(portMan.acquirePort(PortMan.can_43_label, "ShootingMotorLeft"));
        shootingMotorRight = new WPI_TalonFX(portMan.acquirePort(PortMan.can_42_label, "ShootingMotorRight"));
        eatingMotor = new WPI_TalonSRX(portMan.acquirePort(PortMan.can_12_label, "EatingMotor"));
        carouselMotor = new WPI_TalonSRX(portMan.acquirePort(PortMan.can_11_label, "CarouselMotor"));
        hoodMotor = new WPI_TalonSRX(portMan.acquirePort(PortMan.can_27_label, "HoodMotor"));
        
        shootingMotorRight.follow(shootingMotorLeft);
        shootingMotorLeft.setInverted(true);

        //shootingMotorLeft.setInverted(false);

        pValue = .4;
        iValue = 0;
        dValue = .2;
        targetCarouselVelocity = 100; 
        targetEatingVelocity = 100;
        targetShootingVelocity = 100;
        targetHoodPosition = 0.0;

        shootingMotorLeft.setNeutralMode(NeutralMode.Coast);
        shootingMotorLeft.configFactoryDefault();
        shootingMotorLeft.configAllowableClosedloopError(0, 5);
        shootingMotorLeft.setSelectedSensorPosition(0, 0, 0);
        shootingMotorLeft.config_kP(0, pValue, 0);
        shootingMotorLeft.config_kI(0, iValue, 0);
        shootingMotorLeft.config_kD(0, dValue, 0);
        shootingMotorLeft.config_kF(0, 0, 0);
        shootingMotorLeft.configClosedloopRamp(.9);

        shootingMotorRight.setNeutralMode(NeutralMode.Coast);
        shootingMotorRight.configFactoryDefault();
        shootingMotorRight.configAllowableClosedloopError(0, 5);
        shootingMotorRight.setSelectedSensorPosition(0, 0, 0);
        shootingMotorRight.config_kP(0, pValue, 0);
        shootingMotorRight.config_kI(0, iValue, 0);
        shootingMotorRight.config_kD(0, dValue, 0);
        shootingMotorRight.config_kF(0, 0, 0);
        shootingMotorRight.configClosedloopRamp(.9);

        eatingMotor.setNeutralMode(NeutralMode.Coast);
        eatingMotor.configFactoryDefault();
        eatingMotor.configAllowableClosedloopError(0, 5);
        eatingMotor.setSelectedSensorPosition(0, 0, 0);
        eatingMotor.config_kP(0, pValue, 0);
        eatingMotor.config_kI(0, iValue, 0);
        eatingMotor.config_kD(0, dValue, 0);
        eatingMotor.config_kF(0, 0, 0);
        eatingMotor.configClosedloopRamp(.9);
        
        carouselMotor.setNeutralMode(NeutralMode.Coast);
        carouselMotor.configFactoryDefault();
        carouselMotor.configAllowableClosedloopError(0, 5);
        carouselMotor.setSelectedSensorPosition(0, 0, 0);
        carouselMotor.config_kP(0, pValue, 0);
        carouselMotor.config_kI(0, iValue, 0);
        carouselMotor.config_kD(0, dValue, 0);
        carouselMotor.config_kF(0, 0, 0);
        carouselMotor.configClosedloopRamp(.9);

        hoodMotor.setNeutralMode(NeutralMode.Brake);
        hoodMotor.configFactoryDefault();
        hoodMotor.configAllowableClosedloopError(0, 5);
        hoodMotor.setSelectedSensorPosition(0, 0, 0);
        hoodMotor.config_kP(0, pValue, 0);
        hoodMotor.config_kI(0, iValue, 0);
        hoodMotor.config_kD(0, dValue, 0);
        hoodMotor.config_kF(0, 0, 0);
        hoodMotor.configClosedloopRamp(.9);
        
    }
    
    //spin the carousel
    public void spinCarousel(){

        logger.info("spin carousel");
        logger.info("spin [" + targetCarouselVelocity + "]");

        //spin carousel
        carouselMotor.set(ControlMode.Velocity, targetCarouselVelocity);
        logger.info("[" + targetCarouselVelocity + "]");

        /*// if switch is triggered, set percent output to 0 to stop spinning
        if(!stopCarousel.get())
        {
            logger.info("stop carousel");
            stopCarousel();
        }*/
    }

    public void stopCarousel()
    {
        carouselMotor.set(ControlMode.PercentOutput, 0);
    }

    //angle the shooter
    public void angleShooter(double pos){
        logger.info("angle shooter");
        logger.info("angle [" + pos + "]");
        
        //set off switch to start (be at bottom)
        //start motors, stop when it's in the right position
    }

    public void eatBall(){
        logger.info("eat ball");
        logger.info("spin green wheels [" + targetEatingVelocity + "]");
        eatingMotor.set(ControlMode.Velocity, targetEatingVelocity);
        logger.info("[" + targetEatingVelocity + "]");
    }

    public void spitOut(){
        logger.info("spit out ball");
        logger.info("spin green wheels [" + - targetEatingVelocity + "]");
        eatingMotor.set(ControlMode.Velocity, - targetEatingVelocity);
        logger.info("[" + - targetEatingVelocity + "]");
    }

    public void stopEating(){
        //stop eating
        logger.info("stop eating");
        eatingMotor.set(ControlMode.PercentOutput, 0);
    }

    public void shootBall(){
        logger.info("shoot ball");
        logger.info("shoot ball [" + targetShootingVelocity + "]");
        shootingMotorLeft.set(ControlMode.Velocity, targetShootingVelocity);
        logger.info("[" + targetShootingVelocity + "]" );
    }

    public void reverseShooter(){
        logger.info("reverse shooter");
        logger.info("reverse shooter [" + - targetShootingVelocity + "]");
        shootingMotorLeft.set(ControlMode.Velocity, - targetShootingVelocity);
        logger.info("[" + - targetShootingVelocity + "]"); 
    }

    public void stopShooting(){
        //stop the shooter
        logger.info("stop shooting");
        shootingMotorLeft.set(ControlMode.PercentOutput, 0);
    }

    public void changePID(double p, double i, double d){
        if(pValue != p)
            pValue = p;
        if(iValue != i)
            iValue = i;
        if(dValue != d)
            dValue = d;
    }

    public double getCurrentCarouselVelocity(){
        return carouselMotor.getSelectedSensorVelocity();
    }

    public double getCurrentEatingVelocity(){
        return eatingMotor.getSelectedSensorVelocity();
    }

    public double getCurrentShootingVelocity(){
        return shootingMotorLeft.getSelectedSensorVelocity();
    }
    
    public double getCurrentHoodPosition(){
        return hoodMotor.getSelectedSensorPosition();
    }

    public double getTargetCarouselVelocity(){
        return targetCarouselVelocity;
    }

    public double getTargetEatingVelocity(){
        return targetEatingVelocity;
    }

    public double getTargetShootingVelocity(){
        return targetShootingVelocity;
    }

    public double getTargetHoodPosition(){
        return targetHoodPosition;
    }

    public double getPValue(){
        return pValue;
    }

    public double getIValue(){
        return iValue;
    }

    public double getDValue(){
        return dValue;
    }

    public double getEatingTolerance(){
        return eatingTol;
    }
    
    public double getShootingTolerance(){
        return shootingTol;
    }

    public void setPValue(double p){
        pValue = p;
    }

    public void setIValue(double i){
        iValue = i;
    }

    public void setDValue(double d){
        dValue = d;
    }

    public void setTargetCarouselVelocity(double vel){
        targetCarouselVelocity = vel;
    }

    public void setTargetEatingVelocity(double vel){
        targetEatingVelocity = vel;
    }

    public void setTargetShootingVelocity(double vel){
        targetShootingVelocity = vel;
    }
    
    public void setTargetHoodPosition(double pos){
        targetHoodPosition = pos;
    }

    public void setEatingTolerance(double tol){
        eatingTol = tol;
    }

    public void setShootingTolerance(double tol){
        shootingTol = tol;
    }

    public boolean isAtShootingVelocity(){
        if(Math.abs(getCurrentShootingVelocity() - targetShootingVelocity) <= shootingTol)
            atTargetShootingVel = true;
        return atTargetShootingVel;
    }

    public boolean isAtEatingVelocity(){
        if(Math.abs(getCurrentEatingVelocity() - targetEatingVelocity) <= eatingTol)
            atTargetEatingVel = true;
        return atTargetEatingVel;
    }

    /*public double getCurrent(){
        //return motor.getSupplyCurrent();
    }*/
}

//comment