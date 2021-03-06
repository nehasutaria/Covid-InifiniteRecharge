package frc.robot.subsystem;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.logging.Logger;

import frc.robot.OI;
import frc.robot.OzoneException;
import frc.robot.subsystem.balldelivery.BallDelivery;
import frc.robot.subsystem.balldelivery.commands.ReverseShooter;
import frc.robot.subsystem.balldelivery.commands.ShootBall;
import frc.robot.subsystem.balldelivery.commands.StopShooting;
import frc.robot.subsystem.balldelivery.commands.StopEating;
import frc.robot.subsystem.balldelivery.commands.EatBalls;
import frc.robot.subsystem.balldelivery.commands.SpinCarousel;
import frc.robot.subsystem.balldelivery.commands.SpitOutBalls;
import frc.robot.subsystem.balldelivery.commands.AngleHood;
import frc.robot.subsystem.balldelivery.commands.DeliverBall;
import frc.robot.subsystem.balldelivery.commands.StopDelivery;
import frc.robot.subsystem.balldelivery.commands.StopCarousel;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.controlpanel.ControlPanel;
import frc.robot.subsystem.controlpanel.commands.RotateToColor;
import frc.robot.subsystem.controlpanel.commands.SpinManual;
import frc.robot.subsystem.controlpanel.commands.SpinRotations;
import frc.robot.subsystem.controlpanel.commands.SpinnerRetract;
import frc.robot.subsystem.controlpanel.commands.SpinnerUp;
import frc.robot.subsystem.controlpanel.commands.Stop;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.commands.IntakeDown;
import frc.robot.subsystem.intake.commands.IntakeSpinBack;
import frc.robot.subsystem.intake.commands.IntakeSpinForward;
import frc.robot.subsystem.intake.commands.IntakeStop;
import frc.robot.subsystem.intake.commands.IntakeUp;
import frc.robot.subsystem.telemetry.Pigeon;
import frc.robot.subsystem.telemetry.Telemetry;
import frc.robot.subsystem.telemetry.commands.ChaseBall;
import frc.robot.subsystem.telemetry.commands.DriveToBall;
import frc.robot.subsystem.telemetry.commands.GoToHorizontalDistance;
import frc.robot.subsystem.telemetry.commands.GoToVerticalDistance;
import frc.robot.subsystem.telemetry.commands.RotateTowardsBall;
import frc.robot.subsystem.telemetry.commands.SquareSelf;
import frc.robot.subsystem.onewheelshooter.OneWheelShooter;
import frc.robot.subsystem.winch.Winch;
import frc.robot.subsystem.winch.commands.WinchUp;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelReverse;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelShoot;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelStop;
import frc.robot.subsystem.pixylinecam.PixyLineCam;
import frc.robot.subsystem.pixylinecam.commands.PollPixyLine;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.climber.commands.Climb;
import frc.robot.subsystem.climber.commands.ClimberControl;
import frc.robot.subsystem.climber.commands.ClimberControlBack;
import frc.robot.subsystem.climber.commands.ClimberRetract;
import frc.robot.subsystem.commandgroups.CollectionMode;
import frc.robot.subsystem.commandgroups.ControlPanelMode;
//import frc.robot.subsystem.commandgroups.ControlPanelMode;
import frc.robot.subsystem.commandgroups.MoveMode;
import frc.robot.subsystem.commandgroups.ScoreLowMode;
import frc.robot.subsystem.commandgroups.SpitBallsMode;
import frc.robot.subsystem.commandgroups.ScoreHighMode;
import frc.robot.subsystem.commandgroups.StartingConfiguration;
//import frc.robot.subsystem.commandgroups.StartingConfiguration;
import frc.robot.subsystem.transport.commands.ScoreLow;
import frc.robot.subsystem.transport.Transport;
import frc.robot.subsystem.transport.commands.*;
import frc.robot.subsystem.transport.commands.TakeIn;
import frc.robot.subsystem.transport.commands.StopTransport;
import frc.robot.subsystem.swerve.DrivetrainSubsystem;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.drivers.Gyroscope;
import frc.common.drivers.NavX;
import edu.wpi.first.wpilibj.SPI;

public class SubsystemFactory {

    private static SubsystemFactory me;

    static Logger logger = Logger.getLogger(SubsystemFactory.class.getName());

    private static String botName;
    private HashMap<String, String> allMACs; // will contain mapping of MACs to Bot Names

    private static DisplayManager displayManager;

    private PowerDistributionPanel pdp;

    /**
     * keep all available subsystem declarations here.
     */

    private Transport transport;
    private ControlPanel controlPanel;
    private Climber climber;
    private OneWheelShooter oneWheelShooter;
    private Telemetry telemetry;
    private BallDelivery ballDelivery;
    private PixyLineCam pixyLineCam;
    private DrivetrainSubsystem2910 driveTrain;
    private Intake intake;
    private Winch winch;
    private Pigeon pigeon;
    private NavX navX;
    
    
    private static ArrayList<SBInterface> subsystemInterfaceList;

    private SubsystemFactory() {
        // private constructor to enforce Singleton pattern
        botName = "unknown";
        allMACs = new HashMap<>();
        // add all the mappings from MACs to names here
        // as you add mappings here:
        // 1) update the select statement in the init method
        // 2) add the init method for that robot
        allMACs.put("00:80:2F:30:DB:F8", "COVID"); // usb0
        allMACs.put("00:80:2F:30:DB:F9", "COVID"); // eth0
        allMACs.put("00:80:2F:28:64:39", "RIO99"); //usb0
        allMACs.put("00:80:2F:28:64:38", "RIO99"); //eth0
        allMACs.put("00:80:2F:27:04:C7", "RIO3"); //usb0 
        allMACs.put("00:80:2F:27:04:C6", "RIO3"); //eth0
        allMACs.put("00:80:2F:17:D7:4B", "RIO2"); //eth0
        allMACs.put("00:80:2F:17:D7:4C", "RIO2"); //usb0
        allMACs.put("00:80:2F:25:B4:CA", "CALIFORNIA"); //usb0
    }

    public static SubsystemFactory getInstance() {

        if (me == null) {
            me = new SubsystemFactory();
        }

        return me;
    }

    public void init(DisplayManager dm, PortMan portMan) throws Exception {

        logger.info("initializing");

        botName = getBotName();

        logger.info("Running on " + botName);

        displayManager = dm;
        subsystemInterfaceList = new ArrayList<SBInterface>();
        pdp = new PowerDistributionPanel(2);

        try {

            // Note that you should update this switch statement as you add bots to the list
            // above
            switch (botName) {
           
            case "RIO3":
                initRIO3(portMan);
                break;
            case "RIO2":
                initRIO2(portMan);
                break;
            case "RIO99":
                initRIO99(portMan);
                break;
            case "COVID":
                initCovid(portMan);
                break;
            case "CALIFORNIA":
                initCalifornia(portMan);
                break;
            default:
                initCovid(portMan); // default to football if we don't know better
            }

            initCommon(portMan);

        } catch (Exception e) {
            throw e;
        }
    }


    /**
     * 
     * init subsystems that are common to all bots
     * 
     */

    private void initCommon(PortMan portMan) {
    }

    public void initCovid(PortMan portMan) throws Exception {
        logger.info("initializing Covid");
        
        navX = new NavX(SPI.Port.kMXP);
        navX.calibrate();
        navX.setInverted(true);
        
        HashMap<String, String> canAssignments = new HashMap<String, String>();
        canAssignments.put("FL.Swerve.angle", PortMan.can_35_label);
        canAssignments.put("FL.Swerve.drive", PortMan.can_34_label);

        canAssignments.put("FR.Swerve.angle", PortMan.can_32_label);
        canAssignments.put("FR.Swerve.drive", PortMan.can_33_label);

        canAssignments.put("BL.Swerve.angle", PortMan.can_36_label);
        canAssignments.put("BL.Swerve.drive", PortMan.can_37_label);

        canAssignments.put("BR.Swerve.angle", PortMan.can_31_label);
        canAssignments.put("BR.Swerve.drive", PortMan.can_30_label);

        double flOff = -Math.toRadians(58.5);
        double frOff = -Math.toRadians(142.6);
        double blOff = -Math.toRadians(318.9);
        double brOff = -Math.toRadians(73.16);

        driveTrain = DrivetrainSubsystem2910.getInstance();
        driveTrain.init(portMan, canAssignments, flOff, blOff, frOff, brOff);
    }

    private void initRIO2(PortMan portMan) throws Exception {
        logger.info("Initializing RIO2");
    }

    private void initRIO3(PortMan portMan ) throws Exception {

        logger.info("initializing RIO3");
        pigeon = new Pigeon(portMan.acquirePort(PortMan.can_21_label, "Pigeon"));

        HashMap<String, String> canAssignments = new HashMap<String, String>();
        canAssignments.put("FL.Swerve.angle", PortMan.can_09_label);
        canAssignments.put("FL.Swerve.drive", PortMan.can_07_label);

        canAssignments.put("FR.Swerve.angle", PortMan.can_03_label);
        canAssignments.put("FR.Swerve.drive", PortMan.can_62_label);

        canAssignments.put("BL.Swerve.angle", PortMan.can_61_label);
        canAssignments.put("BL.Swerve.drive", PortMan.can_11_label);

        canAssignments.put("BR.Swerve.angle", PortMan.can_58_label);
        canAssignments.put("BR.Swerve.drive", PortMan.can_06_label);

        double flOff = -Math.toRadians(1.1);
        double frOff = -Math.toRadians(311.24);
        double blOff = -Math.toRadians(119.6);
        double brOff = -Math.toRadians(262.9);

        driveTrain = DrivetrainSubsystem2910.getInstance();
        driveTrain.init(portMan, canAssignments, flOff, blOff, frOff, brOff);
    }

    private void initCalifornia(PortMan portMan) throws Exception {
        logger.info("Initializing CALIFORNIA");

        navX = new NavX(SPI.Port.kMXP);
        navX.calibrate();
        navX.setInverted(true);
        
        HashMap<String, String> canAssignments = new HashMap<String, String>();
        canAssignments.put("FL.Swerve.angle", PortMan.can_17_label);
        canAssignments.put("FL.Swerve.drive", PortMan.can_06_label);

        canAssignments.put("FR.Swerve.angle", PortMan.can_14_label);
        canAssignments.put("FR.Swerve.drive", PortMan.can_09_label);

        canAssignments.put("BL.Swerve.angle", PortMan.can_15_label);
        canAssignments.put("BL.Swerve.drive", PortMan.can_10_label);

        canAssignments.put("BR.Swerve.angle", PortMan.can_59_label);
        canAssignments.put("BR.Swerve.drive", PortMan.can_60_label);

        double flOff = -Math.toRadians(339.3);
        double frOff = -Math.toRadians(266.9);
        double blOff = -Math.toRadians(61.4);
        double brOff = -Math.toRadians(123.1);

        driveTrain  = DrivetrainSubsystem2910.getInstance();
        driveTrain.init(portMan, canAssignments, flOff, blOff, frOff, brOff);

        /**
         * All of the Telemery Stuff goes here
         */

        telemetry = new Telemetry();
        telemetry.init(portMan);
        displayManager.addTelemetry(telemetry);

        //SquareSelf ccc = new SquareSelf(telemetry, 2.34);
        //OI.getInstance().bind(ccc, OI.LeftJoyButton6, OI.WhenPressed);

        //GoToHorizontalDistance ccd= new GoToHorizontalDistance(telemetry, 2.34);
        //OI.getInstance().bind(ccd, OI.LeftJoyButton7, OI.WhenPressed);

        //GoToVerticalDistance cce = new GoToVerticalDistance(telemetry, 2.34);
        //OI.getInstance().bind(cce, OI.LeftJoyButton10, OI.WhenPressed);

        //DriveToBall ccf = new DriveToBall(telemetry);
        //OI.getInstance().bind(ccf, OI.RightJoyButton11, OI.WhileHeld);

        //RotateTowardsBall ccg = new RotateTowardsBall(telemetry);
        //OI.getInstance().bind(ccg, OI.LeftJoyButton11, OI.WhileHeld);

        //ChaseBall cch = new ChaseBall(telemetry);
        //OI.getInstance().bind(cch, OI.RightJoyButton10, OI.WhileHeld);

        /**
        * shooter stuff goes here
        */
        ballDelivery = new BallDelivery();
        ballDelivery.init(portMan);
        displayManager.addBallDelivery(ballDelivery);

        //ShootBall cci = new ShootBall(ballDelivery);
        //OI.getInstance().bind(cci, OI.LeftJoyButton7, OI.WhenPressed);

        ReverseShooter ccj = new ReverseShooter(ballDelivery);
        OI.getInstance().bind(ccj, OI.LeftJoyButton10, OI.WhenPressed);
        
        StopShooting cck = new StopShooting(ballDelivery);
        OI.getInstance().bind(cck, OI.LeftJoyButton11, OI.WhenPressed);

        EatBalls ccl = new EatBalls(ballDelivery);
        OI.getInstance().bind(ccl, OI.RightJoyButton11, OI.WhenPressed);

        StopEating ccm = new StopEating(ballDelivery);
        OI.getInstance().bind(ccm, OI.RightJoyButton10, OI.WhenPressed);

        SpitOutBalls ccn = new SpitOutBalls(ballDelivery);
        OI.getInstance().bind(ccn, OI.RightJoyButton3, OI.WhenPressed);

        SpinCarousel cco = new SpinCarousel(ballDelivery);
        OI.getInstance().bind(cco, OI.RightJoyButton6, OI.WhenPressed);

        AngleHood ccp = new AngleHood(ballDelivery);
        OI.getInstance().bind(ccp, OI.RightJoyButton4, OI.WhenPressed);

        StopCarousel ccq = new StopCarousel(ballDelivery);
        OI.getInstance().bind(ccq, OI.RightJoyButton7, OI.WhenPressed);

        DeliverBall ccr = new DeliverBall(ballDelivery);
        OI.getInstance().bind(ccr, OI.LeftJoyButton6, OI.WhenPressed);
        
        StopDelivery ccs = new StopDelivery(ballDelivery);
        OI.getInstance().bind(ccs, OI.LeftJoyButton7, OI.WhenPressed);
    }

    private void initRIO99(PortMan portMan) throws Exception {
        logger.info("Initializing RIO99");
        
    }


    public PowerDistributionPanel getPDP(){
        return pdp;
    }
    public ControlPanel getControlPanel() {
        return controlPanel;
    }
    public DrivetrainSubsystem2910 getDriveTrain(){
        return driveTrain;
    }
    public Climber getClimber() {
        return climber;
    }

    public Transport getTransport() {
        return transport;
    }
    public Intake getIntake(){
        return intake;
    }
    public OneWheelShooter getShooter(){
        return oneWheelShooter;
    }

    public Gyroscope getGyro() {
        if(pigeon != null) {
            return pigeon;
        } else {
            return navX;
        }
    }

    private String getBotName() throws Exception {

        Enumeration<NetworkInterface> networks;
            networks = NetworkInterface.getNetworkInterfaces();

            String activeMACs = "";
            for (NetworkInterface net : Collections.list(networks)) {
                String mac = formatMACAddress(net.getHardwareAddress());
                activeMACs += (mac+" ");
                logger.info("Network #"+net.getIndex()+" "+net.getName()+" "+mac);
                if (allMACs.containsKey(mac)) {
                    botName = allMACs.get(mac);
                    logger.info("   this MAC is for "+botName);
                }
            }

            return botName;
        }

    /**
     * Formats the byte array representing the mac address as more human-readable form
     * @param hardwareAddress byte array
     * @return string of hex bytes separated by colons
     */
    private String formatMACAddress(byte[] hardwareAddress) {
        if (hardwareAddress == null || hardwareAddress.length == 0) {
            return "";
        }
        StringBuilder mac = new StringBuilder(); // StringBuilder is a premature optimization here, but done as best practice
        for (int k=0;k<hardwareAddress.length;k++) {
            int i = hardwareAddress[k] & 0xFF;  // unsigned integer from byte
            String hex = Integer.toString(i,16);
            if (hex.length() == 1) {  // we want to make all bytes two hex digits 
                hex = "0"+hex;
            }
            mac.append(hex.toUpperCase());
            mac.append(":");
        }
        mac.setLength(mac.length()-1);  // trim off the trailing colon
        return mac.toString();
    }

}