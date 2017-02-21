package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3238.robot.Autonomous.*;

import java.util.ArrayList;

public class Robot extends IterativeRobot implements PIDOutput
{
    private Chassis chassis;
    private Collector collector;
    private Climber climber;
    private Shooter shooter;
    private AHRS navX;
    private CANTalon leftLeader, rightLeader;
    SubsystemPhaser SubPhaser;
    double[][] boilerStraightFirst, boilerStraightSecond, centerLift, retrievalStraightFirst,
    retrievalStraightSecond;
    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();
    PIDController turnController;
    boolean e = false;
    int auto_selection = 0;
    Phaser auto;
    MotionProfileExample leftController, rightController;
    public static final double kP = 0.05,
                                kI = 0.0,
    kD = 0.02,
            kF = 0.0;
    int  controlStatus = 0,
            controlCalls = 0;
    public static final double fTalon = 0.415, pTalon = 3.0, kToleranceDegrees = 2.0;
    double rotateToAngleRate = 0.0;
    private int codesPerRev = 360;
    boolean notShooting = true;
    @Override public void robotInit()
    {
        auto = new Phaser();
        leftLeader = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
        CANTalon leftFollower = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
        rightLeader = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
        CANTalon rightFollower = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
        CANTalon leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
        CANTalon rightCollect = new CANTalon(Constants.Collector.RIGHT_TALON_ID);
        CANTalon liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
        CANTalon climbTalonOne = new CANTalon(Constants.Climber.CLIMB_TALON_ONE_ID);
        CANTalon climbTalonTwo = new CANTalon(Constants.Climber.CLIMB_TALON_TWO_ID);
        CANTalon agitatorTalon = new CANTalon(Constants.Shooter.AGITATOR_TALON_ID);
        CANTalon shooterTalon = new CANTalon(Constants.Shooter.SHOOTER_TALON_ID);
        Joystick joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);
        
        climber = new Climber(climbTalonOne, climbTalonTwo, joystick);
        chassis = new Chassis(leftLeader, rightLeader,  joystick);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                joystick);
        leftFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftFollower.set(leftLeader.getDeviceID());
        rightFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightFollower.set(rightLeader.getDeviceID());
        navX = new AHRS(SPI.Port.kMXP);

        leftLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftLeader.configEncoderCodesPerRev(codesPerRev);
        rightLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightLeader.configEncoderCodesPerRev(codesPerRev);
        turnController = new PIDController(kP, kI, kD, kF, navX, this);
        turnController.setInputRange(-180.0f, 180.0f);

        turnController.setOutputRange(-0.51625, 0.51625);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        boilerStraightFirst = Profiles.boilerForwardFirst.Points;
        boilerStraightSecond = Profiles.boilerForwardSecond.Points;
        setEncoderInversions();
        leftController = new MotionProfileExample(leftLeader);
        rightController = new MotionProfileExample(rightLeader);
        shooter = new Shooter(agitatorTalon, shooterTalon, joystick);
        setEncoderInversions();
        resetEncoderPosition();
        CameraServer.getInstance().startAutomaticCapture();
        SubPhaser = new SubsystemPhaser(collector, shooter);
    }
    
    @Override public void disabledPeriodic()
    {
        resetEncoderPosition();
//        DriverStation.reportError("Nav : " + navX.getAngle(), false);
        auto_selection = Preferences.getInstance().getInt("auto", 2 );
        SmartDashboard.putNumber("Auto Found", auto_selection);
        notShooting = true;
    }
    int boilerChoice = 0;
    @Override public void autonomousInit()
    {
//        auto_selection = Preferences.getInstance().getInt("auto", 3 );
        DriverStation.reportError("selected: " + auto_selection, false);
        boilerChoice = 0;
        switch(auto_selection) {
            default:
                break;
            case 1: //Only Retrieval Side Peg
                setSideSpeedProfileRet();
                break;
            case 2: //Only Center Peg
                setCenterProfile();
                break;
            case 3: //Only Boiler Peg
                setSideSpeedProfile();
                break;
            case 4: //Center Peg and Boiler Shoot
                setCenterProfile();
                boilerChoice = 1;
                break;
            case 5: //Boiler Peg and Boiler Shoot
                setSideSpeedProfile();
                boilerChoice = 2;
                break;
            case 6: //WIP only approach
                setNZMoveProfile();
                break;
            case 7:
                setNZEndProfile();
                break;
        }
//        setSideSpeedProfileRet();
        prepEncodersForProfile();
        navX.zeroYaw();
        doneO = false;
        e = false;
        c = false;
    d=false;
    count = 0;
    }
    boolean doneO = false, d = false, c = false, b = false;
    String status = "0";
    int count = 0;
    ArrayList<String> flow= new ArrayList<String>();

    @Override public void autonomousPeriodic()
    {
    }
    
    @Override public void teleopInit()
    {
        collector.init();
        setMotorsDriveMode();
    }
    
    @Override public void teleopPeriodic()
    {
        chassis.proRun();
        collector.run();
        climber.run();
        shooter.run();
        DriverStation.reportError("Left Enc Position: " + leftLeader.getEncPosition()
        + "\nRight Enc Position: " + rightLeader.getEncPosition(), false);
    }


    boolean gear = false;
    @Override public void testInit()
    {
        auto.PhaseCollection.clear();
        switch(auto_selection) {
            case Constants.Autonomous.BOILERSIDESHOOT: // BoilerSideLift
                auto.addPhase(new Phase(Profiles.leftBoiler.Points, Profiles.rightBoiler.Points, true, Phase.NONE));
                auto.addPhase(new Phase(Profiles.leftSideBoilerShot.Points, Profiles.rightSideBoilerShot.Points, false, Phase.REVSHOOTGEAR));
                auto.addPhase(new Phase(Profiles.leftSideBoilerShot.Points, Profiles.rightSideBoilerShot.Points, false, Phase.SHOOT));
                break;
            case Constants.Autonomous.BOILERSIDE:
                auto.addPhase(new Phase(Profiles.leftBoiler.Points, Profiles.rightBoiler.Points, true, Phase.NONE));

                auto.addPhase(new Phase(Profiles.leftBack.Points, Profiles.rightBack.Points, true, Phase.PLACEGEAR));

                break;

            case Constants.Autonomous.CENTER:
                auto.addPhase(new Phase(Profiles.centerLift.Points, Profiles.centerLift.Points, true, Phase.NONE));

                auto.addPhase(new Phase(Profiles.leftBack.Points, Profiles.rightBack.Points, true, Phase.PLACEGEAR));
                break;

            case Constants.Autonomous.CENTERSHOOT:
                auto.addPhase(new Phase(Profiles.centerLift.Points, Profiles.centerLift.Points, true, Phase.NONE));

                auto.addPhase(new Phase(Profiles.leftCenterBoilerShot.Points, Profiles.rightCenterBoilerShot.Points, true, Phase.REVSHOOTGEAR));
                auto.addPhase(new Phase(Profiles.leftCenterBoilerShot.Points, Profiles.rightCenterBoilerShot.Points, false, Phase.SHOOT));



                break;

            case Constants.Autonomous.RETRIEVALSIDE:
                auto.addPhase(new Phase(Profiles.leftBoiler.Points, Profiles.rightBoiler.Points, false, Phase.NONE));
                auto.addPhase(new Phase(Profiles.leftBack.Points, Profiles.rightBack.Points, true, Phase.PLACEGEAR));

                break;

            case Constants.Autonomous.RETRIEVALNEUTRALZONE:
                auto.addPhase(new Phase(Profiles.leftBoiler.Points, Profiles.rightBoiler.Points, false, Phase.NONE));

                auto.addPhase(new Phase(Profiles.leftNZ.Points, Profiles.rightNZ.Points, false, Phase.PLACEGEAR));
                auto.addPhase(new Phase(Profiles.finishNZL.Points, Profiles.finishNZR.Points, false, Phase.NONE));

                break;
        }

        initMotionProfile();
    }
    @Override public void testPeriodic()
    {
        if(auto.run(motionProfileLoop())) setNewMotionProfile();
        SubPhaser.run(auto.getCurrentPhase());
        notShooting = auto.getCurrentPhase().subsystemProperty != Phase.SHOOT;

    }//the greatest line ever


    @Override
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
    Notifier _notifer = new Notifier(new PeriodicRunnable());
    class PeriodicRunnable implements java.lang.Runnable
    {
        public void run()
        {
            leftLeader.processMotionProfileBuffer();
            rightLeader.processMotionProfileBuffer();
        }
    }
    public void pollProfileStatus()
    {
        leftLeader.getMotionProfileStatus(_status);
        rightLeader.getMotionProfileStatus(_status);
    }
    public boolean motionProfileLoop()
    {
        pollProfileStatus();
        DriverStation.reportWarning(
                "auto periodic: MP status = " + _status.toString(), false);
        if(notShooting)
        switch(controlStatus)
        {
            case 0:
                DriverStation.reportWarning("auto case 0", false);
                _notifer.startPeriodic(0.005);
                leftController.fillFed();
                rightController.fillFed();
                leftLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                rightLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                controlStatus++;
                return e;
            default:
                DriverStation.reportWarning("auto case 1", false);
                // _talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                if(!e)
                {

                    double motorOutput = leftLeader.getOutputVoltage()
                            / leftLeader.getBusVoltage();
                    double altMotorOutput = rightLeader.getOutputVoltage()
                            / rightLeader.getBusVoltage();
                    System.out.println("motor output: " + motorOutput);
                    System.out.println("motor speed: " + leftLeader.getSpeed());
                    System.out.println("error in native units: "
                            + leftLeader.getClosedLoopError());
                }


                if(_status.activePointValid && _status.activePoint.isLastPoint)
                {
                    SmartDashboard.putNumber("Loops", controlCalls);
                    DriverStation.reportWarning(
                            "auto case default: profile done", false);
                    leftLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    rightLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    DriverStation.reportWarning("# auto loops: " + controlCalls,
                            false);
                    System.out.println(leftLeader.getPosition());
                    System.out.println(rightLeader.getPosition());
                    e = true;
                }

                controlCalls++;

                return e;
        }
        else {
            leftLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        }
        return false;
    }
    public void resetDiagnostics()
    {
        controlStatus = 0;
        controlCalls = 0;
        e = false;
    }
    public void prepEncodersForProfile()
    {
        resetEncoderPosition();
        enableMotionProfileMode();
        setEncoderInversions();
        setEncoderPID();
        resetDiagnostics();
    }
    public void resetEncoderPosition()
    {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }
    public void enableMotionProfileMode()
    {
        leftLeader.clearMotionProfileTrajectories();
        leftLeader.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
        leftLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        leftLeader.changeMotionControlFramePeriod(5);
        rightLeader.clearMotionProfileTrajectories();
        rightLeader.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
        rightLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        rightLeader.changeMotionControlFramePeriod(5);

    }
    public void setEncoderInversions()
    {
        rightLeader.reverseOutput(false);
        leftLeader.reverseSensor(false);
        rightLeader.reverseSensor(true);
        leftLeader.reverseOutput(true);
    }
    public void setEncoderPID()
    {
        leftLeader.setF(fTalon);
        leftLeader.setP(pTalon);
        leftLeader.setI(0.0);
        rightLeader.setP(pTalon);
        leftLeader.setD(0.0);
        rightLeader.setF(fTalon);
        rightLeader.setI(0.0);
        rightLeader.setD(0.0);
    }
    public void setLayeredProfile(int n) {
        if(n == 1) {
            leftController.setFedProfile(boilerStraightFirst);

            rightController.setFedProfile(boilerStraightFirst);
        } else if(n == 2) {
            leftController.setFedProfile(boilerStraightSecond);


            rightController.setFedProfile(boilerStraightSecond);
        }
    }
    public void setMotorsDriveMode()
    {
        leftLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    }

    public void setCenterLiftProfile() {
        leftController.setFedProfile(centerLift);
        rightController.setFedProfile(centerLift);

    }
    public void setRetrievalProfile() {
        leftController.setFedProfile(retrievalStraightFirst);
        rightController.setFedProfile(retrievalStraightSecond);
        navX.zeroYaw();
    }
    public void setSideSpeedProfile() {
        leftController.setFedProfile(Profiles.rightBoiler.Points);
        rightController.setFedProfile(Profiles.leftBoiler.Points);
    }
    public void setSideSpeedProfileRet() {
        leftController.setFedProfile(Profiles.leftBoiler.Points);
        rightController.setFedProfile(Profiles.rightBoiler.Points);

    }
    public void setCenterProfile() {
        leftController.setFedProfile(Profiles.centerLift.Points);

        rightController.setFedProfile(Profiles.centerLift.Points);
    }
    public void setSideBoilerShotProfile() {
        leftController.setFedProfile(Profiles.leftSideBoilerShot.Points);
        rightController.setFedProfile(Profiles.rightSideBoilerShot.Points);
    }
    public void setCenterBoilerShotProfile() {
        leftController.setFedProfile(Profiles.leftSideBoilerShot.Points);
        rightController.setFedProfile(Profiles.rightSideBoilerShot.Points);
    }
    public void setNZMoveProfile() {
        leftController.setFedProfile(Profiles.leftNZ.Points);
        rightController.setFedProfile(Profiles.rightNZ.Points);
    }
    public void setNZEndProfile() {
        leftController.setFedProfile(Profiles.finishNZL.Points);

        rightController.setFedProfile(Profiles.finishNZR.Points);
    }

    public void initMotionProfile() {
        setNewMotionProfile();
    }
    public void setNewMotionProfile() {
        e = false;
        prepEncodersForProfile();
//        try {
        if(auto.PhaseCollection.size() > 0) {
            leftController.setFedProfile(auto.getCurrentProfileLeft());
            rightController.setFedProfile(auto.getCurrentProfileRight());
        } else {
            notShooting = false;
        }
    }
}

