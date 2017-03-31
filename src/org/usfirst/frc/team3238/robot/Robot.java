package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.BaseSystemNotInitializedException;
import org.usfirst.frc.team3238.robot.Autonomous.*;
import org.usfirst.frc.team3238.robot.Autonomous.Profiles.*;

import java.util.ArrayList;

public class Robot extends IterativeRobot {
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
    Joystick joy1 = new Joystick(0);
    boolean RedSide = false;
    MotionProfileExample leftController, rightController;
    public static final double kP = 0.05,
            kI = 0.0,
            kD = 0.02,
            kF = 0.0;
    int controlStatus = 0,
            controlCalls = 0;
    public static final double fTalon = 0.415, pTalon = 3.0, kToleranceDegrees = 2.0;
    double rotateToAngleRate = 0.0;
    private int codesPerRev = 360;
    boolean shooterDisabled = false;
    boolean talonsDisabled = false;
    String selectedAuto;
    SendableChooser<String> autoChooser = new SendableChooser<>();

    @Override
    public void robotInit() {
        autoChooser.addDefault(Constants.Autonomous.DISABLED, Constants.Autonomous.DISABLED);
        addChooserString(Constants.Autonomous.BOILER_GEAR);
        addChooserString(Constants.Autonomous.BOILER_SHOOT);
        addChooserString(Constants.Autonomous.CENTER_GEAR);
        addChooserString(Constants.Autonomous.CENTER_SHOOT);
        addChooserString(Constants.Autonomous.RETRIEVAL_GEAR);
        addChooserString(Constants.Autonomous.RETRIEVAL_RUN);
        addChooserString(Constants.Autonomous.MLG_HOPPER);
        addChooserString(Constants.Autonomous.SHOOT_THEN_GEAR);
        SmartDashboard.putData("Auto Selection", autoChooser);
        auto = new Phaser();
        CANTalon leftFollower = null;
        CANTalon rightFollower = null;
        CANTalon leftCollect = null;
        CANTalon rightCollect = null;
        CANTalon liftCollect = null;
        CANTalon climbTalonOne = null;
        CANTalon climbTalonTwo = null;
        CANTalon agitatorTalon = null;
        CANTalon shooterTalon = null;
        try {
            leftLeader = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
            leftFollower = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
            rightLeader = new CANTalon(
                    Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
            rightFollower = new CANTalon(
                    Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
            leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
            rightCollect = new CANTalon(Constants.Collector.RIGHT_TALON_ID);
            liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
            climbTalonOne = new CANTalon(Constants.Climber.CLIMB_TALON_ONE_ID);
            climbTalonTwo = new CANTalon(Constants.Climber.CLIMB_TALON_TWO_ID);
            agitatorTalon = new CANTalon(Constants.Shooter.AGITATOR_TALON_ID);
            shooterTalon = new CANTalon(Constants.Shooter.SHOOTER_TALON_ID);
        } catch (Exception e1) {
            DriverStation.reportError("CHECK YOUR CAN BUS, DRIVERS!!!!!!!!!!!!!!!!!!!!!!!!!!\n" + e1.getMessage(), false);
        }
        DriverStation.reportWarning("" + shooterTalon.getIZone(), false);
        if (shooterTalon.GetFirmwareVersion() == 0.0) {
            shooterDisabled = true;
            DriverStation.reportWarning("Shooter is not on the robot", false);
        } else {
            DriverStation.reportWarning("Shooter is detected", false);
        }
        if (leftLeader.GetFirmwareVersion() == 0.0 || rightLeader.GetFirmwareVersion() == 0.0 || leftFollower.GetFirmwareVersion() == 0.0 || rightFollower.GetFirmwareVersion() == 0.0) {
            DriverStation.reportError("AT LEAST ONE OF YOUR DRIVE TALONS IS DEAD!!!!!!!!!!!!!!!", false);
            throw new BaseSystemNotInitializedException("Drive Talons Disabled");
        }
        Joystick joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);

        climber = new Climber(climbTalonOne, climbTalonTwo, joystick);
        chassis = new Chassis(leftLeader, rightLeader,  joystick);
        chassis.setShooterDisabled(shooterDisabled);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                joystick);
        leftFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftFollower.set(leftLeader.getDeviceID());
        rightFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightFollower.set(rightLeader.getDeviceID());
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch(Exception e) {}
        leftLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftLeader.configEncoderCodesPerRev(codesPerRev);
        rightLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightLeader.configEncoderCodesPerRev(codesPerRev);
        boilerStraightFirst = HardcodedProfiles.boilerForwardFirst.Points;
        boilerStraightSecond = HardcodedProfiles.boilerForwardSecond.Points;
        setEncoderInversions();
        leftController = new MotionProfileExample(leftLeader);
        rightController = new MotionProfileExample(rightLeader);
        shooter = new Shooter(agitatorTalon, shooterTalon, joystick);
        setEncoderInversions();
        resetEncoderPosition();
        CameraServer.getInstance().startAutomaticCapture();
//        CameraServer.getInstance().
        SubPhaser = new SubsystemPhaser(collector, shooter);
        SmartDashboard.putNumber("auto", auto_selection);
        SmartDashboard.putBoolean("Red Side", false);
    }

    public void addChooserString(String string) {
        autoChooser.addObject(string, string);
    }

    @Override public void disabledPeriodic()
    {
        SmartDashboard.putString("Alliance", DriverStation.getInstance().getAlliance().toString());

        if(talonsDisabled)
            System.exit(0);
        if(shooterDisabled)
            DriverStation.reportWarning("No Shooter", false);
        shooter.init();
        resetEncoderPosition();
        RedSide = DriverStation.getInstance().getAlliance().toString().equals("Red");
        selectedAuto = autoChooser.getSelected();
        SmartDashboard.putString("Auto Selected", selectedAuto);
    }
public static void say(String s) {
        DriverStation.reportError(s, false);
}
    int boilerChoice = 0;

    @Override
    public void autonomousInit() {
        selectedAuto = autoChooser.getSelected();
        say("Your auto is " + selectedAuto);

        chassis.init();
        auto.PhaseCollection.clear();
        auto.skipCollection.clear();
        SubPhaser.calledCollect = false;

        switch (selectedAuto)
        {
            case Constants.Autonomous.DISABLED:
                setupDisabledAuto();
                break;
            case Constants.Autonomous.BOILER_GEAR:
                setupBoilerGear();
                break;
            case Constants.Autonomous.BOILER_SHOOT:
                if(RedSide)
                {
                    setupBoilerShootRed();
                } else  {
                    setupBoilerShootBlue();
                }
                break;
            case Constants.Autonomous.CENTER_GEAR:
                setupCenterGear();
                break;
            case Constants.Autonomous.CENTER_SHOOT:
                if(RedSide)
                {
                    setupCenterShootRed();
                } else  {
                    setupCenterShootBlue();
                }
                break;
            case Constants.Autonomous.RETRIEVAL_GEAR:
                setupRetrievalGear();
                break;
            case Constants.Autonomous.RETRIEVAL_RUN:
                setupRetrievalRun();
                break;
            case Constants.Autonomous.MLG_HOPPER:
                if(RedSide)
                {
                    setupMLGHopperRed();
                } else
                {
                    setupMLGHopperBlue();
                }
                break;
            case Constants.Autonomous.SHOOT_THEN_GEAR:
                if(RedSide)
                {
                    setupShootThenGearRed();
                } else
                {
                    setupShootThenGearBlue();
                }
                break;
        }
        initMotionProfile();
    }

    public void setupDisabledAuto() {
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, false, Phase.NONE));
    }

    public void setupShootThenGearRed() {
        auto.addPhase(new Phase(normalStartToSideLiftWithStopToShootRED.DriveToShotLocationPoints_LEFT_RED, normalStartToSideLiftWithStopToShootRED.DriveToShotLocationPoints_RIGHT_RED, true, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.QUICKSHOT, 4));
        auto.addPhase(new Phase(normalStartToSideLiftWithStopToShootRED.ContinueToLiftPoints_LEFT_RED, normalStartToSideLiftWithStopToShootRED.ContinueToLiftPoints_RIGHT_RED, true, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, true, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupShootThenGearBlue() {
        auto.addPhase(new Phase(normalStartToSideLiftWithStopToShootBLUE.DriveToShotLocationPoints_LEFT_BLUE, normalStartToSideLiftWithStopToShootBLUE.DriveToShotLocationPoints_RIGHT_BLUE, false, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, false, Phase.QUICKSHOT, 4));
        auto.addPhase(new Phase(normalStartToSideLiftWithStopToShootBLUE.ContinueToLiftPoints_LEFT_BLUE, normalStartToSideLiftWithStopToShootBLUE.ContinueToLiftPoints_RIGHT_BLUE, false, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, false, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupRetrievalRun() {
        auto.addPhase(new Phase(HardcodedProfiles.leftBoiler.Points, HardcodedProfiles.rightBoiler.Points, !RedSide, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftNZ.Points, HardcodedProfiles.rightNZ.Points, !RedSide, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.finishNZL.Points, HardcodedProfiles.finishNZR.Points, !RedSide, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupRetrievalGear() {
        auto.addPhase(new Phase(HardcodedProfiles.leftBoiler.Points, HardcodedProfiles.rightBoiler.Points, !RedSide, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, RedSide, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupCenterShootBlue() {
        auto.addPhase(new Phase(HardcodedProfiles.centerLift.Points, HardcodedProfiles.centerLift.Points, false, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, false, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);

        auto.addPhase(new Phase(centerShootForwardLEFT.Points, centerShootForwardRIGHT.Points, false, Phase.REVSHOOT));
        auto.addPhase(new Phase(centerShootCurveBlueLEFT.Points, centerShootCurveBlueRIGHT.Points, false, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, false, Phase.QUICKSHOT, 10.0));
    }

    public void setupCenterShootRed() {
        auto.addPhase(new Phase(HardcodedProfiles.centerLift.Points, HardcodedProfiles.centerLift.Points, true, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points,true, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
        auto.addPhase(new Phase(centerShootForwardLEFT.Points, centerShootForwardRIGHT.Points, true, Phase.REVSHOOT));
        auto.addPhase(new Phase(centerShootCurveRedLEFT.Points, centerShootCurveRedRIGHT.Points, true, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.QUICKSHOT, 10.0));
    }

    public void setupCenterGear() {
        auto.addPhase(new Phase(HardcodedProfiles.centerLift.Points, HardcodedProfiles.centerLift.Points, RedSide, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, RedSide, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupBoilerShootBlue() {
        auto.addPhase(new Phase(HardcodedProfiles.leftBoiler.Points, HardcodedProfiles.rightBoiler.Points, false, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.sideBoilerShotBlueLeft.Points, HardcodedProfiles.sideBoilerShotBlueRight.Points, true, Phase.REVSHOOTGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.sideBoilerShotBlueLeft.Points, HardcodedProfiles.sideBoilerShotBlueRight.Points, true, Phase.QUICKSHOT));
    }

    public void setupBoilerGear() {
        auto.addPhase(new Phase(HardcodedProfiles.leftBoiler.Points, HardcodedProfiles.rightBoiler.Points, RedSide, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, RedSide, Phase.PLACEGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
    }

    public void setupBoilerShootRed() {
        auto.addPhase(new Phase(HardcodedProfiles.leftBoiler.Points, HardcodedProfiles.rightBoiler.Points, RedSide, Phase.NONE));
        auto.addPhase(new Phase(HardcodedProfiles.leftSideBoilerShot.Points, HardcodedProfiles.rightSideBoilerShot.Points, RedSide, Phase.REVSHOOTGEAR));
        auto.addPhase(new Phase(HardcodedProfiles.leftSideBoilerShot.Points, HardcodedProfiles.rightSideBoilerShot.Points, RedSide, Phase.QUICKSHOT));
    }

    public void setupMLGHopperRed() {
        auto.addPhase(new Phase(curveHitLEFT.Points, curveHitRIGHT.Points, true, Phase.NONE,2.0));
        auto.addPhase(new Phase(hopperHitBackLEFT.Points, hopperHitBackRIGHT.Points, true, Phase.NONE));
        auto.addPhase(new Phase(hopperShootTurnRedLEFT.Points, hopperShootTurnRIGHT.Points, true, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.QUICKSHOT, 10.0));
    }

    public void setupMLGHopperBlue() {
        auto.addPhase(new Phase(curveHitLEFT.Points, curveHitRIGHT.Points, false, Phase.NONE,2.0));
        auto.addPhase(new Phase(hopperHitBackLEFT.Points, hopperHitBackRIGHT.Points, false, Phase.NONE));
        auto.addPhase(new Phase(hopperShootTurnBlueLEFT.Points, hopperShootTurnBlueRIGHT.Points, false, Phase.REVSHOOT));
        auto.addPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, false, Phase.QUICKSHOT, 10.0));
    }

    boolean doneO = false, d = false, c = false, b = false;
    String status = "0";
    int count = 0;
    ArrayList<String> flow= new ArrayList<String>();

    @Override
    public void autonomousPeriodic() {
//        chassis.printAngle();
        chassis.reportEncoders();
        if (auto.run(motionProfileLoop())) setNewMotionProfile();

        DriverStation.reportWarning("" + auto.getCurrentPhase(), false);
        SubPhaser.run(auto.getCurrentPhase());
    }
    
    @Override public void teleopInit()
    {
        collector.init();
        chassis.init();
        setMotorsDriveMode();
    }

    @Override
    public void teleopPeriodic() {
        chassis.proRun();
        collector.run();
        climber.run();
        shooter.run();
//        if(joy1.getRawButton(8))
//        shooter.runDirect();
//        else
//            shooter.stop();
//        shooter.run();
//        shooter.printEnc();
//        DriverStation.reportError("Left Enc Position: " + leftLeader.getEncPosition()
//        + "\nRight Enc Position: " + rightLeader.getEncPosition(), false);

//        SmartDashboard.putNumber("Shooter Encoder", shooter.shooter.getEncPosition());
    }


    boolean gear = false;

    @Override
    public void testInit() {
        setMotorsDriveMode();
        chassis.init();
        collector.init();
    }

    @Override
    public void testPeriodic() {
        chassis.run();
        collector.run();
        climber.run();
        shooter.run();
        if(joy1.getRawButton(9))
        {
            collector.placeGearAuto();
            chassis.placeGear();
        }
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
        switch(controlStatus) {
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
                if (!e) {

                    double motorOutput = leftLeader.getOutputVoltage()
                            / leftLeader.getBusVoltage();
                    double altMotorOutput = rightLeader.getOutputVoltage()
                            / rightLeader.getBusVoltage();
                    System.out.println("motor output: " + motorOutput);
                    System.out.println("motor speed: " + leftLeader.getSpeed());
                    System.out.println("error in native units: "
                            + leftLeader.getClosedLoopError());
                }

                DriverStation.reportError("Active Point Valid:" + _status.activePointValid + "\n" +
                        "lastPoint: " + _status.activePoint.isLastPoint, false);
                if (_status.activePointValid && _status.activePoint.isLastPoint) {
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
    public void setEncoderInversions(boolean b) {
        rightLeader.reverseOutput(!b);
        leftLeader.reverseSensor(!b);
        rightLeader.reverseSensor(b);
        leftLeader.reverseOutput(b);
    }
    public void setEncoderInversions() {
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
        leftController.setFedProfile(HardcodedProfiles.rightBoiler.Points);
        rightController.setFedProfile(HardcodedProfiles.leftBoiler.Points);
    }
    public void setSideSpeedProfileRet() {
        leftController.setFedProfile(HardcodedProfiles.leftBoiler.Points);
        rightController.setFedProfile(HardcodedProfiles.rightBoiler.Points);

    }
    public void setCenterProfile() {
        leftController.setFedProfile(HardcodedProfiles.centerLift.Points);

        rightController.setFedProfile(HardcodedProfiles.centerLift.Points);
    }
    public void setSideBoilerShotProfile() {
        leftController.setFedProfile(HardcodedProfiles.leftSideBoilerShot.Points);
        rightController.setFedProfile(HardcodedProfiles.rightSideBoilerShot.Points);
    }
    public void setCenterBoilerShotProfile() {
        leftController.setFedProfile(HardcodedProfiles.leftSideBoilerShot.Points);
        rightController.setFedProfile(HardcodedProfiles.rightSideBoilerShot.Points);
    }
    public void setNZMoveProfile() {
        leftController.setFedProfile(HardcodedProfiles.leftNZ.Points);
        rightController.setFedProfile(HardcodedProfiles.rightNZ.Points);
    }
    public void setNZEndProfile() {
        leftController.setFedProfile(HardcodedProfiles.finishNZL.Points);

        rightController.setFedProfile(HardcodedProfiles.finishNZR.Points);
    }

    public void initMotionProfile() {
        setNewMotionProfile();
    }
    public void setNewMotionProfile() {
        say("prechecking for gear");
        if(auto.getCurrentPhase().isGearCheck()) {
            if(auto.isSkip()) {
                auto.insertPhase(new Phase(HardcodedProfiles.emptyProfile.Points, HardcodedProfiles.emptyProfile.Points, true, Phase.NONE), collector).setGearCheck(true);
                auto.insertPhase(new Phase(HardcodedProfiles.leftBack.Points, HardcodedProfiles.rightBack.Points, !RedSide, Phase.PLACEGEAR));
                auto.insertPhase(new Phase(retryGearLEFT.Points, retryGearRIGHT.Points, !RedSide, Phase.NONE));
            }
            say("checking for gear");
        }
        e = false;
        prepEncodersForProfile();
//        try {
        if(auto.PhaseCollection.size() > 0) {
            leftController.setFedProfile(auto.getCurrentProfileLeft());
            rightController.setFedProfile(auto.getCurrentProfileRight());
        }
    }
}

