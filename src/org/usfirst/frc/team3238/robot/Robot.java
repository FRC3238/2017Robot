package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot
{
    private Chassis chassis;
    private Collector collector;
    private Climber climber;
    
    @Override public void robotInit()
    {
        CANTalon driveLeftTalonA = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
        CANTalon driveLeftTalonB = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
        CANTalon driveRightTalonA = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
        CANTalon driveRightTalonB = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
        CANTalon leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
        CANTalon rightCollect = new CANTalon(Constants.Collector.RIGHT_TALON_ID);
        CANTalon liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
        CANTalon climbTalonOne = new CANTalon(Constants.Climber.CLIMB_TALON_ONE_ID);
        CANTalon climbTalonTwo = new CANTalon(Constants.Climber.CLIMB_TALON_TWO_ID);
        Joystick joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);
        
        climber = new Climber(climbTalonOne, climbTalonTwo, joystick);
        chassis = new Chassis(driveLeftTalonA, driveLeftTalonB,
                driveRightTalonA, driveRightTalonB, joystick);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                joystick);
    }
    
    @Override public void disabledPeriodic()
    {
        climber.disabled();
    }
    
    @Override public void autonomousInit()
    {
        
    }
    
    @Override public void autonomousPeriodic()
    {
        
    }
    
    @Override public void teleopInit()
    {
        collector.init();
    }
    
    @Override public void teleopPeriodic()
    {
        chassis.run();
        collector.run();
        climber.run();
    }

    
    @Override public void testInit()
    {
        
    }
    
    @Override public void testPeriodic()
    {
     chassis.proRun();
     collector.run();
     climber.run();
    }
}

