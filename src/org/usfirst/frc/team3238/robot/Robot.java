package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Main controlling class for all robot actions
 *
 * @author aaron, jefferson
 */
public class Robot extends IterativeRobot
{
    private Chassis chassis;
    private Collector collector;
    private Climber climber;
    
    /**
     * Called once when robot starts
     */
    @Override public void robotInit()
    {
        // Chassis
        CANTalon driveLeftTalonA = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
        CANTalon driveLeftTalonB = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
        CANTalon driveRightTalonA = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
        CANTalon driveRightTalonB = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
        
        // Collector
        CANTalon leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
        CANTalon rightCollect = new CANTalon(
                Constants.Collector.RIGHT_TALON_ID);
        CANTalon liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
        
        // Climber
        CANTalon climbTalonOne = new CANTalon(
                Constants.Climber.CLIMB_TALON_ONE_ID);
        CANTalon climbTalonTwo = new CANTalon(
                Constants.Climber.CLIMB_TALON_TWO_ID);
        
        // Joystick
        Joystick joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);
        
        climber = new Climber(climbTalonOne, climbTalonTwo, joystick);
        chassis = new Chassis(driveLeftTalonA, driveLeftTalonB,
                driveRightTalonA, driveRightTalonB, joystick);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                joystick);
    }
    
    /**
     * Called periodically when robot is disabled
     */
    @Override public void disabledPeriodic()
    {
        
    }
    
    /**
     * Called once when auto period begins
     */
    @Override public void autonomousInit()
    {
        
    }
    
    /**
     * Called periodically when auto mode is running
     */
    @Override public void autonomousPeriodic()
    {
        
    }
    
    /**
     * Called once when teleop period begins
     */
    @Override public void teleopInit()
    {
        collector.init();
    }
    
    /**
     * Called periodically when teleop mode is running
     */
    @Override public void teleopPeriodic()
    {
        chassis.run();
        collector.run();
        climber.run();
    }
    
    /**
     * Called once when test mode begins
     */
    @Override public void testInit()
    {
        collector.init();
    }
    
    /**
     * Called periodically when test mode is running
     */
    @Override public void testPeriodic()
    {
        chassis.run();
        collector.run();
        climber.run();
    }
}

