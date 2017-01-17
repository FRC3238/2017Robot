package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot
{
    Chassis chassis;
    Collector collector;
    DigitalInput collectLimit;
    Joystick joystick;
    CANTalon driveLeftTalonA, driveLeftTalonB, driveRightTalonA, driveRightTalonB;
    CANTalon leftCollect, rightCollect, liftCollect;
    
    @Override public void robotInit()
    {
        driveLeftTalonA = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
        driveLeftTalonB = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
        driveRightTalonA = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
        driveRightTalonB = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
        leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
        rightCollect = new CANTalon(Constants.Collector.RIGHT_TALON_ID);
        liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
        collectLimit = new DigitalInput(Constants.Collector.LIMIT_CHANNEL);
        joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);
        chassis = new Chassis(driveLeftTalonA, driveLeftTalonB,
                driveRightTalonA, driveRightTalonB, joystick);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                collectLimit, joystick);
    }
    
    @Override public void disabledPeriodic()
    {
        
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
    }
    
    @Override public void testInit()
    {
        
    }
    
    @Override public void testPeriodic()
    {
        
    }
}

