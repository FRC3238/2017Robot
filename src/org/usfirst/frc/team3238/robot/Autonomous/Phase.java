package org.usfirst.frc.team3238.robot.Autonomous;



/**
 * Created by Team 3238 on 2/20/2017.
 */

public class Phase {
    public static final int NONE = 0, REVSHOOT = 1, SHOOT = 2, PLACEGEAR = 3, REVSHOOTGEAR = 4, QUICKSHOT = 5;
    double[][] leftProfile, rightProfile;
    boolean reverse, shoot = false, placeGear = false;
    double delay = 0.00;

    public int subsystemProperty = 0;
    public Phase(double[][] leftProfile, double[][] rightProfile, boolean reverse, int subsystemProperty) {
        this.subsystemProperty = subsystemProperty;
        this.leftProfile = reverse ? rightProfile : leftProfile;
        this.rightProfile = reverse ? leftProfile : rightProfile;
    }
    public Phase(double[][] leftProfile, double[][] rightProfile, boolean reverse, int subsystemProperty, double delay) {
        this.subsystemProperty = subsystemProperty;
        this.leftProfile = reverse ? rightProfile : leftProfile;
        this.rightProfile = reverse ? leftProfile : rightProfile;
        this.delay = delay;
    }
    public double getDelay() {
        return delay;
    }
    public double[][] getLeftProfile() {
        return leftProfile;
    }
    public double[][] getRightProfile() {
        return rightProfile;
    }
    public void setSubsystemProperty(int Property) {
        this.subsystemProperty = Property;
    }
    public void setDelay(double delay) {
        this.delay = delay;
    }
}
