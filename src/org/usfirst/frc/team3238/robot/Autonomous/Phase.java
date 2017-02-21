package org.usfirst.frc.team3238.robot.Autonomous;



/**
 * Created by Team 3238 on 2/20/2017.
 */

public class Phase {
    public static final int NONE = 0, REVSHOOT = 1, SHOOT = 2, PLACEGEAR = 3, REVSHOOTGEAR = 4;
    double[][] leftProfile, rightProfile;
    boolean reverse, shoot = false, placeGear = false;

    public int subsystemProperty = 0;
    public Phase(double[][] leftProfile, double[][] rightProfile, boolean reverse, int subsystemProperty) {
        this.subsystemProperty = subsystemProperty;
        this.leftProfile = reverse ? rightProfile : leftProfile;
        this.rightProfile = reverse ? leftProfile : rightProfile;
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
}
