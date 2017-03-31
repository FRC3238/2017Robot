package org.usfirst.frc.team3238.robot.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

/**
 * Created by Team 3238 on 2/20/2017.
 */
public class Phaser {
    public ArrayList<Phase> PhaseCollection;
    Timer delayTimer = new Timer();
    public Phaser() {
        PhaseCollection = new ArrayList<Phase>();
    }
    public void addPhase(Phase phase) {
        PhaseCollection.add(phase);
    }
    public double[][] initLeft() {
        return PhaseCollection.get(0).getLeftProfile();
    }
    public double[][] initRight() {
        return PhaseCollection.get(0).getRightProfile();
    }
    public boolean run(boolean finishedProfile) {
        if (finishedProfile) {
            delayTimer.reset();
            delayTimer.start();
            if (PhaseCollection.size() > 1) {
                while(delayTimer.get() < PhaseCollection.get(0).getDelay());
                if (delayTimer.get() >= PhaseCollection.get(0).getDelay()) {
                    DriverStation.reportError("Delay Achieved", false);

                    delayTimer.stop();
                    PhaseCollection.remove(0);
                    return true;
                } else {
                    return false;
//                return true;
                }
            }
        }
            delayTimer.reset();
            return false;

    }
    public Phase getCurrentPhase() {
        if(PhaseCollection.size() > 0)
            return PhaseCollection.get(0);
        else
            return new Phase(new double[][] {}, new double[][] {}, false, Phase.NONE);
    }
    public double[][] getCurrentProfileLeft() {
        try {
        if(PhaseCollection.size() > 0)

            return PhaseCollection.get(0).getLeftProfile();
        }  catch(Exception e) {
            e.printStackTrace();

        }
        return null;
    }
    public double[][] getCurrentProfileRight() {
        try {
            if (PhaseCollection.size() > 0)


                return PhaseCollection.get(0).getRightProfile();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

}
