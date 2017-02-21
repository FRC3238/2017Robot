package org.usfirst.frc.team3238.robot.Autonomous;

import java.util.ArrayList;

/**
 * Created by Team 3238 on 2/20/2017.
 */
public class Phaser {
    public ArrayList<Phase> PhaseCollection;
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
        if(finishedProfile) {
            if(PhaseCollection.size() > 1) {
                PhaseCollection.remove(0);
                return true;
            }
        }
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
