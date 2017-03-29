package org.usfirst.frc.team3238.robot.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team3238.robot.Robot;

import java.util.ArrayList;

/**
 * Created by Team 3238 on 2/20/2017.
 */
public class Phaser {
    public ArrayList<Phase> PhaseCollection;
    public ArrayList<SkipCondition> skipCollection;
    Timer delayTimer = new Timer();
    Timer skipTimer = new Timer();
    public Phaser() {
        PhaseCollection = new ArrayList<Phase>();
        skipCollection = new ArrayList<SkipCondition>();
    }
    public Phase addPhase(Phase phase, SkipCondition condition) {
        PhaseCollection.add(phase);
        skipCollection.add(condition);
        return phase;
    }
    public void addPhase(Phase phase)
    {
        addPhase(phase, run);
    }
    public double[][] initLeft() {
        return PhaseCollection.get(0).getLeftProfile();
    }
    public double[][] initRight() {
        return PhaseCollection.get(0).getRightProfile();
    }

    public interface SkipCondition {
        public boolean skip();
        public void reset();
    }

    public SkipCondition run = new SkipCondition() {
        @Override
        public boolean skip() {
            Robot.say("Default skip condition is being called");
            return false;
        }

        @Override
        public void reset() {

        }
    };

    public boolean run(boolean finishedProfile) {
        if (finishedProfile) {
            delayTimer.start();
            if (PhaseCollection.size() > 1) {
                if (delayTimer.get() >= PhaseCollection.get(0).getDelay()) {
                    DriverStation.reportError("Delay Achieved", false);

                    if (PhaseCollection.size() > 1) {
                        PhaseCollection.remove(0);
                        skipCollection.remove(0);
                    }

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

    public boolean isSkip() {
        boolean skip = false;
        if (PhaseCollection.size() >= 1) {
//            boolean skip = false;
            skipTimer.stop();
            skipTimer.reset();
            skipTimer.start();
            while(skipTimer.get() < 0.5)
            {
                Robot.say("" + (skip = skipCollection.get(0).skip()));
                Robot.say("isSkip is called with t=" + skipTimer.get());
            }
            skipCollection.get(0).reset();
            skipTimer.stop();
            skipTimer.reset();
            if (skip) {
                Robot.say("Gear is still onboard!");
//                PhaseCollection.remove(0);
//                skipCollection.remove(0);

            }
        }
        return skip;
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
