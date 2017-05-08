/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 *
 * The only routines we call on Talon are....
 *
 * changeMotionControlFramePeriod
 *
 * getMotionProfileStatus       
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 *
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 *
 * getControlMode, to check if we are in Motion Profile Control mode.
 *
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
package org.usfirst.frc.team3238.robot.Autonomous;


import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team3238.robot.Autonomous.instrumentation;

public class MotionProfileExample {

    /**
     * The status of the motion profile executer and buffer inside the Talon.
     * Instead of creating a new one every time we call getMotionProfileStatus,
     * keep one copy.
     */
    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();

    /**
     * reference to the talon we plan on manipulating. We will not changeMode()
     * or call set(), just get motion profile status and make decisions based on
     * motion profile.
     */
    private CANTalon _talon;
    /**
     * State machine to make sure we let enough of the motion profile stream to
     * talon before we fire it.
     */
    private int _state = 0;
    /**
     * Any time you have a state machine that waits for external events, its a
     * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
     * down to '0' which will print an error message. Counting loops is not a
     * very accurate method of tracking timeout, but this is just conservative
     * timeout. Getting time-stamps would certainly work too, this is just
     * simple (no need to worry about timer overflows).
     */
    private int _loopTimeout = -1;
    /**
     * If start() gets called, this flag is set and in the control() we will
     * service it.
     */
    private boolean _bStart = false;

    /**
     * Since the CANTalon.set() routine is mode specific, deduce what we want
     * the set value to be and let the calling module apply it whenever we
     * decide to switch to MP mode.
     */
    private CANTalon.SetValueMotionProfile _setValue = CANTalon.SetValueMotionProfile.Disable;
    /**
     * How many trajectory points do we wait for before firing the motion
     * profile.
     */
    private static final int kMinPointsInTalon = 5;
    /**
     * Just a state timeout to make sure we don't get stuck anywhere. Each loop
     * is about 20ms.
     */
    private static final int kNumLoopsTimeout = 10;

    /**
     * Lets create a periodic task to funnel our trajectory points into our talon.
     * It doesn't need to be very accurate, just needs to keep pace with the motion
     * profiler executer.  Now if you're trajectory points are slow, there is no need
     * to do this, just call _talon.processMotionProfileBuffer() in your teleop loop.
     * Generally speaking you want to call it at least twice as fast as the duration
     * of your trajectory points.  So if they are firing every 20ms, you should call 
     * every 10ms.
     */
    double[][] fedProfile;
    int kNumPoints;
    class PeriodicRunnable implements java.lang.Runnable {
        public void run() {  _talon.processMotionProfileBuffer();    }
    }
    Notifier _notifer = new Notifier(new PeriodicRunnable());
    public void setFedProfile(double[][] profile) {
        fedProfile = profile;
        kNumPoints = profile.length;
    }

    /**
     * C'tor
     *
     * @param talon
     *            reference to Talon object to fetch motion profile status from.
     */
    public MotionProfileExample(CANTalon talon) {
        _talon = talon;
        /*
         * since our MP is 10ms per point, set the control frame rate and the
         * notifer to half that
         */
        _talon.changeMotionControlFramePeriod(5);
        //_notifer.startPeriodic(0.005);
    }

    /**
     * Called to clear Motion profile buffer and reset state info during
     * disabled and when Talon is not in MP control mode.
     */
    public void reset() {
        /*
         * Let's clear the buffer just in case user decided to disable in the
         * middle of an MP, and now we have the second half of a profile just
         * sitting in memory.
         */
        _talon.clearMotionProfileTrajectories();
        /* When we do re-enter motionProfile control mode, stay disabled. */
        _setValue = CANTalon.SetValueMotionProfile.Disable;
        /* When we do start running our state machine start at the beginning. */
        _state = 0;
        _loopTimeout = -1;
        /*
         * If application wanted to start an MP before, ignore and wait for next
         * button press
         */
        _bStart = false;
    }

    /**
     * Called every loop.
     */


    /** Start filling the MPs to all of the involved Talons. */
    public void fillFed() {
        startFilling(fedProfile, kNumPoints);
    }

    private void startFilling(double[][] profile, int totalCnt) {

        /* create an empty point */
        CANTalon.TrajectoryPoint point = new CANTalon.TrajectoryPoint();

        /* did we get an underrun condition since last time we checked ? */
        if (_status.hasUnderrun) {
            /* better log it so we know about it */
            DriverStation.reportWarning("filling: underrun", false);
            instrumentation.OnUnderrun();
            /*
             * clear the error. This flag does not auto clear, this way 
             * we never miss logging it.
             */
            _talon.clearMotionProfileHasUnderrun();
        }
        /*
         * just in case we are interrupting another MP and there is still buffer
         * points in memory, clear it.
         */
        _talon.clearMotionProfileTrajectories();

        /* This is fast since it's just into our TOP buffer */
        for (int i = 0; i < totalCnt; ++i) {
            /* for each point, fill our structure and pass it to API */
            point.position = profile[i][0];
            point.velocity = profile[i][1];
            point.timeDurMs = (int) profile[i][2];
            point.profileSlotSelect = 0; /* which set of gains would you like to use? */
            point.velocityOnly = false; /* set true to not do any position
                                         * servo, just velocity feedforward
                                         */
            point.zeroPos = false;
            if (i == 0)
                point.zeroPos = true; /* set this to true on the first point */

            point.isLastPoint = false;
            if ((i + 1) == totalCnt)
                point.isLastPoint = true; /* set this to true on the last point  */

            _talon.pushMotionProfileTrajectory(point);
        }
    }

    /**
     * Called by application to signal Talon to start the buffered MP (when it's
     * able to).
     */
    void startMotionProfile() {
        _bStart = true;
    }

    /**
     *
     * @return the output value to pass to Talon's set() routine. 0 for disable
     *         motion-profile output, 1 for enable motion-profile, 2 for hold
     *         current motion profile trajectory point.
     */
    CANTalon.SetValueMotionProfile getSetValue() {
        return _setValue;
    }
}