package org.usfirst.frc.team3238.robot.Autonomous;

import org.usfirst.frc.team3238.robot.Constants;
import org.usfirst.frc.team3238.robot.Robot;
import org.usfirst.frc.team3238.robot.Shooter;
import org.usfirst.frc.team3238.robot.Collector;

/**
 * Created by Team 3238 on 2/20/2017.
 */
public class SubsystemPhaser {
    Collector collector;
    Shooter shooter;
    public boolean calledCollect = false;
    public SubsystemPhaser(Collector collector, Shooter shooter) {
        this.collector = collector;
        this.shooter = shooter;
    }
    public void run(Phase current) {
        switch(current.subsystemProperty) {
            case Phase.NONE:
                break;
            case Phase.REVSHOOT:
                shooter.autoPrep(Constants.Shooter.SHOOT_RPM);
                break;
            case Phase.SHOOT:
                shooter.autoShoot();
                break;
            case Phase.PLACEGEAR:
                placeGear();
                break;
            case Phase.REVSHOOTGEAR:
                shooter.autoPrep(-1);
                placeGear();
                break;
            case Phase.QUICKSHOT:
                shooter.quickShoot();
                break;
        }
    }
    public void placeGear() {
        if(!calledCollect) {
            collector.placeGear();
            calledCollect = true;
        }
        collector.run();
    }
}
