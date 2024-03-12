package org.robotalons.crescendo.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitCommand extends Command{
    
    private double millis;
    private double startingTime;
    private Timer timer;

    public WaitCommand(double seconds) {

        this.millis = seconds * 1000;
    }

    @Override
    public void initialize() {
        startingTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() >= millis + startingTime;
    }
}
