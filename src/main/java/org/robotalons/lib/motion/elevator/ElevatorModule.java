package org.robotalons.lib.motion.elevator;

import java.io.Closeable;
import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class ElevatorModule implements Closeable{
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    protected final Constants CONSTANTS;  
    protected final ElevatorModuleStatusContainer STATUS = new ElevatorModuleStatusContainer();  

    protected ElevatorModule(final Constants Constants) {
        CONSTANTS = Objects.requireNonNull(Constants);
    }

    public static class Constants{
        public static final Integer CURRENT_LIMIT = (60);
        public static final Double GEAR_RATIO = (0.0);
        public static final Double BASE_HEIGHT = (0.0);
        public static final Double MIN_HEIGHT = (0.0);
        public static final Double MAX_HEIGHT = (0.0);
        public static final Double MASS_KG = (0.0);
        public static final Double RADIUS = (0.0);
    }

    public abstract void close();
    public abstract void cease();
    public abstract void update();
    public abstract void periodic();

    protected abstract void setVoltage(int volts);
    protected abstract void pidSet(double demand);
}
