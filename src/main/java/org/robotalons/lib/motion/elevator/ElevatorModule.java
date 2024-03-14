package org.robotalons.lib.motion.elevator;

import java.io.Closeable;
import java.util.Objects;


public abstract class ElevatorModule implements Closeable{
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    protected final Constants CONSTANTS;  
    protected final ElevatorModuleStatusContainer STATUS = new ElevatorModuleStatusContainer();  

    protected ElevatorModule(final Constants Constants) {
        CONSTANTS = Objects.requireNonNull(Constants);
    }

    public static class Constants{
        public  final Integer CURRENT_LIMIT = (60);
        public  final Double GEAR_RATIO = (0.0);
        public  final Double BASE_HEIGHT = (0.0);
        public  final Double MIN_HEIGHT = (0.0);
        public  final Double MAX_HEIGHT = (0.0);
        public  final Double MASS_KG = (0.0);
        public  final Double RADIUS = (0.0);
    }
    public static enum ElevatorStates{
        DISABLED,
        CLOSED,
        RUNNING
    }

    public abstract void close();
    public abstract void cease();
    public abstract void update();
    public abstract void periodic();

    public abstract void setVoltage(double volts);
    public abstract void setRollerVoltage(double volts);
    // protected abstract void pidSet(double demand);
    public abstract void setState(ElevatorStates state);
}
