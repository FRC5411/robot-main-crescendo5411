package org.robotalons.crescendo.subsystems.elevator;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.robotalons.lib.motion.elevator.ElevatorModule;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class REVElevatorSim extends ElevatorModule{
    
    private final REVElevatorSimConstants CONSTANTS;
    private DoubleSupplier APPLIED_VOLTS;
    private final Queue<Double> POSITION_QUEUE;
    private final Lock ODOMETRY_LOCK;
    private Rotation2d SETPOINT;
    private ElevatorStates STATE;


    public REVElevatorSim(REVElevatorSimConstants Constants) {
        super(Constants);
        CONSTANTS = Constants;
        APPLIED_VOLTS = () -> (0);
        SETPOINT = null;
        STATE = ElevatorStates.RUNNING;
        ODOMETRY_LOCK = new ReentrantLock();
        POSITION_QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (STATUS.Position_RD));
        
    }

    @Override
    public void close() {
        POSITION_QUEUE.clear();
    }

    @Override
    public void cease() {
        CONSTANTS.MOTOR.setState((0d),(0d));
    }

    @Override
    public void update() {
        CONSTANTS.MOTOR.update(CONSTANTS.LOOPPERIOD_SEC);

        STATUS.Position_RD = CONSTANTS.MOTOR.getVelocityMetersPerSecond() * CONSTANTS.LOOPPERIOD_SEC;
        STATUS.Velocity_MpSec = CONSTANTS.MOTOR.getVelocityMetersPerSecond();
        STATUS.Volts = APPLIED_VOLTS.getAsDouble();
        STATUS.OdometryPosition_RD = 
            POSITION_QUEUE.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / CONSTANTS.GEAR_RATIO)
                .toArray();
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        switch (STATE){
            case RUNNING:
                double pidOutput = CONSTANTS.PID_CONTROLLER.calculate(
                    Math.toDegrees(STATUS.Position_RD), SETPOINT.getDegrees()
                );
                double feedForwardOutput = CONSTANTS.ELEVATOR_FEEDFORWARD.calculate(
                    Math.toRadians(CONSTANTS.PID_CONTROLLER.getSetpoint().position),
                    CONSTANTS.PID_CONTROLLER.getSetpoint().velocity
                );
                setVoltage(pidOutput + feedForwardOutput);
                break;
            case DISABLED:
                cease();
            case CLOSED:
                close();
                break;
        }
        ODOMETRY_LOCK.unlock();      
    }
    // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
    public static final class REVElevatorSimConstants extends Constants {
        public ElevatorSim MOTOR;
        public Double LOOPPERIOD_SEC = (0.2);
        public ElevatorFeedforward ELEVATOR_FEEDFORWARD;
        public ProfiledPIDController PID_CONTROLLER;
    }

    
    // --------------------------------------------------------------[Methods]---------------------------------------------------------------//  
    @Override
    protected void setVoltage(double volts) {
        APPLIED_VOLTS = () -> volts;
        CONSTANTS.MOTOR.setInput(volts);
    }


    @Override
    protected void setState(ElevatorStates state) {
        STATE = state;
    }
    
}
