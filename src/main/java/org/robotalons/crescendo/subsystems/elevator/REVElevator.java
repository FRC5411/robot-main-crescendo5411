package org.robotalons.crescendo.subsystems.elevator;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.robotalons.lib.motion.elevator.ElevatorModule;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Encoder;

public class REVElevator extends ElevatorModule{
    
    private final ElevatorModuleConstants CONSTANTS;
    private DoubleSupplier APPLIED_VOLTS;
    private final Queue<Double> QUEUE;
    
    public REVElevator(ElevatorModuleConstants Constants) {
        super(Constants);
        CONSTANTS = Constants;
        APPLIED_VOLTS = () -> (0);
        QUEUE = org.robotalons.crescendo.Constants.Odometry.REV_ODOMETRY_THREAD.register(() -> (Status.));
    }

    @Override
    public void close() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }

    @Override
    public void cease() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'cease'");
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
    // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
    public static final class ElevatorModuleConstants extends Constants {
        public CANSparkMax MOTOR;
        public Encoder ENCODER;
        public ElevatorFeedforward ELEVATOR_FEEDFORWARD;
        public SparkMaxPIDController PID_CONTROLLER;
    }

    // --------------------------------------------------------------[Methods]---------------------------------------------------------------//  
    @Override
    protected void setVoltage(int volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    protected void pidSet(double demand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'pidSet'");
    }
    
}
