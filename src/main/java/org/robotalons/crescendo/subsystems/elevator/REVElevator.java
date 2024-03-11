package org.robotalons.crescendo.subsystems.elevator;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.robotalons.lib.motion.elevator.ElevatorModule;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class REVElevator extends ElevatorModule{
    
    private final ElevatorModuleConstants CONSTANTS;
    private DoubleSupplier APPLIED_VOLTS;
    private final Queue<Double> POSITION_QUEUE;
    
    public REVElevator(ElevatorModuleConstants Constants) {
        super(Constants);
        CONSTANTS = Constants;
        APPLIED_VOLTS = () -> (0);
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
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
    // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
    public static final class ElevatorModuleConstants extends Constants {
        public ElevatorSim MOTOR;
        public Double LOOPPERIOD_SEC = (0.2);
        public ElevatorFeedforward ELEVATOR_FEEDFORWARD;
        public PIDController PID_CONTROLLER;
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
