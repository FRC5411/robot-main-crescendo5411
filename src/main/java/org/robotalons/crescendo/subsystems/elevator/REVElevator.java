package org.robotalons.crescendo.subsystems.elevator;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.robotalons.crescendo.subsystems.drivebase.Constants.Simulation;
import org.robotalons.lib.motion.elevator.ElevatorModule;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import talon.motion.profile.ProfilePIDController;

public class REVElevator extends ElevatorModule{
    
    private final ElevatorModuleConstants CONSTANTS;
    private DoubleSupplier APPLIED_VOLTS;
    private final Queue<Double> POSITION_QUEUE;
    private final Lock ODOMETRY_LOCK;
    private Rotation2d SETPOINT;
   
    private double RotationalRelativePosition = 0;
    private double RotationalAbsolutePosition = 0;

    public REVElevator(ElevatorModuleConstants Constants) {
        super(Constants);
        CONSTANTS = Constants;
        APPLIED_VOLTS = () -> (0);
        SETPOINT = null;
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
        // code block modified from mechanical advantage
        double angleDiffRad = CONSTANTS.ROTATIONAL_CONTROLLER.getAngularVelocityRadPerSec() * Simulation.SIMULATION_LOOPPERIOD_SEC;
        RotationalRelativePosition += angleDiffRad;
        RotationalAbsolutePosition += angleDiffRad;
        // reverses negative rotation to positive
        // ie -3.14 -> 3.14 (same position)
        while (RotationalAbsolutePosition < 0) {
            RotationalAbsolutePosition += 2.0 * Math.PI;
        }
        // lowers overshoot
        // ie 7.85 - 3.14 = 3.92
        while (RotationalAbsolutePosition > 2.0 * Math.PI) {
            RotationalAbsolutePosition -=  Math.PI * 2;
        }
        STATUS.Position_RD = CONSTANTS.MOTOR.getVelocityMetersPerSecond() * CONSTANTS.LOOPPERIOD_SEC;
        STATUS.Velocity_MpSec = CONSTANTS.MOTOR.getVelocityMetersPerSecond();
        STATUS.Volts = APPLIED_VOLTS.getAsDouble();
    }

    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        double pidOutput = CONSTANTS.PID_CONTROLLER.calculate(
            Math.toDegrees(STATUS.Position_RD), SETPOINT.getDegrees()
        );
        double feedForwardOutput = CONSTANTS.ELEVATOR_FEEDFORWARD.calculate(
            Math.toRadians(CONSTANTS.PID_CONTROLLER.getSetpoint().position),
            CONSTANTS.PID_CONTROLLER.getSetpoint().velocity
        );
        setVoltage(pidOutput + feedForwardOutput);
    }
    // --------------------------------------------------------------[Internal]---------------------------------------------------------------//  
    public static final class ElevatorModuleConstants extends Constants {
        public ElevatorSim MOTOR;
        public Double LOOPPERIOD_SEC = (0.2);
        public ElevatorFeedforward ELEVATOR_FEEDFORWARD;
        public ProfiledPIDController PID_CONTROLLER;
    }

    
    // @Override
    // public Rotation2d getRelativeRotation() {
    //     return (Status.RotationalRelativePosition.plus(Azimuth_Offset));
    // }
    // --------------------------------------------------------------[Methods]---------------------------------------------------------------//  
    @Override
    protected void setVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    protected void pidSet(double demand) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'pidSet'");
    }
    
}
