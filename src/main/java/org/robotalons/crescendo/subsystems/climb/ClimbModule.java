// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.climb;

import java.io.IOException;

import org.robotalons.crescendo.subsystems.climb.Constants.Measurements;
import org.robotalons.lib.climb.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public final class ClimbModule extends Climb {

    private static CANSparkMax MOTOR;
    private static Encoder ENCODER;
    private static SparkMaxPIDController MOTOR_PID;
    private static Double K_GEARRATIO;
    private static Double K_TICKS_2_RAD;
    private static ArmFeedforward MOTOR_FF;
    

    public ClimbModule(final Integer MOTOR_ID, final Integer A_CHANNEL, final Integer B_CHANNEL){
        
        MOTOR = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        ENCODER = new Encoder(A_CHANNEL, B_CHANNEL);
        MOTOR_PID = MOTOR.getPIDController();

        MOTOR_FF = new ArmFeedforward(
            Constants.Measurements.LEFT_ARM_KS,
            Constants.Measurements.LEFT_ARM_KG,
            Constants.Measurements.LEFT_ARM_KV,
            Constants.Measurements.LEFT_ARM_KA
            );

        configPID();
        config();  

        K_GEARRATIO = Measurements.K_GEARRATIO;
        K_TICKS_2_RAD = Measurements.K_TICKS_2_RAD;
    }

    @Override
    public void close() throws IOException {
        MOTOR.close();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    @Override
    public void pidSet(Double SETPOINT) {
        MOTOR_PID.setReference(
        SETPOINT, 
        ControlType.kPosition, 
        (0), 
        MOTOR_FF.calculate(getPosistion(), getVelocity()), 
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void set(Double DEMAND) {
        MOTOR.set(DEMAND);
    }

    @Override
    public synchronized Double getTemperature() {
        return MOTOR.getMotorTemperature();
    }

    @Override
    public synchronized Double getVelocity() {
        return ENCODER.getDistancePerPulse();
    }

    @Override
    public synchronized Double getPosistion() {
        return ENCODER.getDistance() * K_TICKS_2_RAD / K_GEARRATIO;
    }

    private synchronized static void configPID() {
        MOTOR_PID.setFeedbackDevice((MotorFeedbackSensor) ENCODER);
        MOTOR_PID.setP(Measurements.LEFT_ARM_P);
        MOTOR_PID.setI(Measurements.LEFT_ARM_I);
        MOTOR_PID.setD(Measurements.LEFT_ARM_D);
    }

    private  synchronized static void config(){
        MOTOR.setIdleMode(IdleMode.kBrake);
        MOTOR.restoreFactoryDefaults();
        MOTOR.clearFaults();
        
        MOTOR.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);

        MOTOR.setSoftLimit(
        SoftLimitDirection.kForward, 
        Measurements.K_FORWARD_ARM_LIMIT);
        
        MOTOR.setSoftLimit(
        SoftLimitDirection.kReverse, 
        Measurements.K_REVERSE_ARM_LIMIT);
    }

}
