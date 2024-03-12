// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.drive;

import java.util.stream.IntStream;

import org.robotalons.crescendo.subsystems.drive.Constants.Measurements;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase{
    private final CANSparkMax DRIVE_MOTOR;
    private final CANSparkMax ASIMUTH_MOTOR;

    private final boolean DRIVE_FLIPPED;
    private final boolean ASIMUTH_FLIPPED;

    private final int NUMBER;

    private final CANcoder ABSOULTE_ENCODER;
    
    private final double ABSOLUTE_ENCODER_OFFSET;
    private final boolean ABSOLUTE_ENCODER_REVERSED;

    private final RelativeEncoder DRIVE_ENCODER;
    private final RelativeEncoder ASIMUTH_ENCODER;

    private final PIDController DRIVE_PID;
    private final PIDController ASIMUTH_PID;
    private final SimpleMotorFeedforward DRIVE_FF;

    public SwerveModule(int Drive_ID, int Asimuth_ID, boolean Drive_Flipped, boolean Asimuth_Flipped, int NUM, double OFFSET, int Encoder_ID, boolean Encoder_Reversed){
        DRIVE_FLIPPED = Drive_Flipped;
        ASIMUTH_FLIPPED = Asimuth_Flipped;
        
        DRIVE_MOTOR = new CANSparkMax(Drive_ID, MotorType.kBrushless);
        ASIMUTH_MOTOR = new CANSparkMax(Asimuth_ID, MotorType.kBrushless);

        NUMBER = NUM;

        ABSOLUTE_ENCODER_OFFSET = OFFSET;
        ABSOLUTE_ENCODER_REVERSED = Encoder_Reversed;

        ABSOULTE_ENCODER = new CANcoder(Encoder_ID);

        DRIVE_ENCODER = DRIVE_MOTOR.getEncoder();
        ASIMUTH_ENCODER = ASIMUTH_MOTOR.getEncoder();

        config();

        if(NUMBER == 0){
            DRIVE_PID = new PIDController(
                Constants.Measurements.Modules.FL.DRIVE_P_GAIN,
                Constants.Measurements.Modules.FL.DRIVE_I_GAIN,
                Constants.Measurements.Modules.FL.DRIVE_D_GAIN
                );
            ASIMUTH_PID = new PIDController(
                Constants.Measurements.Modules.FL.ASIMUTH_P_GAIN,
                Constants.Measurements.Modules.FL.ASIMUTH_I_GAIN,
                Constants.Measurements.Modules.FL.ASIMUTH_D_GAIN
            );

            DRIVE_FF = new SimpleMotorFeedforward(
                Constants.Measurements.Modules.FL.DRIVE_KS_GAIN,
                Constants.Measurements.Modules.FL.DRIVE_KV_GAIN,
                Constants.Measurements.Modules.FL.DRIVE_KA_GAIN
            );
        }

        else if(NUMBER == 1){
            DRIVE_PID = new PIDController(
                Constants.Measurements.Modules.FR.DRIVE_P_GAIN,
                Constants.Measurements.Modules.FR.DRIVE_I_GAIN,
                Constants.Measurements.Modules.FR.DRIVE_D_GAIN
                );
            ASIMUTH_PID = new PIDController(
                Constants.Measurements.Modules.FR.ASIMUTH_P_GAIN,
                Constants.Measurements.Modules.FR.ASIMUTH_I_GAIN,
                Constants.Measurements.Modules.FR.ASIMUTH_D_GAIN
            );

            DRIVE_FF = new SimpleMotorFeedforward(
                Constants.Measurements.Modules.FR.DRIVE_KS_GAIN,
                Constants.Measurements.Modules.FR.DRIVE_KV_GAIN,
                Constants.Measurements.Modules.FR.DRIVE_KA_GAIN
            );
        }

        else if(NUMBER == 2){
            DRIVE_PID = new PIDController(
                Constants.Measurements.Modules.RL.DRIVE_P_GAIN,
                Constants.Measurements.Modules.RL.DRIVE_I_GAIN,
                Constants.Measurements.Modules.RL.DRIVE_D_GAIN
                );
            ASIMUTH_PID = new PIDController(
                Constants.Measurements.Modules.RL.ASIMUTH_P_GAIN,
                Constants.Measurements.Modules.RL.ASIMUTH_I_GAIN,
                Constants.Measurements.Modules.RL.ASIMUTH_D_GAIN
            );

            DRIVE_FF = new SimpleMotorFeedforward(
                Constants.Measurements.Modules.RL.DRIVE_KS_GAIN,
                Constants.Measurements.Modules.RL.DRIVE_KV_GAIN,
                Constants.Measurements.Modules.RL.DRIVE_KA_GAIN
            );
        }

        else if(NUMBER == 3){
            DRIVE_PID = new PIDController(
                Constants.Measurements.Modules.RR.DRIVE_P_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_I_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_D_GAIN
                );
            ASIMUTH_PID = new PIDController(
                Constants.Measurements.Modules.RR.ASIMUTH_P_GAIN,
                Constants.Measurements.Modules.RR.ASIMUTH_I_GAIN,
                Constants.Measurements.Modules.RR.ASIMUTH_D_GAIN
            );

            DRIVE_FF = new SimpleMotorFeedforward(
                Constants.Measurements.Modules.RR.DRIVE_KS_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_KV_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_KA_GAIN
            );
        }

        else{
             DRIVE_PID = new PIDController(
                Constants.Measurements.Modules.RR.DRIVE_P_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_I_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_D_GAIN
                );
            ASIMUTH_PID = new PIDController(
                Constants.Measurements.Modules.RR.ASIMUTH_P_GAIN,
                Constants.Measurements.Modules.RR.ASIMUTH_I_GAIN,
                Constants.Measurements.Modules.RR.ASIMUTH_D_GAIN
            );

            DRIVE_FF = new SimpleMotorFeedforward(
                Constants.Measurements.Modules.RR.DRIVE_KS_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_KV_GAIN,
                Constants.Measurements.Modules.RR.DRIVE_KA_GAIN
            );           
        }

    }

    public void cease(){
        DRIVE_MOTOR.stopMotor();
        ASIMUTH_MOTOR.stopMotor();
    }

    private synchronized void config() {
        cease();

        DRIVE_MOTOR.restoreFactoryDefaults();
        ASIMUTH_MOTOR.restoreFactoryDefaults();

        DRIVE_MOTOR.setCANTimeout((250));
        ASIMUTH_MOTOR.setCANTimeout((250));

        DRIVE_MOTOR.setInverted(DRIVE_FLIPPED);
        ASIMUTH_MOTOR.setInverted(ASIMUTH_FLIPPED);

        DRIVE_MOTOR.setIdleMode(IdleMode.kBrake);
        ASIMUTH_MOTOR.setIdleMode(IdleMode.kCoast);

        ASIMUTH_MOTOR.setSmartCurrentLimit((30));
        DRIVE_MOTOR.setSmartCurrentLimit((40));
        
        DRIVE_MOTOR.enableVoltageCompensation((12d));
        ASIMUTH_MOTOR.enableVoltageCompensation((12d));

        ASIMUTH_ENCODER.setPosition((0d));
        ASIMUTH_ENCODER.setMeasurementPeriod((10));
        ASIMUTH_ENCODER.setAverageDepth((2));
        
        ASIMUTH_ENCODER.setPosition(
        (RobotBase.isReal())?
            (-ABSOLUTE_ENCODER_OFFSET + (Rotation2d.fromRotations(ABSOULTE_ENCODER.getAbsolutePosition().getValue())).getRotations()):
            (0d)
        );

        ASIMUTH_ENCODER.setAverageDepth((2));
        ASIMUTH_ENCODER.setMeasurementPeriod((10));

        ASIMUTH_PID.enableContinuousInput(-Math.PI, Math.PI);

        IntStream.range((0),(4)).forEach((Index) -> {
        DRIVE_MOTOR.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drive.Constants.Measurements.ODOMETRY_FREQUENCY));
        ASIMUTH_MOTOR.setPeriodicFramePeriod(
            PeriodicFrame.kStatus2, (int) (1000d / org.robotalons.crescendo.subsystems.drive.Constants.Measurements.ODOMETRY_FREQUENCY));      
        });

        DRIVE_MOTOR.setCANTimeout((0));
        ASIMUTH_MOTOR.setCANTimeout((0));

        DRIVE_MOTOR.burnFlash();
        ASIMUTH_MOTOR.burnFlash();

        DRIVE_ENCODER.setPositionConversionFactor(Measurements.DRIVE_ENCODER_ROT2METER);
        ASIMUTH_ENCODER.setPositionConversionFactor(Measurements.ASIMUTH_ENCODER_ROT2RAD);
        DRIVE_ENCODER.setVelocityConversionFactor(Measurements.DRIVE_ENCODER_RPM2METER_SEC);
        ASIMUTH_ENCODER.setVelocityConversionFactor(Measurements.ASIMUTH_ENCODER_RPM2METER_SEC);
    }


    public int getNumber(){
      return NUMBER;
    }

    public double getDrivePosition() {
      return DRIVE_ENCODER.getPosition();
    }

    public double getTurningPosition() {
      return ASIMUTH_ENCODER.getPosition();
    }

    public double getDriveVelocity() {
      return DRIVE_ENCODER.getVelocity();
    }

    public double getTurningVelocity() {
      return ASIMUTH_ENCODER.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
      Double angle = ABSOULTE_ENCODER.getSupplyVoltage().getValue();
      angle /= RobotController.getVoltage5V();
      angle *= 2.0 * Math.PI;
      angle -= ABSOLUTE_ENCODER_OFFSET;
      return angle =  angle * (ABSOLUTE_ENCODER_REVERSED ? -1.0 : 1.0);
    }

    public void setTranslationalVoltage(final Double Voltage) {
      DRIVE_MOTOR.setVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
    }

    public void setASIMUTHVoltage(final Double Voltage) {
      ASIMUTH_MOTOR.setVoltage(MathUtil.clamp(Voltage, (-12d), (12d)));
    }

    public void resetEncoders() {
        DRIVE_ENCODER.setPosition(0);
        ASIMUTH_ENCODER.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            cease();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        DRIVE_MOTOR.set(state.speedMetersPerSecond / Constants.Measurements.ROBOT_MAXIMUM_DRIVE_VELOCITY);
        ASIMUTH_MOTOR.set(ASIMUTH_PID.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void calculateDrivePID(double setpoint){
      DRIVE_MOTOR.set(DRIVE_PID.calculate(getDrivePosition(), setpoint));
    }

    public void calculateAsimuthPID(double setpoint){
      ASIMUTH_MOTOR.set(ASIMUTH_PID.calculate(getTurningPosition(), setpoint));
    }
    
    //TODO: Fix to return proper posistion
    public SwerveModulePosition getModulePosition() {
      return new SwerveModulePosition();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
