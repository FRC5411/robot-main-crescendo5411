// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.cannon;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CannonSubsystem extends SubsystemBase {

  private static TalonFX LEFT_MOTOR;
  private static TalonFX RIGHT_MOTOR;

  private static VisionSubsystem vision;

  // private final DigitalOutput INDEXER_SENSOR;
 
  private static CANSparkMax PIVOT_MOTOR;
  private static PIDController PIVOT_CONTROLLER_PID;

  private static DutyCycleEncoder PIVOT_ABSOLUTE_ENCODER;

  /** Creates a new CannonSubsystem. */
  public CannonSubsystem() {
    // Motors
    LEFT_MOTOR = new TalonFX(Constants.Ports.FIRING_CONTROLLER_LEFT_ID);

    RIGHT_MOTOR = new TalonFX(Constants.Ports.FIRING_CONTROLLER_RIGHT_ID);

    PIVOT_MOTOR = new CANSparkMax(Constants.Ports.PIVOT_CONTROLLER_ID, MotorType.kBrushless);

    // PIDS
    PIVOT_CONTROLLER_PID = new PIDController(
      Constants.Measurements.PIVOT_P_GAIN, 
      Constants.Measurements.PIVOT_I_GAIN, 
      Constants.Measurements.PIVOT_D_GAIN);


    PIVOT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Constants.Ports.PIVOT_ABSOLUTE_ENCODER_ID);

    vision = new VisionSubsystem();

    config();

  }

  //TODO: Make sure I did the interpolration correctly
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final var Target = vision.getAprilTagPose((DrivebaseSubsystem.getRotation().getRadians() % (2) * Math.PI >= Math.PI)? (3): (7)).get();
  
    double Distance = (PhotonUtils.getDistanceToPose(DrivebaseSubsystem.getPose(), Target));

    final var Interpolated = Measurements.PIVOT_FIRING_MAP.interpolate(
      Measurements.PIVOT_LOWER_BOUND,
      Measurements.PIVOT_UPPER_BOUND,
      Math.hypot((double) Distance, Measurements.SPEAKER_HEIGHT_METERS) / Math.hypot(Measurements.PIVOT_MAXIMUM_RANGE_METERS, Measurements.SPEAKER_HEIGHT_METERS));

    if(Interpolated != (null)) {
      final var Percentage = 
      (Math.abs(RIGHT_MOTOR.getVelocity().getValue() / Interpolated.get((0), (0))) + (Math.abs(Units.rotationsToDegrees(getPivotRotation())) / Interpolated.get((1), (0)))) / 2;

      setPivotAngle(Interpolated.get(1, 0));
    

    Logger.recordOutput(("Cannon/InterpolatedDistance"), Distance); 
    Logger.recordOutput(("Cannon/InterpolatedPercentile"), Percentage);
    Logger.recordOutput(("Cannon/InterpolatedVelocity"), Interpolated.get((0), (0)));
    Logger.recordOutput(("Cannon/InterpolatedRotation"), Units.radiansToDegrees(Interpolated.get((1), (0))));   
    }   

    else{
    Logger.recordOutput(("Cannon/InterpolatedDistance"), 0d); 
    Logger.recordOutput(("Cannon/InterpolatedPercentile"), 0d);
    Logger.recordOutput(("Cannon/InterpolatedVelocity"), 0d);
    Logger.recordOutput(("Cannon/InterpolatedRotation"), 0d);        
    }

    Logger.recordOutput(("Cannon/MeasuredRotation"), getPivotRotation());
  }

  //TODO: Add other set methods
  public static void setPivotAngle(double angle){
    PIVOT_MOTOR.set(PIVOT_CONTROLLER_PID.calculate(getPivotRotation(), angle));
  } 

  public static void pivotUp(){
    PIVOT_MOTOR.set((0.5));
  }

  public static void pivotDown(){
    PIVOT_MOTOR.set((-0.5));
  }

  public static void pivotStop(){
    PIVOT_MOTOR.set((0));
  }

  public static void shoot(){
    LEFT_MOTOR.set((0.8));
    RIGHT_MOTOR.set((0.8));
  }

  public static void stop(){
    LEFT_MOTOR.set((0));
    RIGHT_MOTOR.set((0));
  }  
  
  public static void reverse(){
    LEFT_MOTOR.set(-(0.8));
    RIGHT_MOTOR.set(-(0.8));
  }

  private static double getPivotRotation() {
    return Units.rotationsToRadians((PIVOT_ABSOLUTE_ENCODER.getAbsolutePosition() - Constants.Measurements.ABSOLUTE_ENCODER_OFFSET));
  }

  public void config(){
    LEFT_MOTOR.clearStickyFaults();
    LEFT_MOTOR.setNeutralMode(NeutralModeValue.Brake);
    LEFT_MOTOR.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
    LEFT_MOTOR.setInverted((false));

    RIGHT_MOTOR.clearStickyFaults();
    RIGHT_MOTOR.setNeutralMode(NeutralModeValue.Brake);
    RIGHT_MOTOR.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
    RIGHT_MOTOR.setInverted((false));

    PIVOT_MOTOR.clearFaults();
    PIVOT_MOTOR.setIdleMode(IdleMode.kBrake);
    PIVOT_MOTOR.setSmartCurrentLimit(40);
    PIVOT_MOTOR.setInverted((false));

    PIVOT_MOTOR.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.Measurements.PIVOT_MINIMUM_ROTATION);
    PIVOT_MOTOR.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.Measurements.PIVOT_MAXIMUM_ROTATION);

    PIVOT_CONTROLLER_PID.enableContinuousInput(Measurements.PIVOT_MINIMUM_ROTATION, Measurements.PIVOT_MAXIMUM_ROTATION);
  }
}
