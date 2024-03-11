// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.climb;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.climb.Constants.Measurements;
import org.robotalons.crescendo.subsystems.climb.Constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  public static CANSparkMax LEFT_ARM;
  public static CANSparkMax RIGHT_ARM;
  
  public static RelativeEncoder LEFT_ENCODER;
  public static RelativeEncoder RIGHT_ENCODER;

  public static PIDController LEFT_PID;
  public static PIDController RIGHT_PID;

  public static ArmFeedforward LEFT_FF;
  public static ArmFeedforward RIGHT_FF;

  public static CANSparkMax[] MODULES;
  public static RelativeEncoder[] ENCODERS; 
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {} static {
    LEFT_ARM = new CANSparkMax(Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_ARM = new CANSparkMax(Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);

    LEFT_ENCODER = LEFT_ARM.getEncoder();
    RIGHT_ENCODER = RIGHT_ARM.getEncoder();

    LEFT_PID = new PIDController(Measurements.LEFT_ARM_P, Measurements.LEFT_ARM_I, Measurements.LEFT_ARM_D);
    RIGHT_PID = new PIDController(Measurements.RIGHT_ARM_P, Measurements.RIGHT_ARM_I, Measurements.RIGHT_ARM_D);

    LEFT_FF = new ArmFeedforward(Measurements.LEFT_ARM_KS, Measurements.LEFT_ARM_KG, Measurements.LEFT_ARM_KV, Measurements.LEFT_ARM_KA);
    RIGHT_FF = new ArmFeedforward(Measurements.RIGHT_ARM_KS, Measurements.RIGHT_ARM_KG, Measurements.RIGHT_ARM_KV, Measurements.RIGHT_ARM_KA);

    MODULES = new CANSparkMax[]{LEFT_ARM, RIGHT_ARM};

    configure();
  }

  public synchronized void set(final int Direction, final Double Demand) {
    if(Direction == 0){
      LEFT_ARM.setVoltage(LEFT_PID.calculate(getPosition(0), Demand) + LEFT_FF.calculate(Demand, (0)));}

    else{
      RIGHT_ARM.setVoltage(RIGHT_PID.calculate(getPosition(1), Demand) + RIGHT_FF.calculate(Demand, (0)));}
  }

  /**
   * Mutates the demand velocities of both sides of the climb subsystem
   * @param Demand Queried load on the motor controller object, which must lie between -1 and +1
   */ 
  public synchronized void set(final Double Demand) {
    LEFT_ARM.setVoltage(LEFT_PID.calculate(getPosition(0), Demand) + LEFT_FF.calculate(Demand, (0)));
    RIGHT_ARM.setVoltage(RIGHT_PID.calculate(getPosition(1), Demand) + RIGHT_FF.calculate(Demand, (0)));
  }

  public static void configure(){
    LEFT_ARM.setIdleMode(IdleMode.kBrake);
    LEFT_ARM.restoreFactoryDefaults();
    LEFT_ARM.clearFaults();

    LEFT_PID.setP(Measurements.LEFT_ARM_P);
    LEFT_PID.setI(Measurements.LEFT_ARM_I);
    LEFT_PID.setD(Measurements.LEFT_ARM_D);
    
    RIGHT_PID.setP(Measurements.RIGHT_ARM_P);
    RIGHT_PID.setI(Measurements.RIGHT_ARM_I);
    RIGHT_PID.setD(Measurements.RIGHT_ARM_D);
        
    LEFT_ARM.setSmartCurrentLimit(Measurements.CURRENT_LIMIT);

    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kForward, 
    Measurements.FORWARD_ARM_LIMIT);
        
    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse, 
    Measurements.REVERSE_ARM_LIMIT);

    RIGHT_ARM.setIdleMode(IdleMode.kBrake);
    RIGHT_ARM.restoreFactoryDefaults();
    RIGHT_ARM.clearFaults();
        
    RIGHT_ARM.setSmartCurrentLimit(Measurements.CURRENT_LIMIT);

    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kForward, 
    Measurements.FORWARD_ARM_LIMIT);
        
    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse, 
    Measurements.FORWARD_ARM_LIMIT);
  }

  public static synchronized double getPosition(final int climb) {
    return ENCODERS[climb].getPosition() * Measurements.TICK_CONVERSION_FACTOR;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climb/ Left Arm Posistion", getPosition(0));
    Logger.recordOutput("Climb/ Right Arm Posistion", getPosition(1));
  }
}
