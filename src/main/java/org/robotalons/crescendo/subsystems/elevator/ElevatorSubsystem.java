// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.robotalons.crescendo.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.elevator.Constants.Devices;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.elevator.ElevatorModule;
import org.robotalons.lib.utilities.Operator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;



public class ElevatorSubsystem extends TalonSubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final ElevatorModule MODULE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Operator CurrentOperator;
  private static Double CurrentTime;
  private static ElevatorSubsystem Instance;
  private static Pose2d Position;

  public ElevatorSubsystem() {
    super(("Elevator Subsystem"));
  } static {
    Instance = new ElevatorSubsystem();
    CurrentTime = Timer.getFPGATimestamp();
    MODULE = Devices.ELEVATOR_MODULE;
    Logger.recordOutput(("Elevator/Position"), new Pose2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO Kinematics
    Logger.recordOutput(("Elevator/Position"), new Pose2d());
  }

  @Override
  public void configure(Operator Operator) {
    CurrentOperator = Operator;
    // Nothing here yet...
  }

  @Override
  public void close() {
    MODULE.close();
  }

  @Override
  public Operator getOperator() {
    return CurrentOperator;
  }

  public static void setElevator(double speed){
    MODULE.setVoltage(speed);
  }

  public static void setRoller(double speed){
    MODULE.setRollerVoltage(speed);
  }
}
