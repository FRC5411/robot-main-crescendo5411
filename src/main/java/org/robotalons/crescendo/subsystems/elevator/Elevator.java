// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package org.robotalons.crescendo.subsystems.elevator;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {
//   /** Creates a new Elevator. */
//   private static CANSparkMax elevatorMotor;
//   private static CANSparkMax rollerMotor;

//   public Elevator() {
//     elevatorMotor = new CANSparkMax(51, MotorType.kBrushless);
//     rollerMotor = new CANSparkMax(52, MotorType.kBrushless);
//   }

//   public static void setElevator(double speed){
//     elevatorMotor.set(speed);
//   }

//   public static void setRoller(double speed){
//     rollerMotor.set(speed);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
