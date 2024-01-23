// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.Logger;

// ---------------------------------------------------------------[Libraries]--------------------------------------------------------------- //


// ------------------------------------------------------------[Climb Subsystem]------------------------------------------------------------ //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes from and to the intake and shooter.<p>
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */
public class ClimbSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final CANSparkMax LEFT_CONTROLLER;  
  private static final CANSparkMax RIGHT_CONTROLLER; 
  private static final RelativeEncoder LEFT_ENCODER;
  private static final RelativeEncoder RIGHT_ENCODER;
  private static final SparkMaxPIDController LEFT_PID;
  private static final SparkMaxPIDController RIGHT_PID;
  private static final ArmFeedforward LEFT_FEEDFORWARD;
  private static final ArmFeedforward RIGHT_FEEDFORWARD;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /**
   * Indexer Subsystem Constructor
   */
  public ClimbSubsystem() {} static {
    LEFT_CONTROLLER = new CANSparkMax(Constants.Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_CONTROLLER = new CANSparkMax(Constants.Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    LEFT_ENCODER = LEFT_CONTROLLER.getEncoder();
    //TODO: Add Left Encoder Tick To Radian Configurations
    RIGHT_ENCODER = RIGHT_CONTROLLER.getEncoder();
    //TODO: Add Right Encoder Tick To Radian Configurations
    LEFT_PID = LEFT_CONTROLLER.getPIDController();
    //TODO: Add Left PID Configurations
    RIGHT_PID = RIGHT_CONTROLLER.getPIDController();
    //TODO: Add Right PID Configurations

    LEFT_FEEDFORWARD = new ArmFeedforward(
      Constants.Measurements.LEFT_ARM_KS,
      Constants.Measurements.LEFT_ARM_KG,
      Constants.Measurements.LEFT_ARM_KV,
      Constants.Measurements.LEFT_ARM_KA
    );

    RIGHT_FEEDFORWARD = new ArmFeedforward(
      Constants.Measurements.RIGHT_ARM_KS,
      Constants.Measurements.RIGHT_ARM_KG,
      Constants.Measurements.RIGHT_ARM_KV,
      Constants.Measurements.RIGHT_ARM_KA
    );
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    //TODO: Updates Odometry (Multi-Thread Safe)
    Constants.Objects.ODOMETRY_LOCKER.lock();
    Logger.recordOutput(("Climb/LeftEncoder"),LEFT_ENCODER.getPosition());
    Logger.recordOutput(("Climb/RightEncoder"),RIGHT_ENCODER.getPosition());
  }
  
  /**
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    LEFT_CONTROLLER.close();
    RIGHT_CONTROLLER.close();
  }
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  /**
   * Mutates the current reference for both arms with a new demand, goal, or 'set-point'
   * @param LeftDemand Desired rotation of the left arm in radians
   * @param RightDemand Desired rotation of the right arm in radians
   */
  public synchronized void set(final Double LeftDemand, final Double RightDemand) {
    //TODO: PID Implementation Here
    LEFT_PID.setReference(
      LeftDemand, 
      ControlType.kPosition, 
      (0), 
      LEFT_FEEDFORWARD.calculate(0, 0), 
      SparkMaxPIDController.ArbFFUnits.kVoltage);
    RIGHT_PID.setReference(
      RightDemand, 
      ControlType.kPosition, 
      (0), 
      RIGHT_FEEDFORWARD.calculate(0, 0), 
      SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  /**
   * Mutates the current reference with a new demand, goal, or 'set-point'
   * @param Demand Desired rotation of both arms in radians
   */
  public synchronized void set(final Double Demand) {
    set(Demand,Demand);
  }
  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  
  //TODO: Add Arm Angle & Velocity Acessors (PROPERLY JAVADOC COMMENTED)

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized ClimbSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new ClimbSubsystem();
      return Instance;
  }
}
