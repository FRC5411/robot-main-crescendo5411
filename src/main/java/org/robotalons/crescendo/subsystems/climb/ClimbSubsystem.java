// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
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
  private static final CANSparkMax LEFT_ARM;  
  private static final CANSparkMax RIGHT_ARM; 
  private static final RelativeEncoder LEFT_ENCODER;
  private static final RelativeEncoder RIGHT_ENCODER;
  private static final SparkMaxPIDController LEFT_PID;
  private static final SparkMaxPIDController RIGHT_PID;
  private static final ArmFeedforward LEFT_FEEDFORWARD;
  private static final ArmFeedforward RIGHT_FEEDFORWARD;
  private static final CANSparkMax[] MOTORS;
  private static final RelativeEncoder[] ENCODERS;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /**
   * Indexer Subsystem Constructor
   */
  public ClimbSubsystem() {} static {
    LEFT_ARM = new CANSparkMax(Constants.Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_ARM = new CANSparkMax(Constants.Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);

    LEFT_ENCODER = LEFT_ARM.getEncoder();
    LEFT_ENCODER.setPositionConversionFactor(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.K_TICKS_2_RAD);

    RIGHT_ENCODER = RIGHT_ARM.getEncoder();
    RIGHT_ENCODER.setPositionConversionFactor(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.K_TICKS_2_RAD);


    LEFT_PID = LEFT_ARM.getPIDController();
    RIGHT_PID = RIGHT_ARM.getPIDController();

    configPID();

    config(RIGHT_ARM);
    config(LEFT_ARM);

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

    MOTORS = new CANSparkMax[]{LEFT_ARM, RIGHT_ARM};
    ENCODERS = new RelativeEncoder[]{LEFT_ENCODER, RIGHT_ENCODER};

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
    LEFT_ARM.close();
    RIGHT_ARM.close();
  }

  // TODO: Check
  public synchronized static void configPID(){

    LEFT_PID.setFeedbackDevice(LEFT_ENCODER);
    LEFT_PID.setP(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.LEFT_ARM_P);
    LEFT_PID.setI(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.LEFT_ARM_I);
    LEFT_PID.setD(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.LEFT_ARM_D);

    RIGHT_PID.setFeedbackDevice(RIGHT_ENCODER);
    RIGHT_PID.setP(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.RIGHT_ARM_P);
    RIGHT_PID.setI(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.RIGHT_ARM_I);
    RIGHT_PID.setD(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.RIGHT_ARM_D);
  }

  //TODO: Run by Isaac
  public synchronized static void config(CANSparkMax motor){
    motor.setIdleMode(IdleMode.kCoast);
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    
    motor.setSmartCurrentLimit(org.robotalons.crescendo.subsystems.climb.Constants.Measurements.K_CURRENT_LIMIT);

    motor.setSoftLimit(
      SoftLimitDirection.kForward, 
      org.robotalons.crescendo.subsystems.climb.Constants.Measurements.K_FORWARD_ARM_LIMIT);
    
    motor.setSoftLimit(
      SoftLimitDirection.kReverse, 
      org.robotalons.crescendo.subsystems.climb.Constants.Measurements.K_REVERSE_ARM_LIMIT);
  }
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  
  /**
   * Mutates the current reference for both arms with a new demand, goal, or 'set-point'
   * @param LeftDemand Desired rotation of the left arm in radians
   * @param RightDemand Desired rotation of the right arm in radians
   */
  public synchronized void pidSet(final Double LeftDemand, final Double RightDemand) {
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
   * @param DEMAND Desired rotation of both arms in radians
   */
  public synchronized void pidSet(final Double DEMAND) {
    pidSet(DEMAND,DEMAND);
  }

  /**
   * Sets the selected motor to a certain demand from 0 - 1 on the motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Desired Motor that wants to be called
   * @param DEMAND Desired demand on the motor
   */
  public synchronized void set(final Integer NUM, final Double DEMAND){
    CANSparkMax MOTOR = MOTORS[NUM];
    MOTOR.set(DEMAND);
  }

  /**
   * Sets the motor to a certain demand from 0 - 1 on the motors
   * @param DEMAND Desired demand on both motors
   */
  public synchronized void set(final Double DEMAND){
    RIGHT_ARM.set(DEMAND);
    LEFT_ARM.set(DEMAND);
  }
  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  
  /**
   * Returns the motors' velocity for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Velocity of Motor
   */
  public static synchronized Double getVelocity(Integer NUM){
    RelativeEncoder ENCODER = ENCODERS[NUM];
    return ENCODER.getVelocity();
  }

   /**
   * Returns the motors' posistion for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Posistion of Motor
   */
  public static synchronized Double getPosistion(Integer NUM){
    RelativeEncoder ENCODER = ENCODERS[NUM];
    return ENCODER.getPosition();
  }

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
