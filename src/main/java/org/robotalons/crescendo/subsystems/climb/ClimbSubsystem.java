// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.climb;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.climb.Constants.Measurements;

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
  private static final Encoder LEFT_ENCODER;
  private static final Encoder RIGHT_ENCODER;
  private static final SparkMaxPIDController LEFT_PID;
  private static final SparkMaxPIDController RIGHT_PID;
  private static final ArmFeedforward LEFT_FEEDFORWARD;
  private static final ArmFeedforward RIGHT_FEEDFORWARD;
  private static final CANSparkMax[] MOTORS;
  private static final Encoder[] ENCODERS;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /**
   * Indexer Subsystem Constructor
   */
  public ClimbSubsystem() {} static {
    LEFT_ARM = new CANSparkMax(Constants.Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_ARM = new CANSparkMax(Constants.Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);

    LEFT_ENCODER = new Encoder(Measurements.K_LEFT_FORWARD_CHANNELA, Measurements.K_LEFT_FORWARD_CHANNELB);

    RIGHT_ENCODER = new Encoder(Measurements.K_RIGHT_FORWARD_CHANNELA, Measurements.K_RIGHT_FORWARD_CHANNELB);

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
    ENCODERS = new Encoder[]{LEFT_ENCODER, RIGHT_ENCODER};

  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    //TODO: Updates Odometry (Multi-Thread Safe)
    Constants.Objects.ODOMETRY_LOCKER.lock();
    Logger.recordOutput(("Climb/RightEncoder/Pos"), getPosistion(1));
    Logger.recordOutput(("Climb/RightEncoder/Velocity"), getVelocity(1));
    Logger.recordOutput(("Climb/RightEncoder/Temperature"), getTemperature(1));

    Logger.recordOutput(("Climb/LeftEncoder/Pos"), getPosistion(0));
    Logger.recordOutput(("Climb/LeftEncoder/Velocity"), getVelocity(0));
    Logger.recordOutput(("Climb/LeftEncoder/Temperature"), getTemperature(0) );
  }
  
  /**
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    LEFT_ARM.close();
    RIGHT_ARM.close();
  }

  public synchronized static void configPID(){
    LEFT_PID.setFeedbackDevice((MotorFeedbackSensor) LEFT_ENCODER);
    LEFT_PID.setP(Measurements.LEFT_ARM_P);
    LEFT_PID.setI(Measurements.LEFT_ARM_I);
    LEFT_PID.setD(Measurements.LEFT_ARM_D);

    RIGHT_PID.setFeedbackDevice((MotorFeedbackSensor) RIGHT_ENCODER);
    RIGHT_PID.setP(Measurements.RIGHT_ARM_P);
    RIGHT_PID.setI(Measurements.RIGHT_ARM_I);
    RIGHT_PID.setD(Measurements.RIGHT_ARM_D);
  }

  //TODO: Run by Isaac
  public synchronized static void config(CANSparkMax motor){
    motor.setIdleMode(IdleMode.kCoast);
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    
    motor.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);

    motor.setSoftLimit(
      SoftLimitDirection.kForward, 
      Measurements.K_FORWARD_ARM_LIMIT);
    
    motor.setSoftLimit(
      SoftLimitDirection.kReverse, 
      Measurements.K_REVERSE_ARM_LIMIT);
  }
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  
  /**
   * Mutates the current reference for both arms with a new demand, goal, or 'set-point'
   * @param LeftDemand Desired rotation of the left arm in radians
   * @param RightDemand Desired rotation of the right arm in radians
   */
  public synchronized void pidSet(final Double LeftDemand, final Double RightDemand) {
    //TODO: Double check PID slot
    LEFT_PID.setReference(
      LeftDemand, 
      ControlType.kPosition, 
      (0), 
      LEFT_FEEDFORWARD.calculate(getPosistion(0), getVelocity(0)), 
      SparkMaxPIDController.ArbFFUnits.kVoltage);
    
      RIGHT_PID.setReference(
      RightDemand, 
      ControlType.kPosition, 
      (1), 
      RIGHT_FEEDFORWARD.calculate(getPosistion(1), getVelocity(1)), 
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
  // TODO: Doulble Check
  public static synchronized Double getVelocity(Integer NUM){
    Encoder ENCODER = ENCODERS[NUM];
    return ENCODER.getDistancePerPulse();
  }

   /**
   * Returns the motors' posistion for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Posistion of Motor
   */
  public static synchronized Double getPosistion(Integer NUM){
    Encoder ENCODER = ENCODERS[NUM];
    return ENCODER.getDistance() * Measurements.K_TICKS_2_RAD / Measurements.K_GEARRATIO;
  }

   /**
   * Returns the motors' temperature for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Temperature of Motor
   */
  public static synchronized Double getTemperature(Integer NUM){
    CANSparkMax MOTOR = MOTORS[NUM];
    return MOTOR.getMotorTemperature();
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
