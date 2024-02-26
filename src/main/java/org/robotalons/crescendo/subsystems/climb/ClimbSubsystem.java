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
import org.robotalons.crescendo.subsystems.climb.Constants.Objects;
import org.robotalons.crescendo.subsystems.climb.Constants.Ports;

import java.io.Closeable;
import java.io.IOException;


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
public final class ClimbSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static CANSparkMax LEFT_ARM;
  private static CANSparkMax RIGHT_ARM;
  private static CANSparkMax[] MOTORS;

  private static Encoder LEFT_ENCODER;
  private static Encoder RIGHT_ENCODER;
  private static Encoder[] ENCODERS;

  private static ArmFeedforward LEFT_FF;
  private static ArmFeedforward RIGHT_FF; 
  private static ArmFeedforward[] FFS;
  
  private static SparkMaxPIDController LEFT_PID;
  private static SparkMaxPIDController RIGHT_PID;
  private static SparkMaxPIDController[] PIDS;

  private static Double K_GEARRATIO;
  private static Double K_TICKS_2_RAD;

  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  
  /**
   * Climb Subsystem Constructor
   */
  public ClimbSubsystem() {
    LEFT_ARM = new CANSparkMax(Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_ARM = new CANSparkMax(Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    
    LEFT_ENCODER = new Encoder(Measurements.K_LEFT_FORWARD_CHANNELA, Measurements.K_LEFT_FORWARD_CHANNELA);
    RIGHT_ENCODER = new Encoder(Measurements.K_RIGHT_FORWARD_CHANNELA, Measurements.K_RIGHT_FORWARD_CHANNELA);

    LEFT_PID = LEFT_ARM.getPIDController();
    RIGHT_PID = RIGHT_ARM.getPIDController();

    LEFT_FF = new ArmFeedforward(
      Constants.Measurements.LEFT_ARM_KS,
      Constants.Measurements.LEFT_ARM_KG,
      Constants.Measurements.LEFT_ARM_KV,
      Constants.Measurements.LEFT_ARM_KA);

    RIGHT_FF = new ArmFeedforward(
      Constants.Measurements.LEFT_ARM_KS,
      Constants.Measurements.LEFT_ARM_KG,
      Constants.Measurements.LEFT_ARM_KV,
      Constants.Measurements.LEFT_ARM_KA);

    MOTORS = new CANSparkMax[]{LEFT_ARM, RIGHT_ARM};
    ENCODERS = new Encoder[]{LEFT_ENCODER, RIGHT_ENCODER};
    PIDS = new SparkMaxPIDController[]{LEFT_PID, RIGHT_PID};
    FFS = new ArmFeedforward[]{LEFT_FF, RIGHT_FF};

    configPID();
    config();

    K_GEARRATIO = Measurements.K_GEARRATIO;
    K_TICKS_2_RAD = Measurements.K_TICKS_2_RAD;

  }
  
  // ------------------------------------------------------ ---------[Methods]--------------------------------------------------------------- //
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCKER.lock();

    Logger.recordOutput("Left Arm Posistion Radians", getPosistion(0));
    Logger.recordOutput("Right Arm Posistion Radians", getPosistion(1));

    Logger.recordOutput("Left Arm Velocity Radians", getVelocity(0));
    Logger.recordOutput("Right Arm Velocity Radians", getVelocity(1));

    Logger.recordOutput("Left Arm Temperature Radians", getTemperature(0));
    Logger.recordOutput("Right Arm Temperature Radians", getTemperature(1));

    Logger.recordOutput("Left Arm TempVoltageerature Radians", getVoltage(0));
    Logger.recordOutput("Right Arm Voltage Radians", getVoltage(1));    

    Objects.ODOMETRY_LOCKER.unlock();
  }
  
  /**
   * Closes this instance and all held resources immediately 
   * @throws IOException 
   */
  public synchronized void close() throws IOException {
    LEFT_ARM.close();
    RIGHT_ARM.close();
  }
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  
  /**
   * Mutates the current reference for both arms with a new demand, goal, or 'set-point'
   * @param LeftDemand Desired rotation of the left arm in radians
   * @param RightDemand Desired rotation of the right arm in radians
   */
  public synchronized void pidSet(final Double LeftDemand, final Double RightDemand) {
    LEFT_PID.setReference(
    LeftDemand, 
    ControlType.kPosition, 
    (0), 
    LEFT_FF.calculate(getPosistion(0), Measurements.K_HOLD_PARRALLEL_GROUND), 
    SparkMaxPIDController.ArbFFUnits.kVoltage);

    RIGHT_PID.setReference(
    LeftDemand, 
    ControlType.kPosition, 
    (0), 
    RIGHT_FF.calculate(getPosistion(1), Measurements.K_HOLD_PARRALLEL_GROUND), 
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
   * @throws IllegalArgumentException when number is not 0 or 1
   */ 
  public synchronized void set(final Integer NUM, final Double DEMAND) throws IllegalArgumentException{
    
    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'set' to throw error");
    }

    MOTORS[NUM].set(DEMAND);
  }

  /**
   * Sets the motor to a certain demand from 0 - 1 on the motors
   * @param DEMAND Desired demand on both motors
   */
  public synchronized void set(final Double DEMAND){
    RIGHT_ARM.set(DEMAND);
    LEFT_ARM.set(DEMAND);
  }

  private synchronized static void configPID() {
    LEFT_PID.setFeedbackDevice((MotorFeedbackSensor) LEFT_ENCODER);
    LEFT_PID.setP(Measurements.LEFT_ARM_P);
    LEFT_PID.setI(Measurements.LEFT_ARM_I);
    LEFT_PID.setD(Measurements.LEFT_ARM_D);
    
    RIGHT_PID.setFeedbackDevice((MotorFeedbackSensor) RIGHT_ENCODER);
    RIGHT_PID.setP(Measurements.RIGHT_ARM_P);
    RIGHT_PID.setI(Measurements.RIGHT_ARM_I);
    RIGHT_PID.setD(Measurements.RIGHT_ARM_D);
  }

  private synchronized static void config(){
    LEFT_ARM.setIdleMode(IdleMode.kBrake);
    LEFT_ARM.restoreFactoryDefaults();
    LEFT_ARM.clearFaults();
        
    LEFT_ARM.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);

    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kForward, 
    Measurements.K_FORWARD_ARM_LIMIT);
        
    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse, 
    Measurements.K_REVERSE_ARM_LIMIT);

    RIGHT_ARM.setIdleMode(IdleMode.kBrake);
    RIGHT_ARM.restoreFactoryDefaults();
    RIGHT_ARM.clearFaults();
        
    RIGHT_ARM.setSmartCurrentLimit(Measurements.K_CURRENT_LIMIT);

    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kForward, 
    Measurements.K_FORWARD_ARM_LIMIT);
        
    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse, 
    Measurements.K_REVERSE_ARM_LIMIT);
    }

  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  
  /**
   * Returns the motors' velocity for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Velocity of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */

  // TODO: Doulble Check Velocity 
  public static synchronized Double getVelocity(Integer NUM) throws IllegalArgumentException{
    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getVelocity' to throw error");
    }

    return ENCODERS[NUM].getDistancePerPulse();
  }

   /**
   * Returns the motors' posistion for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Posistion of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double getPosistion(Integer NUM) throws IllegalArgumentException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getPosistion' to throw error");
    }

    return ENCODERS[NUM].getDistance() * K_TICKS_2_RAD / K_GEARRATIO;
  }

   /**
   * Returns the motors' temperature for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Temperature of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double getTemperature(Integer NUM) throws IllegalArgumentException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getTemperature' to throw error");
    }

    return MOTORS[NUM].getMotorTemperature();
  }

   /**
   * Returns the motors' voltage for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Voltage of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double getVoltage(Integer NUM) throws IllegalArgumentException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getVoltage' to throw error");
    }

    return MOTORS[NUM].getBusVoltage();
  }

   /**
   * Returns the motors' PID constants for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return PID Constants of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double[] getPIDConstants(Integer NUM) throws IllegalArgumentException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getPIDConstants' to throw error");
    }

    return new Double[]{PIDS[NUM].getP(), PIDS[NUM].getI(), PIDS[NUM].getD()};
  }

   /**
   * Returns the motors' FeedForward constants for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return FeedForward Constants of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double[] getFFConstants(Integer NUM) throws IllegalArgumentException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'getFFConstants' to throw error");
    }

    return new Double[]{FFS[NUM].ka, FFS[NUM].kg, FFS[NUM].ks, FFS[NUM].kv};
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
