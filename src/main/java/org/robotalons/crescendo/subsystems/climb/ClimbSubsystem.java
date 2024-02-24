// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.climb;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.Closeable;
import java.io.IOException;
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
public final class ClimbSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static ClimbModule LEFT_ARM;
  private static ClimbModule RIGHT_ARM;
  private static ClimbModule[] MOTORS;

  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  
  /**
   * Climb Subsystem Constructor
   */
  public ClimbSubsystem() {
    LEFT_ARM = new ClimbModule(0, Measurements.K_LEFT_FORWARD_CHANNELA, Measurements.K_LEFT_FORWARD_CHANNELB);
    RIGHT_ARM = new ClimbModule(1, Measurements.K_RIGHT_FORWARD_CHANNELA, Measurements.K_RIGHT_FORWARD_CHANNELB);

    MOTORS = new ClimbModule[]{LEFT_ARM, RIGHT_ARM};

  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {}
  
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
  public synchronized void pidSet(final Double LeftDemand, final Double RightDemand) {}

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

    ClimbModule MOTOR = MOTORS[NUM];
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
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  // TODO: Doulble Check
  public static synchronized Double getVelocity(Integer NUM) throws IllegalArgumentException{
    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'set' to throw error");
    }

    ClimbModule MOTOR = MOTORS[NUM];
    return MOTOR.getVelocity();
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
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'set' to throw error");
    }

    ClimbModule MOTOR = MOTORS[NUM];
    return MOTOR.getPosistion();
  }

   /**
   * Returns the motors' temperature for the specified motors
   * 0 - Left Arm, 1 - Right Arm
   * @param NUM Picks Motors
   * @return Temperature of Motor
   * @throws IllegalArgumentException when number is not 0 or 1
   */
  public static synchronized Double getTemperature(Integer NUM) throws IllegalAccessException{

    if(!(NUM == 1 || NUM == 0)){
      throw new IllegalArgumentException("Parameter 'NUM' was not 0 or 1, causing method 'set' to throw error");
    }

    ClimbModule MOTOR = MOTORS[NUM];
    return MOTOR.getTemperature();
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
