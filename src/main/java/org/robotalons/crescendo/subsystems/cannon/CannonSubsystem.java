// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// ------------------------------------------------------------[Cannon Subsystem]-----------------------------------------------------------//
/**
 *
 *
 * <h1>CannonSubsystem</h1>
 *
 * <p>Utility class which controls the firing of objects to a given target, based on the current angle in radians, distance to target, and 
 * robot drivebase states.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class CannonSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //

  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
    // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {

  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    
    Constants.Objects.ODOMETRY_LOCKER.lock();
  }
  
  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //

  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized CannonSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new CannonSubsystem();
      return Instance;
  }
}
