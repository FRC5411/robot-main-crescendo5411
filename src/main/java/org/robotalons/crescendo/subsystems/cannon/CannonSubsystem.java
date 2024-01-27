// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
import org.robotalons.crescendo.subsystems.cannon.Constants.Devices;
import org.robotalons.lib.roller.Roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private static final Roller DIRECTIONAL_CONTROLLER;
  private static final Roller LAUNCH_CONTROLLER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
  
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {
    DIRECTIONAL_CONTROLLER = Devices.DIRECTIONAL_CONTROLLER;
    LAUNCH_CONTROLLER = Devices.LAUNCH_CONTROLLER;
    // DIRECTIONAL_ENCODER = DIRECTIONAL_CONTROLLER.getEncoder();
    // LAUNCH_ENCODER = LAUNCH_CONTROLLER.getEncoder();
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
  
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
  
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
