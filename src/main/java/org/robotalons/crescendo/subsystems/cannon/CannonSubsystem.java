// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
import java.util.stream.IntStream;

import org.robotalons.crescendo.subsystems.cannon.Constants.Devices;
import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.lib.roller.Roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
  private static OrientationMode Control_Mode;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
  
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {} static {
    DIRECTIONAL_CONTROLLER = Devices.DIRECTIONAL_CONTROLLER;
    LAUNCH_CONTROLLER = Devices.LAUNCH_CONTROLLER;
    Control_Mode = OrientationMode.ROBOT_ORIENTED;

    // DIRECTIONAL_ENCODER = DIRECTIONAL_CONTROLLER.getEncoder();
    // LAUNCH_ENCODER = LAUNCH_CONTROLLER.getEncoder();
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    DIRECTIONAL_CONTROLLER.periodic();
    LAUNCH_CONTROLLER.periodic();
    Constants.Objects.ODOMETRY_LOCKER.lock();
  }
  
  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }
  
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
  public enum OrientationMode {
    OBJECT_ORIENTED,    
    ROBOT_ORIENTED,
    FIELD_ORIENTED,
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
   public static synchronized void set(Double DirectionalDemand, Double LaunchDemand) {
    switch(Control_Mode) {
      case OBJECT_ORIENTED:
        //TODO: AUTOMATION TEAM (OBJECT ORIENTATION DRIVEBASE)
        break;      
      case ROBOT_ORIENTED:
        set(new ChassisSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians()));      
        break;
      case FIELD_ORIENTED:
        set(ChassisSpeeds.fromFieldRelativeSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians(), 
          GYROSCOPE.getYawRotation()));      
        break;
    }
  }

    public static synchronized void set(final ChassisSpeeds Demand) {
      if (Demand.omegaRadiansPerSecond > (1e-6) && Demand.vxMetersPerSecond > (1e-6) && Demand.vyMetersPerSecond > (1e-6)) {
        set();
      } else {
          IntStream.range((0), MODULES.size()).boxed().map(
            (Index) -> 
              MODULES.get(Index).set(Reference[Index]))
            .toArray(SwerveModuleState[]::new));
      }
    }
  

  public static synchronized void set() {
    if(Module_Locking) {
      KINEMATICS.resetHeadings(MODULES.stream().map((Module) -> 
        Module.getObserved().angle
      ).toArray(Rotation2d[]::new));  
    }
    set(new ChassisSpeeds());
  }
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
