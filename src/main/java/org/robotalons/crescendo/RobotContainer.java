// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.PreferenceNames;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem.OrientationMode;
import org.robotalons.lib.utilities.PilotProfile;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>RobotContainer</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 */
public final class RobotContainer {
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  public static PilotProfile DRIVEBASE_PILOT = Profiles.John.PROFILE;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
        Constants.Subsystems.DRIVEBASE_SUBSYSTEM.setDefaultCommand(
      new InstantCommand(() ->
        DrivebaseSubsystem.set(
          new Translation2d(
                applyInputSquare(MathUtil.applyDeadband(-(Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_X_INPUT),
              (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE))),
                applyInputSquare(MathUtil.applyDeadband(-(Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT),
              (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE)))),
          new Rotation2d(
                applyInputSquare(MathUtil.applyDeadband(-(Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.ORIENTATION_INPUT),
              (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.ORIENTATION_DEADZONE)))),
          OrientationMode.ROBOT_ORIENTED), 
          DrivebaseSubsystem.getInstance()
    ));
  }

  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Applies squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  public static Double applyInputSquare(final Double Input) {
    return Math.copySign(Input * Input, Input);
  }
  
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized RobotContainer getInstance() {
      if (java.util.Objects.isNull(Instance)) {
          Instance = new RobotContainer();
      }
      return Instance;
  }
}
