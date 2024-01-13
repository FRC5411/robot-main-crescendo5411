// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

import java.util.List;
import java.util.Set;

import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.lib.utilities.PilotProfile;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>RobotConstants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see RobotContainer
 */
public final class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//

  public static final class Subsystems {
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
    public static final DrivebaseSubsystem DRIVEBASE_SUBSYSTEM = DrivebaseSubsystem.getInstance();
    public static final Set<Subsystem> SUBSYSTEMS = Set.of(
      DrivebaseSubsystem.getInstance()
    );
  }

  public static final class Logging {
    public static final String LOGGING_DEPOSIT_FOLDER = ("src\\main\\java\\main\\deploy\\logs");
    public static final Boolean LOGGING_TURBO_MODE = (false);
    public static final Boolean LOGGING_ENABLED = (false);
    public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (0);
    public static final Integer PATHPLANNER_SERVER = (6969);
  }

  public static final class Profiles { 

    public static final List<PilotProfile> PILOT_PROFILES = List.of(John.PROFILE);

    public static final class PreferenceNames {
      public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
      public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
      public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
      public static final String TRANSLATIONAL_X_DEADZONE = ("TRANSLATIONAL_X_DEADZONE");
      public static final String TRANSLATIONAL_Y_DEADZONE = ("TRANSLATIONAL_Y_DEADZONE");
      public static final String ORIENTATION_DEADZONE = ("ORIENTATION_DEADZONE");
    }

    public static final class KeybindingNames {
      public static final String LOCKING_TOGGLE_TRIGGER = ("LOCKING_ENABLED_TRIGGER");
      public static final String FIELD_ORIENTED_TOGGLE = ("FIELD_ORIENTED_TOGGLE");
    }

    public static final class John {
      public static final Integer CONTROLLER_PORT = (0);
      public static final CommandXboxController CONTROLLER = new CommandXboxController(CONTROLLER_PORT);
      public static final PilotProfile PROFILE = new PilotProfile(("John Doe"))
        .addPreference(PreferenceNames.TRANSLATIONAL_X_INPUT, () -> CONTROLLER.getRawAxis((0)))
        .addPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT, () -> CONTROLLER.getRawAxis((1)))
        .addPreference(PreferenceNames.ORIENTATION_INPUT, () -> CONTROLLER.getRawAxis((4)))
        .addPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE, () -> (0.1))
        .addPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE, () -> (0.1))
        .addPreference(PreferenceNames.ORIENTATION_DEADZONE, () -> (0.1))
        .addKeybinding(KeybindingNames.LOCKING_TOGGLE_TRIGGER, CONTROLLER.a())
        .addKeybinding(KeybindingNames.FIELD_ORIENTED_TOGGLE, CONTROLLER.b());
    }
  }
}
