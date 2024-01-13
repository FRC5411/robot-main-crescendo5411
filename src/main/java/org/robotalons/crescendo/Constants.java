// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.lib.utilities.CTREOdometryThread;
import org.robotalons.lib.utilities.PilotProfile;
import org.robotalons.lib.utilities.REVOdometryThread;

import java.util.List;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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

  public static final class Odometry {
    public static final Lock ODOMETRY_LOCK = new ReentrantLock();
    public static final CTREOdometryThread CTRE_ODOMETRY_THREAD = CTREOdometryThread.create(ODOMETRY_LOCK);
    public static final REVOdometryThread REV_ODOMETRY_THREAD = REVOdometryThread.create(ODOMETRY_LOCK);
  }

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (0);
  }

  public static final class Profiles { 

    public static final List<PilotProfile> PILOT_PROFILES = List.of(
      Example.PROFILE
    );

    public static final class PreferenceNames {
      public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
      public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
      public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
      public static final String TRANSLATIONAL_X_DEADZONE = ("TRANSLATIONAL_X_DEADZONE");
      public static final String TRANSLATIONAL_Y_DEADZONE = ("TRANSLATIONAL_Y_DEADZONE");
      public static final String ORIENTATION_DEADZONE = ("ORIENTATION_DEADZONE");
    }

    public static final class KeybindingNames {
      public static final String MODULE_LOCKING_TOGGLE = ("LOCKING_ENABLED_TRIGGER");
      public static final String ORIENTATION_TOGGLE = ("ORIENTATION_TOGGLE");
      public static final String PATHFINDING_FLIP_TOGGLE = ("PATHFINDING_FLIP_TOGGLE");
    }

    public static final class Example {
      public static final Integer CONTROLLER_PORT = (0);
      public static final CommandXboxController CONTROLLER = new CommandXboxController(CONTROLLER_PORT);
      public static final PilotProfile PROFILE = new PilotProfile(("John Doe"))
        .addPreference(PreferenceNames.TRANSLATIONAL_X_INPUT, () -> CONTROLLER.getRawAxis((1)))
        .addPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT, () -> CONTROLLER.getRawAxis((0)))
        .addPreference(PreferenceNames.ORIENTATION_INPUT, () -> CONTROLLER.getRawAxis((4)))
        .addPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE, () -> (0.1))
        .addPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE, () -> (0.1))
        .addPreference(PreferenceNames.ORIENTATION_DEADZONE, () -> (0.1))
        .addKeybinding(KeybindingNames.MODULE_LOCKING_TOGGLE, CONTROLLER.a())
        .addKeybinding(KeybindingNames.ORIENTATION_TOGGLE, CONTROLLER.b())
        .addKeybinding(KeybindingNames.PATHFINDING_FLIP_TOGGLE, CONTROLLER.x());
    }
  }
}
