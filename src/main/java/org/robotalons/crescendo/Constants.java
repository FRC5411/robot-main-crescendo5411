// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import org.robotalons.crescendo.subsystems.TalonSubsystemBase;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;
import org.robotalons.lib.utilities.PilotProfile;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
    public static final Field2d ROBOT_FIELD = new Field2d();
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
    public static final DrivebaseSubsystem DRIVEBASE_SUBSYSTEM = DrivebaseSubsystem.getInstance();
    public static final List<TalonSubsystemBase> SUBSYSTEMS = new ArrayList<>();
    static {
      SUBSYSTEMS.add(DRIVEBASE_SUBSYSTEM);
    }
  }

  public static final class Logging {
    public static final String LOGGING_DEPOSIT_FOLDER = ("/U/logs");
    public static final Boolean LOGGING_TURBO_MODE = (false);
    public static final Boolean LOGGING_ENABLED = (false);
    public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Odometry {
    public static final Lock CTRE_ODOMETRY_LOCK = new ReentrantLock();
    public static final Lock REV_ODOMETRY_LOCK = new ReentrantLock();
    public static final CTREOdometryThread CTRE_ODOMETRY_THREAD = CTREOdometryThread.create(CTRE_ODOMETRY_LOCK);
    public static final REVOdometryThread REV_ODOMETRY_THREAD = REVOdometryThread.create(REV_ODOMETRY_LOCK);
  }

  public static final class Pathplanner {
    public static final Map<String,PathPlannerAuto> ROUTINES = new HashMap<>();
    static {
      //TODO: AUTOMATION TEAM (DECIDE PATHS)
    }
  }

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (1);
  }

  public static final class Profiles { 

    public static final List<PilotProfile> PILOT_PROFILES = new ArrayList<PilotProfile>();
    static {
      PILOT_PROFILES.add(Pilots.Example.PROFILE);
    }

    public static final class Pilots {
      
      public static final class Example {
        public static final Integer CONTROLLER_PORT = (0);
        public static final CommandXboxController CONTROLLER = new CommandXboxController(CONTROLLER_PORT);
        public static final PilotProfile PROFILE = new PilotProfile(("John Doe"))
          .addPreference(Preferences.TRANSLATIONAL_X_INPUT, () -> CONTROLLER.getRawAxis((1)))
          .addPreference(Preferences.TRANSLATIONAL_Y_INPUT, () -> CONTROLLER.getRawAxis((0)))
          .addPreference(Preferences.ORIENTATION_INPUT, () -> CONTROLLER.getRawAxis((4)))
          .addPreference(Preferences.TRANSLATIONAL_X_DEADZONE, () -> (0.1))
          .addPreference(Preferences.TRANSLATIONAL_Y_DEADZONE, () -> (0.1))
          .addPreference(Preferences.ORIENTATION_DEADZONE, () -> (0.1))
          .addKeybinding(Keybindings.MODULE_LOCKING_TOGGLE, CONTROLLER.a())
          .addKeybinding(Keybindings.ORIENTATION_TOGGLE, CONTROLLER.b())
          .addKeybinding(Keybindings.PATHFINDING_FLIP_TOGGLE, CONTROLLER.x());
      }


    }

    public static final class Preferences {
      public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
      public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
      public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
      public static final String TRANSLATIONAL_X_DEADZONE = ("TRANSLATIONAL_X_DEADZONE");
      public static final String TRANSLATIONAL_Y_DEADZONE = ("TRANSLATIONAL_Y_DEADZONE");
      public static final String ORIENTATION_DEADZONE = ("ORIENTATION_DEADZONE");
    }

    public static final class Keybindings {
      public static final String MODULE_LOCKING_TOGGLE = ("LOCKING_ENABLED_TRIGGER");
      public static final String ORIENTATION_TOGGLE = ("ORIENTATION_TOGGLE");
      public static final String PATHFINDING_FLIP_TOGGLE = ("PATHFINDING_FLIP_TOGGLE");
    }
  }
}
