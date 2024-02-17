// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import org.robotalons.crescendo.Robot.RobotType;
import org.robotalons.crescendo.subsystems.SubsystemManager;
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
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Checks if the robot is deployed with a valid mode
     * @param Options Additional options applied via the command line
     */
  public static void main(String... Options) {
    if (Subsystems.TYPE == RobotType.SIMULATION) {
      System.err.println("CANNOT DEPLOY, INVALID TYPE: " + Subsystems.TYPE.toString());
      System.exit((1));
    }
  }
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Subsystems {
    public static final SubsystemManager MANAGER = SubsystemManager.getInstance();
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
    public static final RobotType TYPE = 
      (Logging.REPLAY_FROM_LOG)?
       (RobotType.REPLAY):
      ((IS_REAL_ROBOT)? 
        (RobotType.CONCRETE):
        (RobotType.SIMULATION)
      );
  }

  public static final class Logging {
    public static final Map<RobotType,String> LOGGING_DEPOSIT = Map.of(
      RobotType.CONCRETE, ("/media/sda1/")
    );
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
          .addPreference(Preferences.TRANSLATIONAL_X_INPUT, () -> -CONTROLLER.getRawAxis((1)))
          .addPreference(Preferences.TRANSLATIONAL_Y_INPUT, () -> -CONTROLLER.getRawAxis((0)))
          .addPreference(Preferences.ORIENTATION_INPUT, () -> CONTROLLER.getRawAxis((4)))
          .addPreference(Preferences.SQUARED_INPUT, () -> (true))
          .addPreference(Preferences.TRANSLATIONAL_X_DEADZONE, () -> (0.2))
          .addPreference(Preferences.TRANSLATIONAL_Y_DEADZONE, () -> (0.2))
          .addPreference(Preferences.ORIENTATION_DEADZONE, () -> (0.2))
          .addKeybinding(Keybindings.ORIENTATION_TOGGLE, CONTROLLER.povCenter())
          .addKeybinding(Keybindings.INTAKE_TOGGLE, CONTROLLER.leftBumper())
          .addKeybinding(Keybindings.SHOOTER_TOGGLE, CONTROLLER.b())
          .addKeybinding(Keybindings.OUTTAKE_TOGGLE, CONTROLLER.rightBumper());
      }
    }

    public static final class Preferences {
      public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
      public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
      public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
      public static final String SQUARED_INPUT = ("SQUARED_INPUT");
      public static final String TRANSLATIONAL_X_DEADZONE = ("TRANSLATIONAL_X_DEADZONE");
      public static final String TRANSLATIONAL_Y_DEADZONE = ("TRANSLATIONAL_Y_DEADZONE");
      public static final String ORIENTATION_DEADZONE = ("ORIENTATION_DEADZONE");
    }

    public static final class Keybindings {
      public static final String MODULE_LOCKING_TOGGLE = ("LOCKING_ENABLED_TRIGGER");
      public static final String ORIENTATION_TOGGLE = ("ORIENTATION_TOGGLE");
      public static final String INTAKE_TOGGLE = ("INTAKE_TOGGLE");
      public static final String SHOOTER_TOGGLE = ("SHOOTER_TOGGLE");
      public static final String OUTTAKE_TOGGLE = ("OUTTAKE_TOGGLE");
    }
  }
}
