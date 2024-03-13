// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.robotalons.crescendo.Robot.RobotType;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.crescendo.subsystems.climb.ClimbSubsystem;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.SuperstructureSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;
import org.robotalons.lib.utilities.Operator;

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

  public static final class Ports {
    public static final Integer POWER_DISTRIBUTION_HUB = (1);
  }

  public static final class Profiles { 

    public static final List<Operator<Keybindings, Preferences>> OPERATORS = new ArrayList<>();
    public static final Map<TalonSubsystemBase<Keybindings,Preferences>,Operator<Keybindings, Preferences>> DEFAULT = new HashMap<>();
    static {
      OPERATORS.add(Operators.Primary.PROFILE);
      OPERATORS.add(Operators.Secondary.PROFILE);

      DEFAULT.put(ClimbSubsystem.getInstance(), Operators.Secondary.PROFILE);
      DEFAULT.put(VisionSubsystem.getInstance(), Operators.Secondary.PROFILE);
      DEFAULT.put(DrivebaseSubsystem.getInstance(), Operators.Primary.PROFILE);
      DEFAULT.put(SuperstructureSubsystem.getInstance(), Operators.Secondary.PROFILE);
    }

    public static final class Operators {
      
      public static final class Primary {
        private static final String NAME = ("ARMAAN K.");
        private static final Integer INPUT_PORT = (0);
        private static final CommandXboxController INPUT_METHOD = new CommandXboxController(INPUT_PORT);
        public static final Operator<Keybindings,Preferences> PROFILE = new Operator<Keybindings,Preferences>(NAME)
          .add(Preferences.TRANSLATION_X_INPUT, () -> -INPUT_METHOD.getRawAxis((1)))
          .add(Preferences.TRANSLATION_Y_INPUT, () -> -INPUT_METHOD.getRawAxis((0)))
          .add(Preferences.ORIENTATION_T_INPUT, () -> INPUT_METHOD.getRawAxis((4)))
          .add(Preferences.SQUARED_INPUT, () -> (true))
          .add(Preferences.TRANSLATIONAL_X_DEADZONE, () -> (2e-2))
          .add(Preferences.TRANSLATIONAL_Y_DEADZONE, () -> (2e-2))
          .add(Preferences.ORIENTATION_DEADZONE, () -> (2e-2))
          .add(Keybindings.ORIENTATION_TOGGLE, INPUT_METHOD.a())
          .add(Keybindings.PRECISION_TOGGLE, INPUT_METHOD.y())
          .add(Keybindings.ALIGNMENT_SPEAKER, INPUT_METHOD.x());
      }

      public static final class Secondary {
        private static final String NAME = ("ALEX P.");
        private static final Integer INPUT_PORT = (1);
        private static final CommandXboxController INPUT_METHOD = new CommandXboxController(INPUT_PORT);
        public static final Operator<Keybindings,Preferences> PROFILE = new Operator<Keybindings,Preferences>(NAME)
          .add(Keybindings.CANNON_PIVOT_PODIUMLINE, INPUT_METHOD.a())
          .add(Keybindings.CANNON_PIVOT_SUBWOOFER, INPUT_METHOD.b())
          .add(Keybindings.CANNON_PIVOT_STARTING_LINE, INPUT_METHOD.x())
          .add(Keybindings.CANNON_PIVOT_WINGLINE, INPUT_METHOD.y())
          .add(Keybindings.CANNON_TOGGLE, Operators.Primary.INPUT_METHOD.leftBumper())
          .add(Keybindings.CLIMB_ROTATE_BACKWARD, INPUT_METHOD.povDown())
          .add(Keybindings.CLIMB_ROTATE_FORWARD, INPUT_METHOD.povUp())
          .add(Keybindings.INTAKE_TOGGLE, Operators.Primary.INPUT_METHOD.leftTrigger())
          .add(Keybindings.OUTTAKE_TOGGLE, Operators.Primary.INPUT_METHOD.rightTrigger())
          .add(Keybindings.SHOOT_TOGGLE, INPUT_METHOD.rightBumper());
      }
    }

    public enum Preferences {
      TRANSLATION_X_INPUT,
      TRANSLATION_Y_INPUT,
      ORIENTATION_T_INPUT,
      SQUARED_INPUT,
      TRANSLATIONAL_X_DEADZONE,
      TRANSLATIONAL_Y_DEADZONE,
      ORIENTATION_DEADZONE
    }

    public enum Keybindings {
      ORIENTATION_TOGGLE,
      CLIMB_ROTATE_FORWARD,
      CLIMB_ROTATE_BACKWARD,
      PRECISION_TOGGLE,
      INTAKE_TOGGLE,
      OUTTAKE_TOGGLE,
      CANNON_TOGGLE,
      CANNON_PIVOT_WINGLINE,
      CANNON_PIVOT_SUBWOOFER,
      CANNON_PIVOT_STARTING_LINE,
      CANNON_PIVOT_PODIUMLINE,
      CANNON_PIVOT_MAXIMUM,
      ALIGNMENT_SPEAKER,
      ALIGNMENT_AMP,
      ALIGNMENT_OBJECT,
      ALIGNMENT_NEAREST,
      SHOOT_TOGGLE
    }
  }
}
