// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.robotalons.crescendo.Robot.RobotType;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.utilities.CTREOdometryThread;
import org.robotalons.lib.motion.utilities.REVOdometryThread;
import org.robotalons.lib.utilities.Operator;
import org.robotalons.lib.utilities.TypeVector;

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
      System.err.println("CANNOT DEPLOY, INVALID TYPE: " + Subsystems.TYPE);
      System.exit((1));
    }
  }
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//
  public static final class Subsystems {
    public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
    public static final RobotType TYPE = 
      (Logging.REPLAY_FROM_LOG)?
       (RobotType.REPLAY):
      ((IS_REAL_ROBOT)? 
        (RobotType.CONCRETE):
        (RobotType.SIMULATION)
      );
  }

  public static final class Field {
    public static final Integer DEFAULT_ALLIANCE = (1);
    public static final Double FIELD_LENGTH = Units.inchesToMeters((651.223d));
    public static final Double FIELD_WIDTH = Units.inchesToMeters((323.277d));

    public static final class Lines {
      public static final Double WING_LINE_POSITION = Units.inchesToMeters((229.201d));
      public static final Double PODIUM_LINE_POSITION = Units.inchesToMeters((126.75d));
      public static final Double STARTING_LINE_POSITION = Units.inchesToMeters((74.111d));
      public static final Double CENTER_LINE_POSITION = FIELD_LENGTH / 2d;      
    }

    public static final class Elements {
      public static final Double AMP_POSITION = Units.inchesToMeters((72.455d));

      public static final class Speaker {
        public static final Translation3d TOP_RIGHT_SPEAKER =
            new Translation3d(
                Units.inchesToMeters((18.055d)),
                Units.inchesToMeters((238.815d)),
                Units.inchesToMeters((83.091d)));

        public static final Translation3d TOP_LEFT_SPEAKER =
            new Translation3d(
                Units.inchesToMeters((18.055d)),
                Units.inchesToMeters((197.765d)),
                Units.inchesToMeters((83.091d)));

        public static final Translation3d BOTTOM_RIGHT_SPEAKER =
            new Translation3d((0d), Units.inchesToMeters((238.815d)), Units.inchesToMeters((78.324d)));
        public static final Translation3d BOTTOM_LEFT_SPEAKER =
            new Translation3d((0d), Units.inchesToMeters((197.765d)), Units.inchesToMeters((78.324d)));


        public static final Translation3d CENTER_SPEAKER_OPENING =
            BOTTOM_LEFT_SPEAKER.interpolate(TOP_RIGHT_SPEAKER, (1d/2d));
      }
    }
  }

  public static final class Logging {
    public static final Map<RobotType,String> LOGGING_DEPOSIT = Map.of(
      RobotType.CONCRETE, ("/media/sda1/"),
      RobotType.SIMULATION, ("logs/simulation")
    );
    public static final Boolean LOGGING_ENABLED = (false);
    public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Odometry {
    public static final Lock CTRE_ODOMETRY_LOCK = new ReentrantLock((true));
    public static final Lock REV_ODOMETRY_LOCK = new ReentrantLock((true));
    public static final Boolean CTRE_ODOMETRY_ENABLED = (false);
    public static final Boolean REV_ODOMETRY_ENABLED = (false);
    public static final CTREOdometryThread CTRE_ODOMETRY_THREAD = CTREOdometryThread.create(CTRE_ODOMETRY_LOCK);
    public static final REVOdometryThread REV_ODOMETRY_THREAD = REVOdometryThread.create(REV_ODOMETRY_LOCK);

    public static final Double BLUE_ALLIANCE_HORIZONTAL_LOCATION = (0.59d);
    public static final Double RED_ALLIANCE_HORIZONTAL_LOCATION = (15.89d);
    public static final List<Double> ALLIANCE_VERTICAL_LOCATIONS = List.of((2d), (4d), (7.2d));

    static {
      CTRE_ODOMETRY_THREAD.setEnabled(CTRE_ODOMETRY_ENABLED);
      REV_ODOMETRY_THREAD.setEnabled(REV_ODOMETRY_ENABLED);
    }
  }

  public static final class Profiles { 

    public static final List<Operator<Keybindings, Preferences>> OPERATORS = new ArrayList<>();
    public static final Map<TalonSubsystemBase<Keybindings,Preferences,?>, TypeVector<Operator<Keybindings, Preferences>,?>> DEFAULT = new HashMap<>();
    static {
      OPERATORS.add(Operators.Primary.PROFILE);
      OPERATORS.add(Operators.Secondary.PROFILE);
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
          .add(Preferences.TRANSLATIONAL_X_DEADZONE, () -> (2e-1))
          .add(Preferences.TRANSLATIONAL_Y_DEADZONE, () -> (2e-1))
          .add(Preferences.ORIENTATION_DEADZONE, () -> (2e-1))
          .add(Keybindings.ORIENTATION_TOGGLE, INPUT_METHOD.a())
          .add(Keybindings.RESET_GYROSCOPE, INPUT_METHOD.y())
          .add(Keybindings.ALIGNMENT_SPEAKER, INPUT_METHOD.x())
          .add(Keybindings.INTAKE_TOGGLE, INPUT_METHOD.leftTrigger())
          .add(Keybindings.OUTTAKE_TOGGLE, INPUT_METHOD.rightTrigger());          
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
          .add(Keybindings.CLIMB_ROTATE_FORWARD, INPUT_METHOD.povUp());
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
      RESET_GYROSCOPE,
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
      ALIGNMENT_NEAREST
    }
  }
}
