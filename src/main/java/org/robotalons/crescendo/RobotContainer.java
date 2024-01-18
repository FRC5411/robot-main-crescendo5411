// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.subsystems.ClimbArmSubsystem;
import org.robotalons.crescendo.subsystems.ClimbCommand;
// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>RobotContainer</h1>
 *
 * <p>Utility class which defines all modes of robot's event-cycle throughout it's lifetime.
 */
public final class RobotContainer {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final LoggedDashboardChooser<Command> CommandSelector;
  private static CommandXboxController driveController;
  private static CommandXboxController operatorController;
  private static ClimbArmSubsystem climbSS = new ClimbArmSubsystem(0, 0);
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
    CommandSelector = new LoggedDashboardChooser<>(("Autonomous Command Selector"), AutoBuilder.buildAutoChooser());
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    configureDefaultCommands();
    configurePilotKeybinds();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Configures subsystem default commands
   */
  public static void configureDefaultCommands() {

  }

  /**
   * Configures the bindings, and preferences for each subsystem driver
   */
  private static void configurePilotKeybinds() {
    operatorController.povUp()
    .onTrue(new InstantCommand(() -> new ClimbCommand(Constants.highAngle, climbSS)));
    operatorController.povRight()
    .onTrue(new InstantCommand(() -> new ClimbCommand(Constants.idleAngle, climbSS)));
    operatorController.povDown()
    .onTrue(new InstantCommand(() -> new ClimbCommand(Constants.lowAngle, climbSS)));

  }  
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
