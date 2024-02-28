// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
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
  public static final CommandXboxController controller;
  public static final VisionSubsystem subsystem;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
    CommandSelector = new LoggedDashboardChooser<>(("Autonomous Command Selector"), AutoBuilder.buildAutoChooser());
    subsystem = new VisionSubsystem();
    controller = new CommandXboxController(0);
    configureDefaultCommands();
    configurePilotKeybinds();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Configures subsystem default commands
   */
  public static void configureDefaultCommands() {
    controller.a().onTrue(new InstantCommand(() -> subsystem.postSnapshot(1)));
    controller.b().onTrue(new InstantCommand(() -> subsystem.getApproximatedRobotPose()));
    controller.x().onTrue(new InstantCommand(() -> subsystem.getObjectFieldPose()));
    controller.y().onTrue(new InstantCommand(() -> subsystem.getOptimalTarget(1)));
  }

  /**
   * Configures the bindings, and preferences for each subsystem driver
   */
  private static void configurePilotKeybinds() {

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
