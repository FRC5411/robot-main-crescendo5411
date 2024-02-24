// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.subsystems.climb.ClimbSubsystem;
import org.robotalons.crescendo.subsystems.climb.Constants.Measurements;
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
  private static CommandXboxController operatorController;
  private static ClimbSubsystem climbSS = new ClimbSubsystem();
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
    CommandSelector = new LoggedDashboardChooser<>(("Autonomous Command Selector"), AutoBuilder.buildAutoChooser());
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
    
    // Double Arm Right Movement // 
    operatorController.rightTrigger()
    .onTrue(new InstantCommand(() -> climbSS.set(Measurements.CONTROLLER_ARM_SPEED)))
    .onFalse(new InstantCommand(() -> climbSS.set(0.0)));

    // Double Arm Left Movement // 
    operatorController.leftTrigger()
    .onTrue(new InstantCommand(() -> climbSS.set(-Measurements.CONTROLLER_ARM_SPEED)))
    .onFalse(new InstantCommand(() -> climbSS.set(0.0)));

    // PID setpoint // 
    operatorController.a()
    .onTrue(new InstantCommand(() -> climbSS.pidSet(100d)))
    .onFalse(new InstantCommand(() -> climbSS.set(0d)));

  
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
