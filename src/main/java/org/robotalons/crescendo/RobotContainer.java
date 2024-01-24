// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
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
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static final LoggedDashboardChooser<Command> CommandSelector;
  public static final LoggedDashboardChooser<PilotProfile> DrivebasePilotSelector;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {} static {
    CommandSelector = new LoggedDashboardChooser<>(("Autonomous Command Selector"), AutoBuilder.buildAutoChooser());
    final var DrivebasePilotChooser = new SendableChooser<PilotProfile>();
    final var PilotIterator = Profiles.PILOT_PROFILES.iterator();
    final var PilotInitial = PilotIterator.next();
    PilotIterator.forEachRemaining((Profile) -> DrivebasePilotChooser.addOption(Profile.getName(), Profile));
    DrivebasePilotChooser.setDefaultOption(PilotInitial.getName(), PilotInitial);
    DrivebasePilotChooser.onChange(DrivebaseSubsystem::configure);
    DrivebaseSubsystem.configure(PilotInitial);
    DrivebasePilotSelector = new LoggedDashboardChooser<>(("Drivebase Pilot Selector"), DrivebasePilotChooser);
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