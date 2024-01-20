// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.KeybindingNames;
import org.robotalons.crescendo.Constants.Profiles.PreferenceNames;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem.OrientationMode;
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
    final var SendableDrivebasePilotChooser = new SendableChooser<PilotProfile>();
    Profiles.PILOT_PROFILES.forEach((Profile) -> {
      SendableDrivebasePilotChooser.addOption(Profile.getName(), Profile);
    });
    SendableDrivebasePilotChooser.setDefaultOption(Profiles.PILOT_PROFILES.get((0)).getName(), Profiles.PILOT_PROFILES.get((0)));
    DrivebasePilotSelector = new LoggedDashboardChooser<>(("Drivebase Pilot Selector"), SendableDrivebasePilotChooser);
    try {
      var Field = DrivebasePilotSelector.getClass().getDeclaredField(("selectedValue"));
      Field.setAccessible((true));
      try {
        Field.set(DrivebasePilotSelector,Profiles.PILOT_PROFILES.get((0)).getName());
      } catch (final IllegalAccessException Exception) {}
    } catch (final NoSuchFieldException Exception) {}
    configureDefaultCommands();
    configurePilotKeybinds();
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Applies squared inputs to a given input, while retaining the sign
   * @param Input Any Real Number
   * @return Input Squared, with the same sign of the original
   */
  private static Double applyInputSquare(final Double Input) {
    return Math.copySign(Input * Input, Input);
  }

  /**
   * Configures subsystem default commands
   */
  public static void configureDefaultCommands() {
    Constants.Subsystems.DRIVEBASE_SUBSYSTEM.setDefaultCommand(
      new InstantCommand(() ->
      DrivebaseSubsystem.set(
        new Translation2d(
          applyInputSquare(MathUtil.applyDeadband(-(Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.TRANSLATIONAL_X_INPUT),
        (Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE))),
          applyInputSquare(MathUtil.applyDeadband(-(Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT),
        (Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE)))),
        new Rotation2d(
          applyInputSquare(MathUtil.applyDeadband(-(Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.ORIENTATION_INPUT),
        (Double) DrivebasePilotSelector.get().getPreference(PreferenceNames.ORIENTATION_DEADZONE)))),
        OrientationMode.ROBOT_ORIENTED), 
        DrivebaseSubsystem.getInstance()
    ));
  }

  /**
   * Configures the bindings, and preferences for each subsystem driver
   */
  private static void configurePilotKeybinds() {
    synchronized (DrivebasePilotSelector) {
      DrivebasePilotSelector.get().getKeybinding(KeybindingNames.ORIENTATION_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::toggleOrientationType, DrivebaseSubsystem.getInstance()));
      DrivebasePilotSelector.get().getKeybinding(KeybindingNames.MODULE_LOCKING_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::toggleModuleLocking, DrivebaseSubsystem.getInstance()));
      DrivebasePilotSelector.get().getKeybinding(KeybindingNames.PATHFINDING_FLIP_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::togglePathFlipped, DrivebaseSubsystem.getInstance()));
    }
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