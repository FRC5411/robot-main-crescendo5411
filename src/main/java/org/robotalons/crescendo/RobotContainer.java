// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.robotalons.crescendo.Constants.Pathplanner;
import org.robotalons.crescendo.Constants.Profiles;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.subsystems.SubsystemManager;
import org.robotalons.lib.utilities.PilotProfile;

import java.util.ArrayList;
import java.util.List;
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
  public static final List<LoggedDashboardChooser<PilotProfile>> PilotSelectors;
  public static final LoggedDashboardChooser<Command> AutonomousSelector;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static RobotContainer Instance = (null);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  
  private RobotContainer() {} static {
    PilotSelectors = new ArrayList<>();
    AutonomousSelector = new LoggedDashboardChooser<>(("Autonomous Selector"), AutoBuilder.buildAutoChooser());
    Pathplanner.ROUTINES.forEach((Name, Routine) -> AutonomousSelector.addOption(Name, Routine));
    SubsystemManager.getSubsystems().forEach((Subsystem) -> {
      final var Selector = new SendableChooser<PilotProfile>();
      final var Iterator = Profiles.PILOT_PROFILES.iterator();
      final var Initial = Iterator.next();
      Iterator.forEachRemaining((Profile) -> Selector.addOption(Profile.getName(), Profile));
      Selector.setDefaultOption(Initial.getName(), Initial);
      Selector.onChange(Subsystem::configure);
      Subsystem.configure(Initial);
      PilotSelectors.add(new LoggedDashboardChooser<PilotProfile>(Subsystem.getName() + " Pilot Selector", Selector));
    });
    @SuppressWarnings("resource")
    final var INDEXER = new CANSparkMax((22), MotorType.kBrushless);
    INDEXER.setIdleMode(IdleMode.kCoast);
    @SuppressWarnings("resource")
    final var LEFT_OUT = new CANSparkMax((31), MotorType.kBrushless);
    LEFT_OUT.setIdleMode(IdleMode.kCoast);
    @SuppressWarnings("resource")
    final var RIGHT_OUT = new CANSparkMax((32), MotorType.kBrushless);
    RIGHT_OUT.setIdleMode(IdleMode.kCoast);
    @SuppressWarnings("resource")
    final var INTAKE = new CANSparkMax((21), MotorType.kBrushless);
    INDEXER.setIdleMode(IdleMode.kCoast);

    RIGHT_OUT.setInverted((true));
    
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.SHOOTER_TOGGLE).onTrue(new InstantCommand(() -> {
      LEFT_OUT.set(0.75d);
      RIGHT_OUT.set(0.75d);
    }));
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.SHOOTER_TOGGLE).onFalse(new InstantCommand(() -> {
      LEFT_OUT.set(0d);
      RIGHT_OUT.set(0d);
    }));
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.INTAKE_TOGGLE).onTrue(new InstantCommand(() -> {
      INTAKE.set(1d);
      INDEXER.set(1d);
    }));
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.INTAKE_TOGGLE).onFalse(new InstantCommand(() -> {
      INTAKE.set(0d);
      INDEXER.set(0d);
    }));
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.OUTTAKE_TOGGLE).onTrue(new InstantCommand(() -> {
      INTAKE.set(-1d);
      INDEXER.set(-1d);
    }));
    Profiles.PILOT_PROFILES.get(0).getKeybinding(Keybindings.OUTTAKE_TOGGLE).onFalse(new InstantCommand(() -> {
      INTAKE.set(0d);
      INDEXER.set(0d);
    }));
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