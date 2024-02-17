// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.cannon;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.subsystems.cannon.Constants.Measurements;
import org.robotalons.crescendo.subsystems.cannon.Constants.Ports;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.trajectory.TrajectoryManager;
import org.robotalons.lib.utilities.PilotProfile;


// ------------------------------------------------------------[Cannon Subsystem]-----------------------------------------------------------//
/**
 *
 *
 * <h1>CannonSubsystem</h1>
 *
 * <p>Utility class which controls the firing of objects to a given target, based on the current angle in radians, distance to target, and 
 * robot drivebase states.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class CannonSubsystem extends TalonSubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  public static final Pair<CANSparkMax,CANSparkMax> FIRING_CONTROLLERS;
  public static final PIDController FIRING_CONTROLLER_PID;
  public static final RelativeEncoder FIRING_ENCODER;

  public static final CANSparkMax PIVOT_CONTROLLER;
  public static final PIDController PIVOT_CONTROLLER_PID;
  public static final RelativeEncoder PIVOT_ENCODER;

  private static final DutyCycleEncoder PIVOT_ABSOLUTE_ENCODER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static CannonSubsystem Instance;
  private static SwerveModuleState CurrentReference;
  private static FiringMode CurrentMode;  
  private static PilotProfile CurrentPilot;
    // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private CannonSubsystem() {
    super(("Cannon Subsystem"));
  } static {
    CurrentReference = new SwerveModuleState();
    CurrentMode = FiringMode.MANUAL;

    FIRING_CONTROLLERS = new Pair<CANSparkMax,CANSparkMax>(
      new CANSparkMax(Ports.FIRING_CONTROLLER_LEFT_ID, MotorType.kBrushless), 
      new CANSparkMax(Ports.FIRING_CONTROLLER_RIGHT_ID, MotorType.kBrushless));
    FIRING_CONTROLLERS.getFirst().setSmartCurrentLimit((50));
    FIRING_CONTROLLERS.getSecond().setSmartCurrentLimit((50));
    FIRING_ENCODER = FIRING_CONTROLLERS.getFirst().getEncoder();
    FIRING_CONTROLLER_PID = new PIDController(
      Measurements.FIRING_P_GAIN, 
      Measurements.FIRING_I_GAIN, 
      Measurements.FIRING_D_GAIN);
    FIRING_CONTROLLERS.getSecond().setInverted((true));
    FIRING_CONTROLLERS.getSecond().follow(FIRING_CONTROLLERS.getFirst());

    PIVOT_CONTROLLER = new CANSparkMax(Ports.PIVOT_CONTROLLER_ID, MotorType.kBrushless);
    PIVOT_CONTROLLER.setSmartCurrentLimit((40));
    PIVOT_CONTROLLER_PID = new PIDController(
      Measurements.PIVOT_P_GAIN, 
      Measurements.PIVOT_I_GAIN, 
      Measurements.PIVOT_D_GAIN);
    PIVOT_ENCODER = PIVOT_CONTROLLER.getEncoder();
    PIVOT_CONTROLLER_PID.enableContinuousInput((-0), (Math.PI / 2));

    PIVOT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.PIVOT_ABSOLUTE_ENCODER_ID);

    TrajectoryManager.getInstance();
    
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    switch (CurrentMode) {
      case MANUAL:
        break;
      case SEMI:
        break;
      case AUTO:
        break;
    }
    Constants.Objects.ODOMETRY_LOCKER.unlock();
  }


  /**
   * Fires the shooter at the best possible target on the field
   */
  public synchronized static void fire() {
    
  }
  

  /** 
   * Closes this instance and all held resources immediately 
   */
  public synchronized void close() {
    
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes a robot's current mode of firing control
   */
  public enum FiringMode {
    MANUAL,    
    SEMI,
    AUTO,
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  /**
   * Mutates the cannon controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Reference Cannon's new Goal or 'set-point' reference
   */
  public static synchronized void set(final SwerveModuleState Reference) {
    CurrentReference = Reference;
  }

  /**
   * Mutates the Cannon controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of cannon control
   */
  public static synchronized void set(final FiringMode Mode) {
    CurrentMode = Mode;
  }

  /**
   * Mutates the cannon controller's current 'set-point' or reference state and mutates the cannon controller's current mode of operation
   * and how it should identify and calculate reference 'set-points'
   * @param Reference Cannon's new Goal or 'set-point' reference
   * @param Mode Mode of cannon control
   * @return An optimized version of the reference
   */
  public static synchronized void set(final SwerveModuleState Reference, final FiringMode Mode) {
    set(Mode);
    set(Reference);
  }


  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized CannonSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new CannonSubsystem();
      return Instance;
  }

  @Override
  public void configure(final PilotProfile Profile) {
    CurrentPilot = Profile;
    try {
      CurrentPilot.getKeybinding(Keybindings.SHOOTER_TOGGLE)
        .onTrue(new InstantCommand(
          CannonSubsystem::fire,
          CannonSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
  }


  @Override
  public PilotProfile getPilot() {
    return CurrentPilot;
  }
}
