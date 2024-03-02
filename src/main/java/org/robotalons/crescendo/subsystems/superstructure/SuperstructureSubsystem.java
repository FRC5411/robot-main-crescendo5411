// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.superstructure;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.Constants.Measurements;
import org.robotalons.crescendo.subsystems.superstructure.Constants.Ports;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem.CameraIdentifier;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.motion.trajectory.solving.TrajectoryObject;
import org.robotalons.lib.utilities.Operator;
// --------------------------------------------------------[Superstructure Subsystem]--------------------------------------------------------//
/**
 *
 *
 * <h1>SuperstructureSubsystem</h1>
 *
 * <p>Utility class which controls the firing of objects to a given target, based on the current angle in radians, distance to target, and 
 * robot drivebase states.<p>
 * 
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public class SuperstructureSubsystem extends TalonSubsystemBase {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final Pair<TalonFX,TalonFX> FIRING_CONTROLLERS;
  private static final CANSparkMax INDEXER_CONTROLLER;
  private static final CANSparkMax INTAKE_CONTROLLER;
  private static final PIDController FIRING_CONTROLLER_PID;
  private static final StatusSignal<Double> FIRING_VELOCITY;

  //private static final DigitalOutput INDEXER_SENSOR;
 
  private static final CANSparkMax PIVOT_CONTROLLER;
  private static final PIDController PIVOT_CONTROLLER_PID;

  private static final DutyCycleEncoder PIVOT_ABSOLUTE_ENCODER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static SuperstructureSubsystem Instance;
  private static SwerveModuleState CurrentReference;
  private static FiringMode CurrentFiringMode;  
  private static Operator CurrentPilot;
    // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /** 
   * Cannon Subsystem Constructor 
   */
  private SuperstructureSubsystem() { 
    super(("Cannon Subsystem"));
  } static {
    CurrentReference = new SwerveModuleState((0d), new Rotation2d(Measurements.MID_HOLD_ROTATION));
    CurrentFiringMode = FiringMode.MANUAL;
    FIRING_CONTROLLERS = new Pair<TalonFX,TalonFX>(
      new TalonFX(Ports.FIRING_CONTROLLER_LEFT_ID), 
      new TalonFX(Ports.FIRING_CONTROLLER_RIGHT_ID)
    );
    FIRING_CONTROLLERS.getFirst().getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((45d)).withStatorCurrentLimit((40d)));
    FIRING_CONTROLLERS.getSecond().getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((45d)).withStatorCurrentLimit((40d)));
    FIRING_VELOCITY = FIRING_CONTROLLERS.getFirst().getVelocity();
    FIRING_CONTROLLER_PID = new PIDController(
      Measurements.FIRING_P_GAIN, 
      Measurements.FIRING_I_GAIN, 
      Measurements.FIRING_D_GAIN);
    FIRING_CONTROLLERS.getSecond().setInverted((false));

    INDEXER_CONTROLLER = new CANSparkMax(Ports.INDEXER_CONTROLLER_ID, MotorType.kBrushless);
    INDEXER_CONTROLLER.setSmartCurrentLimit((20));
    INDEXER_CONTROLLER.setSecondaryCurrentLimit((30));
    INDEXER_CONTROLLER.setIdleMode(IdleMode.kBrake);
    INDEXER_CONTROLLER.setInverted((false));

    INTAKE_CONTROLLER = new CANSparkMax(Ports.INTAKE_CONTROLLER_ID, MotorType.kBrushless); 
    INTAKE_CONTROLLER.setSmartCurrentLimit((20));
    INTAKE_CONTROLLER.setSecondaryCurrentLimit((30));
    INTAKE_CONTROLLER.setInverted((true));

    PIVOT_CONTROLLER = new CANSparkMax(Ports.PIVOT_CONTROLLER_ID, MotorType.kBrushless);
    PIVOT_CONTROLLER.setSmartCurrentLimit((40));
    PIVOT_CONTROLLER_PID = new PIDController(
      Measurements.PIVOT_P_GAIN, 
      Measurements.PIVOT_I_GAIN, 
      Measurements.PIVOT_D_GAIN);
    PIVOT_CONTROLLER.setInverted(Measurements.PIVOT_INVERTED);
    PIVOT_CONTROLLER_PID.enableContinuousInput(Measurements.PIVOT_MINIMUM_ROTATION, Measurements.PIVOT_MAXIMUM_ROTATION);

    PIVOT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.PIVOT_ABSOLUTE_ENCODER_ID);    
  }
  
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    //TODO: Move into Subsystem Manager
    switch(CurrentFiringMode) {
      case AUTO, SEMI:
        final var Camera = VisionSubsystem.getCameraTransform(CameraIdentifier.SOURCE_CAMERA);
        final var Target = VisionSubsystem.getAprilTagPose(
          (DrivebaseSubsystem.getRotation().getRadians() % (2) * Math.PI >= Math.PI)? (3): (7)).get();
        CurrentReference.angle = Rotation2d.fromRadians(Measurements.PIVOT_FIRING_MAP.interpolate(
          Measurements.PIVOT_LOWER_BOUND,
          Measurements.PIVOT_UPPER_BOUND,
          PhotonUtils.calculateDistanceToTargetMeters(
            Camera.getY(),
            Target.getY(), 
            Camera.getRotation().getY(), 
            Target.getRotation().getY()
          ) / Measurements.PIVOT_MAXIMUM_RANGE_METERS
        ).get((1), (0)));
        break;
      default:
        break;
    }
    final var AbsoluteReading = getPivotRotation();
    if(
      AbsoluteReading > CurrentReference.angle.minus(Rotation2d.fromDegrees((2.5d))).getRadians() 
                                              &&
      AbsoluteReading < CurrentReference.angle.plus(Rotation2d.fromDegrees((2.5d))).getRadians()
                                              && 
                            CurrentFiringMode == FiringMode.AUTO
    ) {
      fire();
    }    
    set(CurrentReference.angle);
    Logger.recordOutput(("Cannon/Reference"), CurrentReference);
    Logger.recordOutput(("Cannon/MeasuredVelocity"), FIRING_VELOCITY.getValueAsDouble());
    Logger.recordOutput(("Cannon/MeasuredRotation"), AbsoluteReading * 360d);
    Constants.Objects.ODOMETRY_LOCKER.unlock();

  }

  /**
   * Quickly creates a Trajectory Object within note specifications
   * @param Velocity  Initial Velocity
   * @param Rotation  Initial Rotation
   * @param Distance  How far lengthwise the object must travel
   * @param Height    How far heightwise the object must travel
   * @return Note preset with the parameters
   */
  @SuppressWarnings("unused")
  private static TrajectoryObject object(final Double Velocity, final Double Distance, final Double Height, final Rotation2d Rotation) {
    return TrajectoryObject.note(Velocity, Rotation, Measurements.CANNON_LENGTH, Distance, Height, (2000));
  }

  /**
   * Fires the shooter at the best possible target on the field
   */
  public static synchronized void fire() {
    set(-5.4d);
  }
  

  @Override
  public synchronized void close() {
    FIRING_CONTROLLERS.getFirst().close();
    FIRING_CONTROLLERS.getSecond().close();
    PIVOT_CONTROLLER.close();
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
   * Mutates the current reference rotation of the cannon as a measurement in radians
   * @param Reference Desired rotation in radians
   */
  private static synchronized void set(final Rotation2d Reference) {
    CurrentReference.angle = Reference;
    PIVOT_CONTROLLER.set(
      PIVOT_CONTROLLER_PID.calculate(getPivotRotation(),
                                    MathUtil.clamp(Reference.getRotations(),Measurements.PIVOT_MINIMUM_ROTATION,Measurements.PIVOT_MAXIMUM_ROTATION)));
  }

  /**
   * Mutates the current reference RPM of the cannon
   * @param Reference Desired velocity in ROM
   */
  private static synchronized void set(final Double Reference) {
    CurrentReference.speedMetersPerSecond = Reference;
    final var Effort = FIRING_CONTROLLER_PID.calculate(FIRING_VELOCITY.getValueAsDouble(), -Reference);
    FIRING_CONTROLLERS.getFirst().set(Effort);
    FIRING_CONTROLLERS.getSecond().set(Effort);;
  }

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
    CurrentFiringMode = Mode;
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

  public static synchronized void setRotation(final Rotation2d Reference) {
    CurrentReference.angle = Reference;
  }

  @Override
  public void configure(final Operator Profile) {
    CurrentPilot = Profile;
    try {
      CurrentPilot.getKeybinding(Keybindings.CANNON_TOGGLE)
        .onTrue(new InstantCommand(
          () -> {
            set((20.4d));
          },
          SuperstructureSubsystem.getInstance()
        ));
        CurrentPilot.getKeybinding(Keybindings.CANNON_TOGGLE)
          .onFalse(new InstantCommand(
          () -> {
            FIRING_CONTROLLERS.getFirst().set((0.375d));
            FIRING_CONTROLLERS.getSecond().set((0.375d));
          }
          ,SuperstructureSubsystem.getInstance()));
    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.CANNON_PIVOT_UP)
        .whileTrue(new InstantCommand(
          () -> {
            CurrentReference.angle = CurrentReference.angle.plus(Rotation2d.fromDegrees((1d)));
          },
          SuperstructureSubsystem.getInstance()
        ).repeatedly());
    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.CANNON_PIVOT_DOWN)
        .whileTrue(new InstantCommand(
          () -> {
            CurrentReference.angle = CurrentReference.angle.minus(Rotation2d.fromDegrees((1d)));
          },
          SuperstructureSubsystem.getInstance()
        ).repeatedly());

    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.OUTTAKE_TOGGLE)
        .onTrue(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((-1d));
            INDEXER_CONTROLLER.set((-1d));
          },
          SuperstructureSubsystem.getInstance()
        ));
        CurrentPilot.getKeybinding(Keybindings.OUTTAKE_TOGGLE)
        .onFalse(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((0d));
            INDEXER_CONTROLLER.set((0d));
          },
          SuperstructureSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
    try {
      CurrentPilot.getKeybinding(Keybindings.INTAKE_TOGGLE)
        .onTrue(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((1d));
            INDEXER_CONTROLLER.set((1d));
          },
          SuperstructureSubsystem.getInstance()
        ));
        CurrentPilot.getKeybinding(Keybindings.INTAKE_TOGGLE)
        .onFalse(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((0d));
            INDEXER_CONTROLLER.set((0d));
          },
          SuperstructureSubsystem.getInstance()
        ));
    } catch(final NullPointerException Ignored) {}
  }
  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized SuperstructureSubsystem getInstance() {
      if (java.util.Objects.isNull(Instance))
          Instance = new SuperstructureSubsystem();
      return Instance;
  }

  /**
   * Provides the current rotational reading of the pivot in rotations
   * @return Pivot rotational reading 
   */
  private static Double getPivotRotation() {
    return -(PIVOT_ABSOLUTE_ENCODER.getAbsolutePosition() - Measurements.ABSOLUTE_ENCODER_OFFSET);
  }

  @Override
  public Operator getOperator() {
    return CurrentPilot;
  }
}
