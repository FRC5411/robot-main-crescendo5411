// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.superstructure;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
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
public class SuperstructureSubsystem extends TalonSubsystemBase<Keybindings,Preferences> {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final Pair<TalonFX,TalonFX> FIRING_CONTROLLERS;
  private static final CANSparkMax INDEXER_CONTROLLER;
  private static final CANSparkMax INTAKE_CONTROLLER;
  private static final StatusSignal<Double> FIRING_VELOCITY;

  private static final CANSparkMax PIVOT_CONTROLLER;
  private static final PIDController PIVOT_CONTROLLER_PID;
  private static final SlewRateLimiter PIVOT_RATE_LIMITER;

  private static final DutyCycleEncoder PIVOT_ABSOLUTE_ENCODER;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static SuperstructureSubsystem Instance;
  private static volatile SwerveModuleState CurrentReference;
  private static volatile SuperstructureState CurrentFiringMode;
  private static volatile Operator<Keybindings,Preferences> CurrentOperator;
    // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /**
   * Cannon Subsystem Constructor
   */
  private SuperstructureSubsystem() {
    super(("Cannon Subsystem"));
  } static {
    CurrentReference = new SwerveModuleState((0d), new Rotation2d(Measurements.PIVOT_MINIMUM_ROTATION));
    CurrentFiringMode = SuperstructureState.MANUAL;
    FIRING_CONTROLLERS = new Pair<TalonFX,TalonFX>(
      new TalonFX(Ports.FIRING_CONTROLLER_LEFT_ID),
      new TalonFX(Ports.FIRING_CONTROLLER_RIGHT_ID)
    );

    FIRING_CONTROLLERS.getFirst().getConfigurator().apply(new SlotConfigs()
      .withKP(Measurements.FIRING_P_GAIN)
      .withKI(Measurements.FIRING_I_GAIN)
      .withKD(Measurements.FIRING_D_GAIN)
    );
    FIRING_CONTROLLERS.getSecond().getConfigurator().apply(new SlotConfigs()
      .withKP(Measurements.FIRING_P_GAIN)
      .withKI(Measurements.FIRING_I_GAIN)
      .withKD(Measurements.FIRING_D_GAIN)
    );

    FIRING_CONTROLLERS.getFirst().getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((45d)).withStatorCurrentLimit((40d)));
    FIRING_CONTROLLERS.getSecond().getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((45d)).withStatorCurrentLimit((40d)));
    FIRING_VELOCITY = FIRING_CONTROLLERS.getFirst().getVelocity();
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

    PIVOT_RATE_LIMITER = new SlewRateLimiter(
      Measurements.PIVOT_POSITIVE_RATE_LIMIT,
      Measurements.PIVOT_NEGATIVE_RATE_LIMIT,
      Measurements.PIVOT_INITIAL_OUTPUT
    );
    PIVOT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.PIVOT_ABSOLUTE_ENCODER_ID);
  }

  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Constants.Objects.ODOMETRY_LOCKER.lock();
    final var Robot = new Pose3d(DrivebaseSubsystem.getPose()).transformBy(VisionSubsystem.getCameraTransform(CameraIdentifier.SOURCE_CAMERA));
    final var Target = VisionSubsystem.getAprilTagPose((DrivebaseSubsystem.getRotation().getRadians() % (2) * Math.PI >= Math.PI)? (3): (7)).get();
    final var Interpolated = Measurements.PIVOT_FIRING_MAP.get(Math.hypot(
      PhotonUtils.getDistanceToPose(Robot.toPose2d(), Target.toPose2d()) / Measurements.PIVOT_MAXIMUM_RANGE_METERS,
      Measurements.SPEAKER_HEIGHT_METERS
    ));
    if(Interpolated != (null)) {
      final var Percentage = ((FIRING_VELOCITY.getValueAsDouble() / Interpolated.get((0), (0))) 
                  + (Units.rotationsToDegrees(getPivotRotation()) / Interpolated.get((1), (0)))) / 2;
      if(CurrentFiringMode == SuperstructureState.AUTO || CurrentFiringMode == SuperstructureState.SEMI) {
        CurrentReference.angle = Rotation2d.fromRadians(Units.radiansToRotations(Interpolated.get((1), (0))));
        if(CurrentFiringMode == SuperstructureState.AUTO) {
          if(Percentage < Measurements.ALLOWABLE_SHOT_PERCENTAGE) {
            set(Interpolated.get((0), (0)));
            INDEXER_CONTROLLER.set((-1d));
          }
        }
      }
      Logger.recordOutput(("Cannon/InterpolatedPercentile"), Percentage);
      Logger.recordOutput(("Cannon/InterpolatedVelocity"), Interpolated.get((0), (0)));
      Logger.recordOutput(("Cannon/InterpolatedRotation"), Interpolated.get((1), (0)));      
    } else {
      Logger.recordOutput(("Cannon/InterpolatedPercentile"), (0d));
      Logger.recordOutput(("Cannon/InterpolatedVelocity"), (0d));
      Logger.recordOutput(("Cannon/InterpolatedRotation"), (0d));
    }
    set(CurrentReference.angle);
    Logger.recordOutput(("Cannon/Reference"), CurrentReference);
    Logger.recordOutput(("Cannon/MeasuredVelocity"), FIRING_VELOCITY.getValueAsDouble());
    Logger.recordOutput(("Cannon/MeasuredRotation"), -getPivotRotation());
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

  @Override
  public synchronized void close() {
    FIRING_CONTROLLERS.getFirst().close();
    FIRING_CONTROLLERS.getSecond().close();
    PIVOT_CONTROLLER.close();
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Describes a robot's current mode of superstructure control
   */
  public enum SuperstructureState {
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
    PIVOT_CONTROLLER.set(PIVOT_CONTROLLER_PID.calculate(
      Units.radiansToRotations(getPivotRotation()),
      MathUtil.clamp(Reference.getRotations(),Measurements.PIVOT_MINIMUM_ROTATION,Measurements.PIVOT_MAXIMUM_ROTATION)));
  }

  /**
   * Mutates the current reference RPM of the cannon
   * @param Reference Desired velocity in RPM
   */
  private static synchronized void set(final Double Reference) {
    FIRING_CONTROLLERS.getFirst().setControl(new VelocityDutyCycle(-Reference));
    FIRING_CONTROLLERS.getSecond().setControl(new VelocityDutyCycle(-Reference));
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
  public static synchronized void set(final SuperstructureState Mode) {
    CurrentFiringMode = Mode;
  }

  /**
   * Mutates the cannon controller's current 'set-point' or reference state and mutates the cannon controller's current mode of operation
   * and how it should identify and calculate reference 'set-points'
   * @param Reference Cannon's new Goal or 'set-point' reference
   * @param Mode Mode of cannon control
   * @return An optimized version of the reference
   */
  public static synchronized void set(final SwerveModuleState Reference, final SuperstructureState Mode) {
    set(Mode);
    set(Reference);
  }

  @Override
  public void configure(final Operator<Keybindings, Preferences> Profile) {
    CurrentOperator = Profile;
    with(() -> {
      CurrentOperator.getKeybinding(Keybindings.OUTTAKE_TOGGLE)
        .onTrue(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((-1d));
            INDEXER_CONTROLLER.set((-1d));
          },
          SuperstructureSubsystem.getInstance()
        ));
      CurrentOperator.getKeybinding(Keybindings.OUTTAKE_TOGGLE)
        .onFalse(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((0d));
            INDEXER_CONTROLLER.set((0d));
          },
          SuperstructureSubsystem.getInstance()
      ));
    });
    with(() -> {
      CurrentOperator.getKeybinding(Keybindings.INTAKE_TOGGLE)
        .onTrue(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((1d));
            INDEXER_CONTROLLER.set((1d));
          },
          SuperstructureSubsystem.getInstance()
        ));
      CurrentOperator.getKeybinding(Keybindings.INTAKE_TOGGLE)
        .onFalse(new InstantCommand(
          () -> {
            INTAKE_CONTROLLER.set((0d));
            INDEXER_CONTROLLER.set((0d));
          },
          SuperstructureSubsystem.getInstance()
        ));
    });
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
   * @return Pivot rotational reading in radians
   */
  private static Double getPivotRotation() {
    return Units.rotationsToRadians(-(PIVOT_ABSOLUTE_ENCODER.getAbsolutePosition() - Measurements.ABSOLUTE_ENCODER_OFFSET));
  }

  @Override
  public Operator<Keybindings, Preferences> getOperator() {
    return CurrentOperator;
  }
}
