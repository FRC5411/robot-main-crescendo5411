// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.superstructure;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;
import org.robotalons.crescendo.subsystems.superstructure.Constants.Measurements;
import org.robotalons.crescendo.subsystems.superstructure.Constants.Objects;
import org.robotalons.crescendo.subsystems.superstructure.Constants.Ports;
import org.robotalons.crescendo.subsystems.vision.VisionSubsystem;
import org.robotalons.lib.utilities.MathUtilities;
import org.robotalons.lib.utilities.Operator;
import org.robotalons.lib.utilities.TalonSubsystemBase;
import org.robotalons.lib.utilities.TypeVector;
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
public class SuperstructureSubsystem extends TalonSubsystemBase<Keybindings,Preferences, N2> {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final TalonFX UPPER_ROLLER_CONTROLLER;
  private static final TalonFX LOWER_ROLLER_CONTROLLER;
  private static final StatusSignal<Double> ROLLER_VELOCITY; 

  private static final CANSparkMax PIVOT_CONTROLLER;
  private static final RelativeEncoder PIVOT_RELATIVE_ENCODER;
  private static final DutyCycleEncoder PIVOT_ABSOLUTE_ENCODER;
  private static final ProfiledPIDController PIVOT_PID_FEEDBACK;
  private static final ArmFeedforward PIVOT_FF_FEEDBACK;

  private static final CANSparkMax INDEXER_CONTROLLER;
  private static final DigitalInput INDEXER_IR_SENSOR;
  private static final CANSparkMax INTAKE_CONTROLLER;
  private static final DigitalInput INTAKE_IR_SENSOR;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static Operator<Keybindings, Preferences> PrimaryOperator;
  private static Operator<Keybindings, Preferences> SecondaryOperator;
  private static SuperstructureSubsystem Instance;
  private static Matrix<N2,N1> Interpolation;
  private static SwerveModuleState Reference;
  private static SuperstructureState State;
  // ------------------------------------------------------------[Constructors]----------------------------------------------------------- //
  /**
   * Superstructure Subsystem Constructor
   */
  private SuperstructureSubsystem() {
    super(("Superstructure-Subsystem"), Nat.N2());
  } static {
    UPPER_ROLLER_CONTROLLER = new TalonFX(Ports.FIRING_CONTROLLER_RIGHT_ID, Ports.CAN_BUS_NAME);
    LOWER_ROLLER_CONTROLLER = new TalonFX(Ports.FIRING_CONTROLLER_LEFT_ID, Ports.CAN_BUS_NAME);
    ROLLER_VELOCITY = UPPER_ROLLER_CONTROLLER.getVelocity();

    UPPER_ROLLER_CONTROLLER.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
    LOWER_ROLLER_CONTROLLER.getConfigurator().apply(new TalonFXConfiguration().CurrentLimits.withSupplyCurrentLimit((25d)).withStatorCurrentLimit((20d)));
    UPPER_ROLLER_CONTROLLER.setInverted((true));
    LOWER_ROLLER_CONTROLLER.setInverted((true));

    UPPER_ROLLER_CONTROLLER.getConfigurator().apply(new SlotConfigs()
      .withKP(Measurements.FIRING_P_GAIN)
      .withKI(Measurements.FIRING_I_GAIN)
      .withKD(Measurements.FIRING_D_GAIN)
    );
    LOWER_ROLLER_CONTROLLER.getConfigurator().apply(new SlotConfigs()
      .withKP(Measurements.FIRING_P_GAIN)
      .withKI(Measurements.FIRING_I_GAIN)
      .withKD(Measurements.FIRING_D_GAIN)
    );

    ROLLER_VELOCITY.setUpdateFrequency((50d));
    UPPER_ROLLER_CONTROLLER.optimizeBusUtilization();
    LOWER_ROLLER_CONTROLLER.optimizeBusUtilization();

    PIVOT_CONTROLLER = new CANSparkMax(Ports.PIVOT_CONTROLLER_ID, MotorType.kBrushless);
    PIVOT_CONTROLLER.setSmartCurrentLimit((40));
    PIVOT_CONTROLLER.setSecondaryCurrentLimit((50d));
    PIVOT_CONTROLLER.setIdleMode(IdleMode.kBrake);
    PIVOT_CONTROLLER.setInverted((false));
    PIVOT_CONTROLLER.setInverted(Measurements.PIVOT_INVERTED);
    PIVOT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.PIVOT_ABSOLUTE_ENCODER_ID);
    PIVOT_RELATIVE_ENCODER = PIVOT_CONTROLLER.getEncoder();
    PIVOT_PID_FEEDBACK = new ProfiledPIDController(
      Measurements.PIVOT_P_GAIN,
      Measurements.PIVOT_I_GAIN, 
      Measurements.PIVOT_D_GAIN, 
      new Constraints(
        Measurements.PIVOT_MAX_VELOCITY, 
        Measurements.PIVOT_MAX_ACCELERATION));
    PIVOT_FF_FEEDBACK = new ArmFeedforward(
      Measurements.PIVOT_KS_GAIN, 
      Measurements.PIVOT_KG_GAIN, 
      Measurements.PIVOT_KV_GAIN);

    INDEXER_CONTROLLER = new CANSparkMax(Ports.INDEXER_CONTROLLER_ID, MotorType.kBrushless);
    INDEXER_CONTROLLER.setSmartCurrentLimit((20));
    INDEXER_CONTROLLER.setSecondaryCurrentLimit((30d));
    INDEXER_CONTROLLER.setIdleMode(IdleMode.kBrake);
    INDEXER_CONTROLLER.setInverted((false));

    INDEXER_IR_SENSOR = new DigitalInput(Ports.INDEXER_IR_SENSOR_ID);

    INTAKE_CONTROLLER = new CANSparkMax(Ports.INTAKE_CONTROLLER_ID, MotorType.kBrushless);
    INTAKE_CONTROLLER.setSmartCurrentLimit((20));
    INTAKE_CONTROLLER.setSecondaryCurrentLimit((30d));
    INDEXER_CONTROLLER.setIdleMode(IdleMode.kCoast);
    INTAKE_CONTROLLER.setInverted((true));

    INTAKE_IR_SENSOR = new DigitalInput(Ports.INTAKE_IR_SENSOR_ID);

    Reference = new SwerveModuleState(); //TODO: Add a Cosine factor
    State = SuperstructureState.MANUAL;
  }
  // ---------------------------------------------------------------[Methods]--------------------------------------------------------------- //
  @Override
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCK.lock();
    Interpolation = Measurements.PIVOT_FIRING_MAP.interpolate(
      Measurements.PIVOT_LOWER_BOUND,
      Measurements.PIVOT_UPPER_BOUND,
      Math.hypot(
        PhotonUtils.getDistanceToPose(DrivebaseSubsystem.getPose(), 
        VisionSubsystem.getAprilTagPose((DrivebaseSubsystem.getAlliance())? (7): (3)).get().toPose2d()),
        Measurements.SPEAKER_HEIGHT_METERS) 
        / Math.hypot(Measurements.PIVOT_MAXIMUM_RANGE_METERS, Measurements.SPEAKER_HEIGHT_METERS));
    if(Interpolation != (null)) {  
      final var Percentage = MathUtilities.mean(
        (1d) - Math.abs(Interpolation.get((0), (0)) - (getVelocity())), 
        (1d) - Math.abs(Interpolation.get((1), (0)) - (getRotation().getRadians())));
      switch(State) {
        case AUTO:
          Reference.angle = Rotation2d.fromRadians(Interpolation.get((1), (0)));
          if(Percentage < Measurements.ALLOWABLE_SHOT_PERCENTAGE) {
            set(Interpolation.get((0), (0)));
            INDEXER_CONTROLLER.set((-1d));
          }
          break;
        case SEMI:
          Reference.angle = Rotation2d.fromRadians(Interpolation.get((1), (0)));
          break;
        case MANUAL:
          break;
      }
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Percentile"), Percentage);
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Velocity"), Interpolation.get((0), (0)));
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Rotation"), Units.radiansToDegrees(Interpolation.get((1), (0))));      
    } else {
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Distance"), (0d)); 
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Percentile"), (0d));
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Velocity"), (0d));
      Logger.recordOutput(("Superstructure-Subsystem/Interpolated-Rotation"), (0d));
    }
    Logger.recordOutput(("Superstructure-Subsystem/Reference"), Reference);
    Logger.recordOutput(("Superstructure-Subsystem/Measured-Velocity"), ROLLER_VELOCITY.getValueAsDouble());
    Logger.recordOutput(("Superstructure-Subsystem/Measured-Rotation"), getRotation().getDegrees());
    set(Reference);
    Objects.ODOMETRY_LOCK.unlock();
  }

  @Override
  public synchronized void close() {
    UPPER_ROLLER_CONTROLLER.close();
    LOWER_ROLLER_CONTROLLER.close();

    PIVOT_CONTROLLER.close();
    PIVOT_ABSOLUTE_ENCODER.close();

    INDEXER_CONTROLLER.close();
    INTAKE_CONTROLLER.close();

    INDEXER_IR_SENSOR.close();
    INTAKE_IR_SENSOR.close();

    Instance = (null);
  }


  /**
   * Utility method for quickly adding button bindings to reach a given rotation, and reset to default
   * @param Keybinding Trigger to bind this association to
   * @param Rotation   Value of rotation to bring to pivot to
   * @param Velocity   Value of velocity to bring the firing controllers to in RPM
   */
  private void configure(final Trigger Keybinding, final Double Rotation, final Double Velocity) {
    Keybinding.onTrue(new InstantCommand(
      () -> {
        Reference.angle = Rotation2d.fromRadians(Rotation);
        set(Velocity);
      },
      SuperstructureSubsystem.getInstance()
    ));
    Keybinding.onFalse(new InstantCommand(
      () -> {
        Reference.angle = Rotation2d.fromRadians(Measurements.PIVOT_MINIMUM_ROTATION);
        UPPER_ROLLER_CONTROLLER.set((Measurements.FIRING_IDLE_PERCENT));
        LOWER_ROLLER_CONTROLLER.set((Measurements.FIRING_IDLE_PERCENT));
      },
      SuperstructureSubsystem.getInstance()
    ));
  }

  @Override
  public void configure(final TypeVector<Operator<Keybindings, Preferences>,N2> Operator) {
    PrimaryOperator = Operator.get((0));
    SecondaryOperator = Operator.get((1));
    PrimaryOperator.getOptionalKeybinding(Keybindings.CANNON_PIVOT_SUBWOOFER).ifPresent((Trigger) -> 
      configure(
        Trigger, 
        Measurements.SUBWOOFER_LINE_ROTATION, 
        Measurements.SUBWOOFER_RPM));
    PrimaryOperator.getOptionalKeybinding(Keybindings.CANNON_PIVOT_WINGLINE).ifPresent((Trigger) -> 
      configure(
        Trigger, 
        Measurements.WING_LINE_ROTATION, 
        Measurements.SUBWOOFER_RPM));
    PrimaryOperator.getOptionalKeybinding(Keybindings.CANNON_PIVOT_PODIUMLINE).ifPresent((Trigger) -> 
      configure(
        Trigger, 
        Measurements.PODIUM_LINE_ROTATION, 
        Measurements.SUBWOOFER_RPM));
    PrimaryOperator.getOptionalKeybinding(Keybindings.CANNON_PIVOT_STARTING_LINE).ifPresent((Trigger) -> 
      configure(
        Trigger, 
        Measurements.STARTING_LINE_ROTATION, 
        Measurements.SUBWOOFER_RPM));
    SecondaryOperator.getOptionalKeybinding(Keybindings.OUTTAKE_TOGGLE).ifPresent((Trigger) -> {
      Trigger.onTrue(new InstantCommand(
        () -> {
          INTAKE_CONTROLLER.set((-1d));
          INDEXER_CONTROLLER.set((-1d));
        },
        SuperstructureSubsystem.getInstance()
      ));
      Trigger.onFalse(new InstantCommand(
        () -> {
          INTAKE_CONTROLLER.set((0d));
          INDEXER_CONTROLLER.set((0d));
        },
        SuperstructureSubsystem.getInstance()
      ));
    });
    SecondaryOperator.getOptionalKeybinding(Keybindings.INTAKE_TOGGLE).ifPresent((Trigger) -> {
      Trigger.onTrue(new InstantCommand(
        () -> {
          INTAKE_CONTROLLER.set((1d));
          INDEXER_CONTROLLER.set((1d));
        },
        SuperstructureSubsystem.getInstance()
      ));
      Trigger.onFalse(new InstantCommand(
        () -> {
          INTAKE_CONTROLLER.set((0d));
          INDEXER_CONTROLLER.set((0d));
        },
        SuperstructureSubsystem.getInstance()
      ));
    });
    SecondaryOperator.getOptionalKeybinding(Keybindings.INTAKE_TOGGLE).ifPresent((Trigger) -> {
      Trigger.onTrue(new InstantCommand(
        () -> set(Measurements.FIRING_STANDARD_VELOCITY),
        SuperstructureSubsystem.getInstance()
      ));
      Trigger.onFalse(new InstantCommand(
        () -> {
          UPPER_ROLLER_CONTROLLER.set((0d));
          LOWER_ROLLER_CONTROLLER.set((0d));
        },
        SuperstructureSubsystem.getInstance()
      ));
    });
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//
  /**
   * Enum of Saved Named Commands
   */
  public enum Commands {
    PLACEHOLDER(new InstantCommand());

    private final Command NAMED_COMMAND;
    /**
     * Named Commands Constructor
     * @param Command Valid named command to register 
     */
    private Commands(final Command Command) {
      NAMED_COMMAND = Command;
      NamedCommands.registerCommand(name(), NAMED_COMMAND);
    }
  }

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
  public static synchronized void set(final Rotation2d Reference) {
    SuperstructureSubsystem.Reference.angle = Reference;
    PIVOT_PID_FEEDBACK.reset(getRotation().getDegrees());
    PIVOT_CONTROLLER.set(
      -(PIVOT_PID_FEEDBACK.calculate(
          getRotation().getDegrees(), 
          SuperstructureSubsystem.Reference.angle.getDegrees()) 
          +PIVOT_FF_FEEDBACK.calculate(getRotation().getRadians(), getVelocity())));
  }

  /**
   * Mutates the current reference RPM of the cannon
   * @param Reference Desired velocity in RPM
   */
  public static synchronized void set(final Double Reference) {
    SuperstructureSubsystem.Reference.speedMetersPerSecond = Reference;
    UPPER_ROLLER_CONTROLLER.setControl(new VelocityDutyCycle((SuperstructureSubsystem.Reference.speedMetersPerSecond / (60d))));
    LOWER_ROLLER_CONTROLLER.setControl(new VelocityDutyCycle((SuperstructureSubsystem.Reference.speedMetersPerSecond / (60d))));
  }

  /**
   * Mutates the cannon controller's current 'set-point' or reference {@link SwerveModuleState state}
   * @param Demand Cannon's new Goal or 'set-point' reference
   */
  public static synchronized void set(final SwerveModuleState Demand) {
    Reference = Demand;
  }

  /**
   * Mutates the Cannon controller's current mode of operation and how it should identify and calculate reference 'set-points'
   * @param Mode Mode of cannon control
   */
  public static synchronized void set(final SuperstructureState Mode) {
    State = Mode;
  }

  /**
   * Mutates the cannon controller's current 'set-point' or reference state and mutates the cannon controller's current mode of operation
   * and how it should identify and calculate reference 'set-points'
   * @param Reference Cannon's new Goal or 'set-point' reference
   * @param Mode Mode of cannon control
   */
  public static synchronized void set(final SwerveModuleState Reference, final SuperstructureState Mode) {
    set(Mode);
    set(Reference);
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

  @Override
  @SuppressWarnings("unchecked")
  public TypeVector<Operator<Keybindings, Preferences>, N2> getOperators() {
    return TypeVector.fill(PrimaryOperator, SecondaryOperator);
  }

  /**
   * Provides the absolute rotation of the superstructure's pivot accounting for both the offset and the pivot's minimum rotation
   * @return Rotation of the superstructure pivot
   */
  public static synchronized Rotation2d getRotation() {
    var Rotation =  Rotation2d.fromRotations(
      (-PIVOT_ABSOLUTE_ENCODER.getAbsolutePosition()) 
      + Measurements.ABSOLUTE_ENCODER_OFFSET 
      + Measurements.PIVOT_MINIMUM_ROTATION);
    if(Measurements.PIVOT_INVERTED) {
      Rotation = Rotation.unaryMinus();
    }
    return Rotation;
  }

  /**
   * Provides the velocity of the superstructure's pivot as RPM
   * @return Velocity of the superstructure pivot 
   */
  public static synchronized Double getVelocity() {
    return PIVOT_RELATIVE_ENCODER.getVelocity() * 60d;
  }
}
