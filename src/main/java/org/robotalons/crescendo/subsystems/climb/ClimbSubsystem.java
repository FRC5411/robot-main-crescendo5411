// ----------------------------------------------------------------[Package]---------------------------------------------------------------- //
package org.robotalons.crescendo.subsystems.climb;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.Constants.Profiles.Keybindings;
import org.robotalons.crescendo.Constants.Profiles.Preferences;
import org.robotalons.crescendo.subsystems.climb.Constants.Measurements;
import org.robotalons.crescendo.subsystems.climb.Constants.Objects;
import org.robotalons.crescendo.subsystems.climb.Constants.Ports;
import org.robotalons.lib.TalonSubsystemBase;
import org.robotalons.lib.utilities.Operator;
import org.robotalons.lib.utilities.TypeVector;
// ------------------------------------------------------------[Climb Subsystem]------------------------------------------------------------ //
/**
 *
 * <h1>IndexerSubsystem</h1>
 *
 * <p>Utility class which controls the indexing of notes from and to the intake and shooter.<p>
 *
 * @see SubsystemBase
 * @see org.robotalons.crescendo.RobotContainer RobotContainer
 */
public final class ClimbSubsystem extends TalonSubsystemBase<Keybindings, Preferences, N1> {
  // --------------------------------------------------------------[Constants]-------------------------------------------------------------- //
  private static final CANSparkMax LEFT_ARM;
  private static final CANSparkMax RIGHT_ARM;

  private static final DutyCycleEncoder LEFT_ABSOLUTE_ENCODER;
  private static final RelativeEncoder LEFT_RELATIVE_ENCODER;
  private static final DutyCycleEncoder RIGHT_ABSOLUTE_ENCODER;
  private static final RelativeEncoder RIGHT_RELATIVE_ENCODER;

  private static final ArmFeedforward LEFT_FF;
  private static final ArmFeedforward RIGHT_FF;

  private static final PIDController LEFT_PID;
  private static final PIDController RIGHT_PID;
  // ---------------------------------------------------------------[Fields]---------------------------------------------------------------- //
  private static volatile Operator<Keybindings, Preferences> Operator;
  private static volatile Double Rotation;
  private static ClimbSubsystem Instance;
  // ------------------------------------------------------------[Constructors]------------------------------------------------------------- //

  /**
   * Climb Subsystem Constructor
   */
  public ClimbSubsystem() {
    super(("Climb-Subsystem-Subsystem"), Nat.N1());
  } static {
    Operator = (null);
    LEFT_ARM = new CANSparkMax(Ports.LEFT_ARM_CONTROLLER_ID, MotorType.kBrushless);
    RIGHT_ARM = new CANSparkMax(Ports.RIGHT_ARM_CONTROLLER_ID, MotorType.kBrushless);

    LEFT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.LEFT_ENCODER_ID);
    RIGHT_ABSOLUTE_ENCODER = new DutyCycleEncoder(Ports.RIGHT_ENCODER_ID);

    LEFT_RELATIVE_ENCODER = LEFT_ARM.getEncoder();
    RIGHT_RELATIVE_ENCODER = RIGHT_ARM.getEncoder();

    LEFT_PID = new PIDController(
      Measurements.LEFT_ARM_P,
      Measurements.LEFT_ARM_I,
      Measurements.LEFT_ARM_D);
    RIGHT_PID = new PIDController(
      Measurements.RIGHT_ARM_P,
      Measurements.RIGHT_ARM_I,
      Measurements.RIGHT_ARM_D);

    LEFT_FF = new ArmFeedforward(
      Constants.Measurements.LEFT_ARM_KS,
      Constants.Measurements.LEFT_ARM_KG,
      Constants.Measurements.LEFT_ARM_KV,
      Constants.Measurements.LEFT_ARM_KA);

    RIGHT_FF = new ArmFeedforward(
      Constants.Measurements.LEFT_ARM_KS,
      Constants.Measurements.LEFT_ARM_KG,
      Constants.Measurements.LEFT_ARM_KV,
      Constants.Measurements.LEFT_ARM_KA);

    LEFT_ARM.setIdleMode(IdleMode.kBrake);
    LEFT_ARM.restoreFactoryDefaults();
    LEFT_ARM.clearFaults();

    LEFT_PID.setP(Measurements.LEFT_ARM_P);
    LEFT_PID.setI(Measurements.LEFT_ARM_I);
    LEFT_PID.setD(Measurements.LEFT_ARM_D);

    RIGHT_PID.setP(Measurements.RIGHT_ARM_P);
    RIGHT_PID.setI(Measurements.RIGHT_ARM_I);
    RIGHT_PID.setD(Measurements.RIGHT_ARM_D);

    LEFT_ARM.setSmartCurrentLimit(Measurements.CURRENT_LIMIT);

    RIGHT_ARM.setIdleMode(IdleMode.kBrake);
    RIGHT_ARM.restoreFactoryDefaults();
    RIGHT_ARM.clearFaults();

    RIGHT_ARM.setSmartCurrentLimit(Measurements.CURRENT_LIMIT);

    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kForward,
    ((Double) Units.radiansToRotations(Measurements.MAXIMUM_ARM_ROTATION)).floatValue());

    RIGHT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse,
    ((Double) Units.radiansToRotations(Measurements.MINIMUM_ARM_ROTATION)).floatValue());

    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kForward,
    ((Double) Units.radiansToRotations(Measurements.MAXIMUM_ARM_ROTATION)).floatValue());

    LEFT_ARM.setSoftLimit(
    SoftLimitDirection.kReverse,
    ((Double) Units.radiansToRotations(Measurements.MINIMUM_ARM_ROTATION)).floatValue());

    LEFT_ARM.setInverted((true));
  }

  // ------------------------------------------------------ ---------[Methods]--------------------------------------------------------------- //
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCKER.lock();

    Logger.recordOutput(("Climb-Subsystem/Left-Absolute-Position"), Units.rotationsToDegrees(getPosition(ClimbState.LEFT)));
    Logger.recordOutput(("Climb-Subsystem/Right-Absolute-Position"), Units.rotationsToDegrees(getPosition(ClimbState.RIGHT)));

    Logger.recordOutput(("Climb-Subsystem/Left-Relative-Position"), Units.rotationsToDegrees(LEFT_RELATIVE_ENCODER.getPosition()));
    Logger.recordOutput(("Climb-Subsystem/Right-Relative-Position"), Units.rotationsToDegrees(RIGHT_RELATIVE_ENCODER.getPosition()));

    Logger.recordOutput(("Climb-Subsystem/Left-Velocity"), LEFT_RELATIVE_ENCODER.getVelocity());
    Logger.recordOutput(("Climb-Subsystem/Right-Velocity"), RIGHT_RELATIVE_ENCODER.getVelocity());

    Logger.recordOutput(("Climb-Subsystem/Left-Temperature"), LEFT_ARM.getMotorTemperature());
    Logger.recordOutput(("Climb-Subsystem/Right-Temperature"), RIGHT_ARM.getMotorTemperature());

    Logger.recordOutput(("Climb-Subsystem/Left-Voltage"), LEFT_ARM.getBusVoltage());
    Logger.recordOutput(("Climb-Subsystem/Right-Voltage"), RIGHT_ARM.getBusVoltage());

    set(Rotation);
    Objects.ODOMETRY_LOCKER.unlock();
  }

  @Override
  public synchronized void close() {
    LEFT_ARM.close();
    RIGHT_ARM.close();
  }

  @Override
  public synchronized void configure(final TypeVector<Operator<Keybindings, Preferences>,N1> Operators) {
    ClimbSubsystem.Operator = Operators.get((0));
    ClimbSubsystem.Operator.getOptionalKeybinding(Keybindings.CLIMB_ROTATE_FORWARD).ifPresent((Trigger) -> {
      Trigger.onTrue(new InstantCommand(() -> {
        LEFT_ARM.set((0.3d));
        RIGHT_ARM.set((0.3d));
      },getInstance()));
      Trigger.onFalse(new InstantCommand(() -> {
        LEFT_ARM.set((0d));
        RIGHT_ARM.set((0d));
      },getInstance()));
    });
    ClimbSubsystem.Operator.getOptionalKeybinding(Keybindings.CLIMB_ROTATE_FORWARD).ifPresent((Trigger) -> {
      Trigger.onTrue(new InstantCommand(() -> {
        LEFT_ARM.set(-(0.3d));
        RIGHT_ARM.set(-(0.3d));
      },getInstance()));
      Trigger.onFalse(new InstantCommand(() -> {
        LEFT_ARM.set((0d));
        RIGHT_ARM.set((0d));
      },getInstance()));
    });
  }
  // --------------------------------------------------------------[Internal]--------------------------------------------------------------- //
  /**
   * Represents directions on the climb subsystem on the physical robot, i.e. which side the hardware lies on.
   */
  public enum ClimbState {
    LEFT,
    RIGHT,
  }
  // --------------------------------------------------------------[Mutators]--------------------------------------------------------------- //
  /**
   * Mutates the demand velocities of either side of the climb subsystem
   * @param Direction Side of the climb subsystem being demanded
   * @param Demand    Queried load on the motor controller object, which must lie between -1 and +1
   */
  public synchronized void set(final ClimbState Direction, final Double Demand) {
    switch(Direction) {
      case LEFT:
        LEFT_ARM.setVoltage(LEFT_PID.calculate(getPosition(ClimbState.LEFT), Demand) + LEFT_FF.calculate(Demand, (0)));
        break;
      case RIGHT:
        RIGHT_ARM.setVoltage(RIGHT_PID.calculate(getPosition(ClimbState.RIGHT), Demand) + RIGHT_FF.calculate(Demand, (0)));
        break;
    }
  }

  /**
   * Mutates the demand velocities of both sides of the climb subsystem
   * @param Demand Queried load on the motor controller object, which must lie between -1 and +1
   */
  public synchronized void set(final Double Demand) {
  //   LEFT_ARM.setVoltage(LEFT_PID.calculate(getPosition(ClimbState.LEFT), Demand) + LEFT_FF.calculate(Demand, (0)));
  //   RIGHT_ARM.setVoltage(RIGHT_PID.calculate(getPosition(ClimbState.RIGHT), Demand) + RIGHT_FF.calculate(Demand, (0)));
  }
  // --------------------------------------------------------------[Accessors]-------------------------------------------------------------- //
  /**
   * Provides the current value of teh absolute encoders of a side of the climb with the offset applied.
   * @param Direction Side of the climb subsystem being demanded
   * @return Current absolute value of the absolute encoders, with a constant offset
   */
  public static synchronized Double getPosition(final ClimbState Direction) {
    return switch (Direction) {
      case LEFT -> LEFT_ABSOLUTE_ENCODER.getAbsolutePosition() - Measurements.LEFT_ENCODER_OFFSET;
      case RIGHT -> RIGHT_ABSOLUTE_ENCODER.getAbsolutePosition() - Measurements.RIGHT_ENCODER_OFFSET;
    };
  }

  @Override
  @SuppressWarnings("unchecked")
  public TypeVector<Operator<Keybindings, Preferences>, N1> getOperators() {
    return TypeVector.fill(Operator);
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
  public static synchronized ClimbSubsystem getInstance() {
    if (java.util.Objects.isNull(Instance)) {
      Instance = new ClimbSubsystem();
    }
    return Instance;
  }
}