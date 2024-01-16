// -------------------------------------------------------------------[Package]---------------------------------------------------------------//
package org.robotalons.lib;

// ------------------------------------------------------------------[Libraries]--------------------------------------------------------------//
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

// ---------------------------------------------------------[Swerve Configurations Class]------------------------------------------------------//
/**
 *
 *
 * <h1>SwerveConfigurations</h1>
 *
 * <p>Contains useful statically-implemented swerve configurations for supported CTRE and REV
 * hardware.
 */
public final class SwerveConfigurations {
    // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
    /**
     * Configure a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX WPI_TalonFX} rotational
     * controller to swerve module specifications
     *
     * @param Controller Controller that is to be configured
     * @param AzimuthEncoder Azimuth CANCoder instance acting as a feedback filter reference point,
     *     and an azimuth sensor position source
     * @param CurrentLimit The current limit configuration of the motor's stator.
     * @param Deadband Desired percent deadband of controller inputs
     * @param Inverted If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureController(
            final WPI_TalonFX Controller,
            final WPI_CANCoder AzimuthEncoder,
            final StatorCurrentLimitConfiguration CurrentLimit,
            final Double Deadband,
            final Boolean Inverted) {
        Controller.configFactoryDefault();
        Controller.clearStickyFaults();
        Controller.setInverted(Inverted);
        Controller.setNeutralMode(NeutralMode.Brake);
        Controller.configRemoteFeedbackFilter(AzimuthEncoder, (0));
        Controller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        Controller.configStatorCurrentLimit(CurrentLimit);
        Controller.setSelectedSensorPosition(AzimuthEncoder.getAbsolutePosition());
        Controller.configNeutralDeadband(Deadband);
        Controller.configVoltageCompSaturation((12));
        Controller.setInverted(TalonFXInvertType.CounterClockwise);
        return Controller;
    }

    /**
     * Configure a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX WPI_TalonFX} translation
     * controller to swerve module specifications
     *
     * @param Controller Controller that is to be configured
     * @param CurrentLimit The current limit configuration of the motor's stator.
     * @param Inverted If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureTranslationController(
            final WPI_TalonFX Controller,
            final StatorCurrentLimitConfiguration CurrentLimit,
            final Boolean Inverted) {
        Controller.configFactoryDefault();
        Controller.clearStickyFaults();
        Controller.setInverted(Inverted);
        Controller.setNeutralMode(NeutralMode.Brake);
        Controller.configStatorCurrentLimit(CurrentLimit);
        Controller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        Controller.setSelectedSensorPosition((0.0));
        Controller.enableVoltageCompensation((true));
        return Controller;
    }

    /**
     * Configure a {@link com.ctre.phoenix.sensors.CANCoder CANCoder} azimuth encoder to swerve
     * module specifications
     *
     * @param AzimuthEncoder Encoder that is to be configured
     * @param Offset The starting encoder's offset
     * @param Inverted If the encoder output should be mirrored
     * @return A copy of the configured encoder
     */
    public static WPI_CANCoder configureRotationEncoder(
            final WPI_CANCoder AzimuthEncoder, final Double Offset, final Boolean Inverted) {
        AzimuthEncoder.configFactoryDefault();
        AzimuthEncoder.clearStickyFaults();
        AzimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        AzimuthEncoder.configSensorDirection(Inverted);
        AzimuthEncoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition);
        AzimuthEncoder.configGetFeedbackTimeBase(SensorTimeBase.PerSecond.value);
        AzimuthEncoder.configMagnetOffset(Offset);
        AzimuthEncoder.setPositionToAbsolute();
        return AzimuthEncoder;
    }

    /**
     * Configure a {@link com.revrobotics.CANSparkMax CANSparkMax} translation or rotation
     * controller to swerve module specifications
     *
     * @param Controller Controller that is to be configured
     * @param AmpLimit The current limit of the controller measured in amps
     * @param NominalVoltage The nominal voltage to compensate output of the controller to
     *     compensate voltage for
     * @param Inverted If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static CANSparkMax configureController(
            final CANSparkMax Controller,
            final Integer AmpLimit,
            final Double NominalVoltage,
            final Boolean Inverted) {
        Controller.restoreFactoryDefaults();
        Controller.clearFaults();
        Controller.setInverted(Inverted);
        Controller.setSmartCurrentLimit(AmpLimit);
        Controller.setIdleMode(IdleMode.kBrake);
        Controller.enableVoltageCompensation(NominalVoltage);
        Controller.burnFlash();
        return Controller;
    }

    /**
     * Configure a {@link com.revrobotics.RelativeEncoder RelativeEncoder} of a translation or
     * rotation controller to swerve module specifications
     *
     * @param Encoder Encoder that is to be configured
     * @param VelocityConversionFactor Factor of conversion when outputting velocity
     * @param PositionConversionFactor Factor of conversion when outputting position
     * @return
     */
    public static RelativeEncoder configureEncoder(
            final RelativeEncoder Encoder,
            final Double VelocityConversionFactor,
            final Double PositionConversionFactor) {
        Encoder.setPosition((0.0));
        Encoder.setVelocityConversionFactor(VelocityConversionFactor);
        Encoder.setPositionConversionFactor(PositionConversionFactor);
        return Encoder;
    }
}
