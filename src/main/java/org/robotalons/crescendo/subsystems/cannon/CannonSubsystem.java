package org.robotalons.crescendo.subsystems.cannon;

import org.robotalons.lib.cannon.Math.LauncherInterpolation;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;


public class CannonSubsystem extends PIDSubsystem{
    
    private TalonFX m_bottomShooter, m_topShooter;
    private CANSparkMax m_pivot;
    private DutyCycleEncoder pivotEncoder;  
    private VoltageOut leftOutput, rightOutput; 
    private static org.robotalons.lib.cannon.Math.LauncherInterpolation pivotMap;
    //private final double UNIT_CIRCLE_OFFSET = Math.toRadians(112);
    private final double FINAL_UNIT_CIRCLE_OFFSET =  Math.toRadians(80);
    private final double SUBWOOFER_DEGREES = 56;
    private final Pigeon2 encoder = new Pigeon2(10);

    public CannonSubsystem() {
        super(new PIDController(0, 0, 0));


        m_bottomShooter = new TalonFX(34);
        m_topShooter = new TalonFX(35);
        m_pivot = new CANSparkMax(32, MotorType.kBrushless);
        pivotMap = new LauncherInterpolation();
        pivotEncoder = new DutyCycleEncoder(0);

        configMotors();
        getController().setSetpoint(Math.toRadians(getPigeonMeasurement()));
        disable();
    }
    


    public void configMotors() {
        m_pivot.restoreFactoryDefaults();
        m_pivot.clearFaults();
        m_pivot.setIdleMode(IdleMode.kBrake);
        m_pivot.setInverted(false);
        m_pivot.setSmartCurrentLimit(40);
    }

    public void launchWithVolts() {
        m_bottomShooter.setControl(new VoltageOut(10));
        m_topShooter.setControl(new VoltageOut(6.25));
    }

    public void slowLaunchWithVolts() {
        m_bottomShooter.setControl(new VoltageOut(3.25));
        m_topShooter.setControl(new VoltageOut(3.25));
    }

    public void stopLaunchWithVolts() {
        m_bottomShooter.setControl(leftOutput.withOutput(0));
        m_topShooter.setControl(rightOutput.withOutput(0));
    }

    public void pivotUp() {
        disable();
        m_pivot.setVoltage(2);
    }

    public void pivotDown() {
        disable();
        // if (getDegrees() < 18.1) {
        //     m_pivot.setVoltage(0);
        // } else {
            m_pivot.setVoltage(-2);
      //  }
    }

    public void pivotStop() {
        disable();
        m_pivot.setVoltage(0);
        
    }

    



    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        
        //PIDmovePivot(MathUtil.clamp(output, -8, 12.3));
        PIDmovePivot(MathUtil.clamp(output, -4, 4));
    }

    public void PIDmovePivot(double volts) {
        double adjustedVolts = volts;
            //negative goes up & positive goes down PLIMPTON ABS ENCODER THEORY
        double constantV;
        double clampedVolts = MathUtil.clamp(adjustedVolts, -2, 2);
        if (clampedVolts > 0) {
            constantV = 0.18;
            m_pivot.setVoltage(clampedVolts + constantV);
        } else if (clampedVolts < 0) {
            constantV = 0.0;
            m_pivot.setVoltage(clampedVolts - 0);
        }
        
    }

    public double getPitch() {
        return encoder.getPitch().getValueAsDouble();
    }
    public double getYaw() {
        return encoder.getYaw().getValueAsDouble();
    }
    public double getRoll() {
        return encoder.getRoll().getValueAsDouble();
    }




    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
       return Math.toRadians(getPigeonMeasurement());
    }

 //   public double getAbsoluteMeasurement() {
 //       return (pivotEncoder.getAbsolutePosition() + 0.6) %1;
 //   }

      public double getPigeonMeasurement() {
        return 180 - (encoder.getRoll().getValueAsDouble());
    }

 //   public double getPivotRadians() {
 //       return ((getAbsoluteMeasurement() * 2 * Math.PI)/2) - FINAL_UNIT_CIRCLE_OFFSET;
 //   }

//    public double getDegrees() {
//        return Math.toDegrees(getPivotRadians());
//    }

    public void setSetpointDegrees(double degrees) {
        double newSetpoint = Math.toRadians(degrees);
        setSetpoint(newSetpoint);
        getController().reset();
        enable();
    }

    // public void interpolateSetpoint() {
    //    double newSetpoint =  Math.toRadians(SUBWOOFER_DEGREES);
    //    if (vision.isConnected()) {
    //        double interpolatedSetpoint = pivotMap.pivotMap.getInterpolated(new InterpolatingDouble(vision.calculateRange())).value;
    //        newSetpoint = Math.toRadians(interpolatedSetpoint);
    //    }
    //    setSetpoint(newSetpoint);
    //    getController().reset();
    //    enable();
    // }

    public void holdPosition() {
        setSetpoint(getMeasurement());
        getController().reset();
        enable();
    }

    public void incrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.25);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(newSetpoint + oldSetpoint);
        getController().reset();
        enable();
    }

    public void decrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.25);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(oldSetpoint - newSetpoint);
        getController().reset();
        enable();
    }

    public boolean isPivotReadyToShoot() {
        if (getController().atSetpoint()) {
            return true;
        } else {
            return false;
        }
     }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putBoolean("Pivot Ready", isPivotReadyToShoot());
        SmartDashboard.putNumber("adjusted pigeon", getPigeonMeasurement());
        SmartDashboard.putNumber("adjusted pigeon rads", Math.toRadians(getPigeonMeasurement()));
        SmartDashboard.putNumber("Pivot PID Controller", getController().getSetpoint());
        SmartDashboard.putNumber("Pivot PID measure", getMeasurement());
        // SmartDashboard.putNumber("pivot degrees", getDegrees());
        // SmartDashboard.putNumber("pivot absolute", getAbsoluteMeasurement());
        // SmartDashboard.putNumber("pivot radians", getPivotRadians());
        SmartDashboard.putNumber("pivot setpoint", getController().getSetpoint());

}
}