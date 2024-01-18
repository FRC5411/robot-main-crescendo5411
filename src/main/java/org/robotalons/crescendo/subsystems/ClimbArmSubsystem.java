package org.robotalons.crescendo.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbArmSubsystem extends SubsystemBase{
    private CANSparkMax leftArm;    
    private CANSparkMax rightArm;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public ClimbArmSubsystem(int id1, int id2){
        leftEncoder = leftArm.getEncoder();
        rightEncoder = rightArm.getEncoder();
        leftArm = configure(new CANSparkMax(id1, MotorType.kBrushed));
        rightArm = configure(new CANSparkMax(id1, MotorType.kBrushed));
        rightArm.follow(leftArm);
    }

    public void setArm(double armSpeed) {
        leftArm.set(armSpeed);
    }

    public double getAngle(){
        return leftEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leftArmEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("rightArmEncoder", rightEncoder.getPosition());
    }

    public CANSparkMax configure(CANSparkMax motor){
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(60);
        motor.setIdleMode(IdleMode.kBrake);
        motor.burnFlash();
        return motor;
    }
}
