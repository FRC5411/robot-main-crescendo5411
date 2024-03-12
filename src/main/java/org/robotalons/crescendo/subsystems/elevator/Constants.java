package org.robotalons.crescendo.subsystems.elevator;

import org.robotalons.crescendo.subsystems.elevator.REVElevator.REVElevatorConstants;
import org.robotalons.crescendo.subsystems.elevator.REVElevatorSim.REVElevatorSimConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;


public class Constants {
    public class Ports {
        public static final Integer ELEVATOR_PORT = (0);
        public class Encoder{
            public static final Integer CHANNEL_A = (1);
            public static final Integer CHANNEL_B = (2);
        }
    }
    public class Measurements{
        public static final Integer CURRENT_LIMIT = (0);
        // Reason for Sim and Real systems to use the K values
        // is because my intention is to make both intefaces the 
        // exact same in terms of performance
        public class Feedforward{
            public static final Double KS = (0.0);
            public static final Double KG = (0.0);
            public static final Double KV = (0.0);
        }
        public class PID {
            public static final Double KP = (0.0);
            public static final Double KI = (0.0);
            public static final Double KD = (0.0);
        }
    }

    public static class SimMotor {
        public static final REVElevatorSimConstants CONSTANTS = new REVElevatorSimConstants();
        static{
            CONSTANTS.MOTOR = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                    DCMotor.getNEO(1), 
                    CONSTANTS.MASS_KG,
                    CONSTANTS.RADIUS,
                    CONSTANTS.GEAR_RATIO),
                DCMotor.getNEO(1),
                CONSTANTS.MIN_HEIGHT,
                CONSTANTS.MAX_HEIGHT,
                true,
                CONSTANTS.BASE_HEIGHT);
            CONSTANTS.ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(
                Measurements.Feedforward.KS,
                Measurements.Feedforward.KG, 
                Measurements.Feedforward.KV);
            CONSTANTS.PID_CONTROLLER = new ProfiledPIDController(
                Measurements.PID.KP,
                Measurements.PID.KI,
                Measurements.PID.KD,
                new TrapezoidProfile.Constraints(1000.0, 500.0));
        }
    }

    public static class RealMotor{
        public static final REVElevatorConstants CONSTANTS = new REVElevatorConstants();
        static {
            CONSTANTS.elevatorMotor = new CANSparkMax(Constants.Ports.ELEVATOR_PORT,MotorType.kBrushless);
            CONSTANTS.elevatorMotor.setIdleMode(IdleMode.kBrake);
            CONSTANTS.elevatorMotor.restoreFactoryDefaults();
            CONSTANTS.elevatorMotor.clearFaults();
            CONSTANTS.elevatorMotor.setSmartCurrentLimit(Constants.Measurements.CURRENT_LIMIT);

            CONSTANTS.encoder = new Encoder(Constants.Ports.Encoder.CHANNEL_A,Constants.Ports.Encoder.CHANNEL_B);
            CONSTANTS.pidController = new ProfiledPIDController(
                Measurements.PID.KP,
                Measurements.PID.KI,
                Measurements.PID.KI, 
                new TrapezoidProfile.Constraints(1600.0, 800.0));
            CONSTANTS.feedforward = new ElevatorFeedforward(
              Measurements.Feedforward.KS,
              Measurements.Feedforward.KG, 
              Measurements.Feedforward.KV);
        }
    }
}
