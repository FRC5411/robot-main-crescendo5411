package org.robotalons.crescendo.subsystems.elevator;

import org.robotalons.crescendo.subsystems.elevator.REVElevator.ElevatorModuleConstants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

    public static class Motor {
        public static final ElevatorModuleConstants CONSTANTS = new ElevatorModuleConstants();
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
            CONSTANTS.PID_CONTROLLER = new PIDController(
                Measurements.PID.KP,
                Measurements.PID.KI,
                Measurements.PID.KD);
        }
    }
}
