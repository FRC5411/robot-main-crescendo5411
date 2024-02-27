package org.robotalons.crescendo.subsystems.elevator;

public class Constants {
    public class Ports {
        public static final Integer ELEVATOR_PORT = (0);
        public class Encoder{
            public static final Integer CHANNEL_A = (1);
            public static final Integer CHANNEL_B = (2);
        }
    }
    public class Measurements{
        public static final Integer CURRENT_LIMIT = (60);
        public class Feedforward{
            public static final Double KS = (0.0);
            public static final Double KG = (0.0);
            public static final Double KV = (0.0);
        }
    }
}
