package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.encoder.EncoderFactory;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;

public class ElevatorConstants {
    protected static final Motor
            ELEVATOR_LEFT_MOTOR = MotorFactory.createSpark("Elevator Left Motor", 0, MotorProperties.SparkType.MAX),
            ELEVATOR_RIGHT_MOTOR = MotorFactory.createSpark("Elevator Right Motor", 1, MotorProperties.SparkType.MAX);

    protected static final Encoder ELEVATOR_ENCODER = EncoderFactory.createCanCoder("Elevator Encoder", 2);

    private static final MotorConfiguration ELEVATOR_LEFT_MOTOR_CONFIGURATION = new MotorConfiguration();
    private static final MotorConfiguration ELEVATOR_ENCODER_CONFIGURATION = new MotorConfiguration();

    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationSlot = new MotorProperties.Slot(1, 0, 0);

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(2),
                20,
                0.003
        );

        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.POSITION);

        ELEVATOR_LEFT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
        ELEVATOR_RIGHT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
    }
}
