package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;

public class ElevatorConstants {
    protected static final Motor
            ELEVATOR_LEFT_MOTOR = MotorFactory.createSpark("Elevator Left Motor", 0, MotorProperties.SparkType.MAX),
            ELEVATOR_RIGHT_MOTOR = MotorFactory.createSpark("Elevator Right Motor", 1, MotorProperties.SparkType.MAX);

    protected static final Sensor
            TOP_BEAM_BREAK = SensorFactory.createDigitalInput("Top Beam Breaker", 2),
            BOTTOM_BEAM_BREAK = SensorFactory.createDigitalInput("Button Beam Breaker", 3);

    private static final MotorConfiguration ELEVATOR_LEFT_MOTOR_CONFIGURATION = new MotorConfiguration();

    protected static final double
            ELEVATOR_MAX_EXTENSION_METERS = 0.86,
            WHEEL_DIAMETER = 0.05,
            WHEEL_SCOPE = Math.PI * WHEEL_DIAMETER;

    protected static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d("Elevator Mechanism", 0.8);

    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationSlot = new MotorProperties.Slot(70, 0, 0, 0, 0,  1.313);

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxVelocity = 10;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxAcceleration = 15;
        ELEVATOR_LEFT_MOTOR_CONFIGURATION.profileMaxJerk = 150;

        ELEVATOR_LEFT_MOTOR_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ELEVATOR,
                DCMotor.getNeoVortex(2),
                60,
                24,
                0.2,
                0.1,
                1.1,
        false
        );

// issue: The Elevator stops at a specific position for unknown
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ELEVATOR_LEFT_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);

        ELEVATOR_LEFT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
        ELEVATOR_RIGHT_MOTOR.configure(ELEVATOR_LEFT_MOTOR_CONFIGURATION);
    }
}