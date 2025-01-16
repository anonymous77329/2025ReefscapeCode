package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.hardware.sensors.Sensor;
import frc.lib.generic.hardware.sensors.SensorFactory;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.ElevatorMechanism2d;

import static frc.robot.utilities.PortsConstants.ElevatorPorts.*;

public class ElevatorConstants {
    protected static final Motor
            MAIN_MOTOR = MotorFactory.createSpark("Elevator Main Motor", MAIN_MOTOR_PORT, MotorProperties.SparkType.MAX),
            FOLLOWER_MOTOR = MotorFactory.createSpark("Elevator Follower Motor", FOLLOWER_MOTOR_PORT, MotorProperties.SparkType.MAX);

    protected static final Sensor
            TOP_BEAM_BREAK = SensorFactory.createDigitalInput("Top Beam Breaker", TOP_BEAM_BREAK_PORT),
            BOTTOM_BEAM_BREAK = SensorFactory.createDigitalInput("Button Beam Breaker", BOTTOM_BEAM_BREAK_PORT);

    private static final MotorConfiguration ELEVATOR_MOTORS_CONFIGURATION = new MotorConfiguration();

    protected static final double
            ELEVATOR_MAX_EXTENSION_METERS = 0.86,
            WHEEL_DIAMETER = 0.05,
            WHEEL_SCOPE = Math.PI * WHEEL_DIAMETER;

    protected static final ElevatorMechanism2d MECHANISM = new ElevatorMechanism2d("Elevator Mechanism", 1);

    static {
        configureMotorConfiguration();
    }

    private static void configureMotorConfiguration() {
        ELEVATOR_MOTORS_CONFIGURATION.idleMode = MotorProperties.IdleMode.BRAKE;
        ELEVATOR_MOTORS_CONFIGURATION.simulationSlot = new MotorProperties.Slot(9, 0, 0, 0, 0,  1.313);// S=1.313

        ELEVATOR_MOTORS_CONFIGURATION.profileMaxVelocity = 10;
        ELEVATOR_MOTORS_CONFIGURATION.profileMaxAcceleration = 15;
        ELEVATOR_MOTORS_CONFIGURATION.profileMaxJerk = 150;

        ELEVATOR_MOTORS_CONFIGURATION.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ELEVATOR,
                DCMotor.getNeoVortex(2),
                60,
                24,
                0.2,
                0.1,
                5.1,
        false
        );

// issue: The Elevator don't follow her path correctly

        MAIN_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MAIN_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        MAIN_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);

        MAIN_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
        FOLLOWER_MOTOR.configure(ELEVATOR_MOTORS_CONFIGURATION);
    }
}