package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_LEFT_MOTOR;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_RIGHT_MOTOR;

public class Elevator extends SubsystemBase {
    public Command setElevatorVoltage(boolean setInverted){
        return new FunctionalCommand(
                () -> {},

                () -> {setMotorVoltage(setInverted);},

                (interrupt) -> {
                },

                () -> false,

                this
        );
    }

    public void periodic() {
        Pose3d current3DPose = new Pose3d(0, 0, getCurrentPosition(), new Rotation3d(0, 0, 0));
        Logger.recordOutput("Elevator", current3DPose);
    }

// need to add if statement so that the elevator won't brake while I press and to start
// the motors and the elevator is in the end
    private void setMotorVoltage(boolean isInverted) {
        final double targetVoltage = isInverted ? -4 : 4;

        ELEVATOR_LEFT_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, targetVoltage);
        ELEVATOR_RIGHT_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, targetVoltage);
    }

    private double getCurrentPosition() {
        return ELEVATOR_LEFT_MOTOR.getSystemPosition();
    }
}
