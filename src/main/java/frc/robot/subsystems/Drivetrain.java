// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.OperatorConstants;

public class Drivetrain extends SubsystemBase {

	private CANSparkMax frontLeft;
	private CANSparkMax frontRight;
	private CANSparkMax backLeft;
	private CANSparkMax backRight;
	private CommandXboxController controller;
	private DifferentialDrive drive;

	public Drivetrain(CommandXboxController controller) {
		this.controller = controller;

		frontLeft = new CANSparkMax(0, MotorType.kBrushless);
		frontRight = new CANSparkMax(1, MotorType.kBrushless);
		backLeft = new CANSparkMax(2, MotorType.kBrushless);
		backRight = new CANSparkMax(3, MotorType.kBrushless);

		backLeft.follow(frontLeft);
		backRight.follow(frontRight);

		drive = new DifferentialDrive(frontLeft, frontRight);

		frontRight.setInverted(true);
		backRight.setInverted(true);
	}

	@Override
	public void periodic() {
		double leftPower = controller.getRawAxis(OperatorConstants.XBOX_LEFT_Y_AXIS);
		double rightPower = controller.getRawAxis(OperatorConstants.XBOX_RIGHT_Y_AXIS);

		// Apply deadbanding and limit motor power
		if (Math.abs(leftPower) < ControlConstants.JOY_DEADBAND)
			leftPower = 0;
		else if (leftPower > ControlConstants.MAX_ROBOT_SPEED)
			leftPower = ControlConstants.MAX_ROBOT_SPEED;
		else if (leftPower < ControlConstants.MIN_ROBOT_SPEED)
			leftPower = ControlConstants.MIN_ROBOT_SPEED;

		if (Math.abs(rightPower) < ControlConstants.JOY_DEADBAND)
			rightPower = 0;
		else if (rightPower > ControlConstants.MAX_ROBOT_SPEED)
			rightPower = ControlConstants.MAX_ROBOT_SPEED;
		else if (rightPower < ControlConstants.MIN_ROBOT_SPEED)
			rightPower = ControlConstants.MIN_ROBOT_SPEED;

		drive.tankDrive(leftPower, rightPower);
	}

	public void stop() {
		frontLeft.set(0);
		frontRight.set(0);
		backLeft.set(0);
		backRight.set(0);
	}
}
