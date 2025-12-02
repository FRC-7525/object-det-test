// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Lib.BallDetectionTest;

import org.team7525.CI.CrashCheck;

public final class Main {

	private Main() {}

	public static void main(String... args) {
		RobotBase.startRobot("Ball".equals(System.getenv("CI_NAME")) ? () -> new BallDetectionTest() : Robot::new);
	}
}
