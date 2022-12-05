/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public final class Constants {

	public static final class USBConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;
		public static final int AUXDRIVER_CONTROLLER_PORT = 1;
	}

	public static final class XboxConstants {

		// Button mappings
		static public int D_PAD = 0;
		static public int A_BUTTON = 1;
		static public int B_BUTTON = 2;
		static public int X_BUTTON = 3;
		static public int Y_BUTTON = 4;
		static public int LB_BUTTON = 5;
		static public int RB_BUTTON = 6;
		static public int BACK_BUTTON = 7;
		static public int START_BUTTON = 8;
		static public int LEFT_STICK = 9;
		static public int RIGHT_STICK = 10;

		// Axis control mappings
		// Notes:
		// - Left and right trigger use axis 3
		// - Left trigger range: 0 to 1
		// - Right trigger range: 0 to -1).
		static public int LEFT_AXIS_X = 0;
		static public int LEFT_AXIS_Y = 1;
		static public int LEFT_TRIGGER_AXIS = 2;
		static public int RIGHT_TRIGGER_AXIS = 3;
		static public int RIGHT_AXIS_X = 4;
		static public int RIGHT_AXIS_Y = 5;

		// Direction pad lookup angles
		static public int POV_UP = 0;
		static public int POV_RIGHT = 90;
		static public int POV_DOWN = 180;
		static public int POV_LEFT = 270;

	}


	public static final class DriveConstants {

		// middle to middle of the wheel
		public static final double TRACK_WIDTH_METERS = 0.638;
		public static final int TALONFX_ENCODER_CPR = 2048;
		public static final double GEAR_RATIO = 10.71; 
		public static final double WHEEL_DIAMETER_METERS = 0.1524;
		public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
		public static final double RAMSETE_B = 2.0;
		public static final double RAMSETE_ZETA = 0.7;
		public static final double MAX_METERS_PER_SECOND = 0.25;
		public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;
		public static final double MAX_VOLTAGE_AUTO = 10;
		public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
				TRACK_WIDTH_METERS);
		public static final SimpleMotorFeedforward SIMPLE_MOTOR_FEED_FOWARD = new SimpleMotorFeedforward(
				PidConstants.KS_FEEDFOWARD, PidConstants.KV_FEEDFOWARD, PidConstants.KA_FEEDFOWARD);
	}

	public static final class CanIdConstants {
		public static final int LEFT_MASTER_ID = 1;
		public static final int LEFT_FOLLOWER_ID = 2;
		public static final int RIGHT_MASTER_ID = 3;
		public static final int RIGHT_FOLLOWER_ID = 4;


	}

	public static final class PidConstants {
		public static final double KS_FEEDFOWARD = 1.2;
		public static final double KV_FEEDFOWARD = 0.329;
		public static final double KA_FEEDFOWARD = 0.0933;
		public static final double OPTIMAL_KP = 4;
		public static final double OPTIMAL_KD = 4.51;
	}

	public static final class AutoAimConstants {
		public static final double KP_ROTATION_AUTOAIM = 0.025;
		public static final double KD_ROTATION_AUTOAIM = 0.0006;
		public static final double ANGLE_TOLERANCE = 1.0; // IN DEGREES
	}

	public static final class VoltageConstants {
		public static final double STOP = 0;
		public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				DriveConstants.SIMPLE_MOTOR_FEED_FOWARD, DriveConstants.DRIVE_KINEMATICS,
				DriveConstants.MAX_VOLTAGE_AUTO);
		public static final double TURN_VOLTAGE_COMPENSATION_VOLTS = 5;

	}

}