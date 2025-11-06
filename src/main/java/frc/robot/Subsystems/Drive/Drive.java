package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import static frc.robot.Subsystems.Drive.TunerConstants.kSpeedAt12Volts;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import java.util.Vector;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Subsystems.Vision.Vision;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Drive extends Subsystem<DriveStates> {

	private static Drive instance;

	private DriveIO driveIO;
	private DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
	private boolean robotMirrored = false;
	private Pose2d lastPose = new Pose2d();
	private double lastTime = 0;
	private final Field2d field = new Field2d();
	private Vision vision = Vision.getInstance();

	private PIDController thetaController = new PIDController(2, 0, 0);

	/**
	 * Constructs a new Drive subsystem with the given DriveIO.
	 *
	 * @param driveIO The DriveIO object used for controlling the drive system.
	 */
	private Drive() {
		super("Drive", DriveStates.FIELD_RELATIVE);
		this.driveIO = switch (ROBOT_MODE) {
			case REAL -> new DriveIOReal();
			case SIM -> new DriveIOSim();
			case TESTING -> new DriveIOReal();
		};

		// Zero Gyro
		addRunnableTrigger(
			() -> {
				driveIO.zeroGyro();
			},
			DRIVER_CONTROLLER::getStartButtonPressed
		);
	}

	/**
	 * Returns the singleton instance of the Drive subsystem.
	 *
	 * @return The Drive Instance.
	 */
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	@Override
	public void runState() {
		driveIO.updateInputs(inputs);
		Logger.processInputs("Drive", inputs);

		if (DriverStation.isDisabled()) robotMirrored = false;

		// Zero on init/when first disabled
		if (!robotMirrored && !DriverStation.isDisabled()) {
			DriverStation.getAlliance()
				.ifPresent(allianceColor -> {
					driveIO.getDrive().setOperatorPerspectiveForward(allianceColor == Alliance.Red ? RED_ALLIANCE_PERSPECTIVE_ROTATION : BLUE_ALLIANCE_PERSPECTIVE_ROTATION);
					robotMirrored = true;
				});
		}
		logOutputs(driveIO.getDrive().getState());
		
		if (!DRIVER_CONTROLLER.getAButton()) {
			getState().driveRobot();
		} else {
			if (vision.getYawToTarget() != null) 
			{
				Matrix<N4, N4> robotToFRMatrix = ROBOT_TO_FRONT_RIGHT_CAMERA.div(ROBOT_TO_FRONT_RIGHT_CAMERA.getTranslation().getNorm()).toMatrix();
				Matrix<N4, N4> cameraToObj= new Transform3d(Translation3d.kZero, new Rotation3d(0,0, vision.getYawToTarget().in(Radians))).toMatrix();
				Matrix<N4, N4> robotToObj = robotToFRMatrix.times(robotToFRMatrix.inv()).times(cameraToObj);
				double currentAngle = getDriveTrain().getPigeon2().getYaw().getValueAsDouble() % 360;
				// double wantedAngle = currentAngle - (vision.getYawToTarget().in(Degree) + ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION.getZ());

				driveFieldRelative(0, 0,  .1 * thetaController.calculate(currentAngle, new Transform3d(robotToObj).getRotation().getMeasureZ().in(Degree)));
			}
		}

		field.setRobotPose(getPose());
		SmartDashboard.putData("Field", field);
	}

	/**
	 * Logs the outputs of the drive system.
	 *
	 * @param state The current state of the SwerveDrive.
	 */
	public void logOutputs(SwerveDriveState state) {
		Logger.recordOutput(SUBSYSTEM_NAME + "/Robot Pose", state.Pose);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Current Time", Utils.getSystemTimeSeconds());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Chassis Speeds", state.Speeds);
		Logger.recordOutput(SUBSYSTEM_NAME + "/velocity", Units.metersToFeet(Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond)));
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModuleStates", state.ModuleStates);
		Logger.recordOutput(SUBSYSTEM_NAME + "/swerveModulePosition", state.ModulePositions);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Translation Difference", state.Pose.getTranslation().minus(lastPose.getTranslation()));
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Pose Jumped", Math.hypot(state.Pose.getTranslation().minus(lastPose.getTranslation()).getX(), state.Pose.getTranslation().minus(lastPose.getTranslation()).getY()) > (kSpeedAt12Volts.in(MetersPerSecond) * 2 * (Utils.getSystemTimeSeconds() - lastTime)));
		FIELD.setRobotPose(lastPose);

		lastPose = state.Pose;
		lastTime = Utils.getSystemTimeSeconds();
	}

	public void driveFieldRelative(double xVelocity, double yVelocity, double angularVelocity) {
		driveIO.setControl(new SwerveRequest.FieldCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
	}

	public void driveRobotRelative(double xVelocity, double yVelocity, double angularVelocity) {
		driveIO.setControl(new SwerveRequest.RobotCentric().withDeadband(DEADBAND).withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(angularVelocity).withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo));
	}

	public void zeroGyro() {
		driveIO.zeroGyro();
	}

	// SYSId Trash (no hate ofc)
	public enum SysIdMode {
		TRANSLATION,
		STEER,
		ROTATION,
	}

	// Util
	public Pose2d getPose() {
		return driveIO.getDrive().getState().Pose;
	}

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return driveIO.getDrive().getState().Speeds;
	}

	public LinearVelocity getVelocity() {
		return MetersPerSecond.of(Math.hypot(driveIO.getDrive().getState().Speeds.vxMetersPerSecond, driveIO.getDrive().getState().Speeds.vyMetersPerSecond));
	}

	public TunerSwerveDrivetrain getDriveTrain() {
		return driveIO.getDrive();
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (ROBOT_MODE == RobotMode.REAL) {
			driveIO.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), visionMeasurementStdDevs);
		} else {
			driveIO.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
	}
}
