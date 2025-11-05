package frc.robot.Subsystems.Vision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.Drive.Drive;
import java.util.Set;

public class VisionConstants {

	public enum CameraResolution {
		HIGH_RESOLUTION,
		NORMAL,
	}

	// Front Left
	public static final String FRONT_LEFT_CAM_NAME = "Front Left Camera";
	public static final Translation3d ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.809459), Units.inchesToMeters(11.164206), Units.inchesToMeters(8.887162));
	public static final Rotation3d ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(-30.4));
	public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION);

	// Front Right
	public static final String FRONT_RIGHT_CAM_NAME = "Front Right Camera";
	public static final Translation3d ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(11.809459), Units.inchesToMeters(-11.164206), Units.inchesToMeters(8.887162));
	public static final Rotation3d ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(27.8));
	public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION, ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION);

	//Back Left
	public static final String BACK_LEFT_CAM_NAME = "Back Left Camera";
	public static final Translation3d ROBOT_TO_BACK_LEFT_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.697), Units.inchesToMeters(11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_BACK_LEFT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(180));
	public static final Transform3d ROBOT_TO_BACK_LEFT_CAMERA = new Transform3d(ROBOT_TO_BACK_LEFT_CAMERA_TRALSLATION, ROBOT_TO_BACK_LEFT_CAMERA_ROTATION);

	//Back Right
	public static final String BACK_RIGHT_CAM_NAME = "Back Right Camera";
	public static final Translation3d ROBOT_TO_BACK_RIGHT_CAMERA_TRALSLATION = new Translation3d(Units.inchesToMeters(-11.697), Units.inchesToMeters(-11.81), Units.inchesToMeters(8.859));
	public static final Rotation3d ROBOT_TO_BACK_RIGHT_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-10), Math.toRadians(180));
	public static final Transform3d ROBOT_TO_BACK_RIGHT_CAMERA = new Transform3d(ROBOT_TO_BACK_RIGHT_CAMERA_TRALSLATION, ROBOT_TO_BACK_RIGHT_CAMERA_ROTATION);

	public static final double CAMERA_DEBOUNCE_TIME = 0.5;

	// TODO: What camera resolutions actually are these? Assuming they're high bc
	// 1080p is high
	// we never even use these why are they here
	public static final CameraResolution BACK_RESOLUTION = CameraResolution.HIGH_RESOLUTION;
	public static final CameraResolution FRONT_RESOLUTION = CameraResolution.HIGH_RESOLUTION;

	public static final int CAMERA_WIDTH = 1200;
	public static final int CAMERA_HEIGHT = 800;
	public static final Rotation2d CAMERA_FOV = Rotation2d.fromDegrees(84.47);
	public static final double CALIB_ERROR_AVG = 2;
	public static final double CALIB_ERROR_STD_DEV = 0.08;
	public static final int CAMERA_FPS = 40;
	public static final int AVG_LATENCY_MS = 40;
	public static final int LATENCY_STD_DEV_MS = 10;
}
