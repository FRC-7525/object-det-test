package frc.robot.Subsystems.GamePieceFinder;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION;
import static frc.robot.Subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION;

import java.util.Deque;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Subsystems.Drive.Drive;

public class GamePieceFinder {

    private static GamePieceFinder instance = new GamePieceFinder();
    
    private Queue<GamePieceParallaxSample> parallaxSamples = new LinkedList<>();
    private Queue<GamePieceVisionSample> visionSamples;

    private Pose2d estimate;

    private final Time ALLOWED_TIMESTAMP_DEVIATION = Seconds.of(0.2);

    private class GamePieceParallaxSample {
        public Pose2d robotPose;
        public PhotonTrackedTarget visionSample;
        public Time timestamp;

        public GamePieceParallaxSample(Pose2d robotPose, Time timestamp, PhotonTrackedTarget visionSample) {
            this.robotPose = robotPose;
            this.visionSample = visionSample;
            this.timestamp = timestamp;
        }
    }

    private class GamePieceVisionSample {
        public PhotonTrackedTarget visionSample;
        public Time timestamp;

        public GamePieceVisionSample(Time timestamp, PhotonTrackedTarget visionSample) {
            this.visionSample = visionSample;
            this.timestamp = timestamp;
        }
    }

    public static GamePieceFinder getInstance() {
        return instance;
    }

    private GamePieceFinder() {

    }

    public void addVisionSample(PhotonTrackedTarget sample) {
        parallaxSamples.add(new GamePieceParallaxSample(Drive.getInstance().getPose(), Milliseconds.of(System.currentTimeMillis()), sample));
    }

    public Pose2d getLatestGamepieceEstimate() {
        return estimate;
    }

    private void updateEstimate() {
        // Parallax Calc Here
        GamePieceParallaxSample firstTempSample = parallaxSamples.poll(); //TODO: Am I good to remove the samples here?
        GamePieceParallaxSample secondTempSample = parallaxSamples.poll();

        
//I need the "first" sample to have the smaller rotation so that I can set it as 0 and go off of that for calculations
        //TODO: Logic will mess up if rotation can be negative
        //TODO: Probably better way to implement this logic lol
        GamePieceParallaxSample firstSample = firstTempSample.robotPose.getRotation().getDegrees() < secondTempSample.robotPose.getRotation().getDegrees() ? firstTempSample : secondTempSample;
        GamePieceParallaxSample secondSample = firstTempSample.robotPose.getRotation().getDegrees() > secondTempSample.robotPose.getRotation().getDegrees() ? firstTempSample : secondTempSample;
        //TODO: Need to make sure there are no weird singularities or smth happening here
        Angle deltaYaw = Degrees.of(Math.abs(secondSample.robotPose.getRotation().minus(firstSample.robotPose.getRotation()).getDegrees()));
        Translation2d robotToRobot = secondSample.robotPose.minus(firstSample.robotPose).getTranslation();

        Logger.recordOutput("PSSTUFF/deltaYaw", deltaYaw.in(Degrees));

        Translation2d firstCameraPos = ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION.toTranslation2d();
        //TODO: Need to check if this rotates the right way - should be to the left
        Translation2d secondCameraPos = firstCameraPos.rotateAround(Translation2d.kZero, Rotation2d.fromDegrees(deltaYaw.in(Degrees))).plus(robotToRobot);
        
        Angle cameraRot = ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION.getMeasureZ();
        Angle firstYaw = Degrees.of(-firstSample.visionSample.yaw);
        Angle secondYaw = Degrees.of(-secondSample.visionSample.yaw);

        double posX = -(Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians))*firstCameraPos.getX()+firstCameraPos.getY() - Math.tan(cameraRot.in(Radians) - secondYaw.in(Radians) - deltaYaw.in(Radians))*secondCameraPos.getX() - secondCameraPos.getY())/(-Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians)) + Math.tan(cameraRot.in(Radians) - secondYaw.in(Radians) - deltaYaw.in(Radians)));
        double posY = -Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians))*posX+Math.tan(cameraRot.in(Radians)-firstYaw.in(Radians))*firstCameraPos.getX()+firstCameraPos.getY();

        Translation2d robotToObject = new Translation2d(posX, posY);    

        //This should cancel the rotation of the bot and make the estimated pose of the object have 0 rotation
        estimate = firstSample.robotPose.plus(new Transform2d(robotToObject, new Rotation2d()));
        Logger.recordOutput("PSSTUFF/estimate", estimate);
    }

    public void unitTestEstimate(double robotRot, double yaw1, double yaw2) {
        //TODO: Need to make sure there are no weird singularities or smth happening here
        Angle deltaYaw = Degrees.of(robotRot);

        Translation2d firstCameraPos = ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION.toTranslation2d();
        //TODO: Need to check if this rotates the right way - should be to the left
        Translation2d secondCameraPos = firstCameraPos.rotateAround(Translation2d.kZero, Rotation2d.fromDegrees(deltaYaw.in(Degrees)));

        Translation2d vecBtwnCameraPos = secondCameraPos.minus(firstCameraPos);

        Angle cameraRot = ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION.getMeasureZ();
        Angle angleOffset = Radians.of(Math.atan(firstCameraPos.getY() / firstCameraPos.getX()));
        Angle firstYaw = Degrees.of(-yaw1);
        Angle secondYaw = Degrees.of(-yaw2);

        //TODO: There's some redundant math here, need to come back and simplify/clean it up later
        Angle alpha = Radians.of(Math.atan(vecBtwnCameraPos.getY() / vecBtwnCameraPos.getX()));

        Angle beta = Degrees.of(90 + alpha.in(Degrees));
        Angle A = Degrees.of(cameraRot.in(Degrees) - firstYaw.in(Degrees) + alpha.in(Degrees));
        Angle B = Degrees.of(360 - (cameraRot.in(Degrees) - secondYaw.in(Degrees) + angleOffset.in(Degrees) + ((180 - deltaYaw.in(Degrees))/2 - beta.in(Degrees)) + beta.in(Degrees)));
        Angle C = Degrees.of(180 - A.in(Degrees) - B.in(Degrees));

        //TODO: Need to make sure coordinate system matches (im assuming right is positive and up is positive)
        Distance b = Meters.of((vecBtwnCameraPos.getDistance(Translation2d.kZero)/Math.sin(C.in(Radians)))*Math.sin(B.in(Radians)));

        Translation2d robotToObject = new Translation2d(
            -(b.in(Meters)*Math.cos(A.in(Radians) - alpha.in(Radians)) + firstCameraPos.getMeasureX().in(Meters)),
            b.in(Meters)*Math.sin(A.in(Radians) - alpha.in(Radians)) - firstCameraPos.getMeasureY().in(Meters)
        );

        Logger.recordOutput("PSSTUFF/estimated translation", robotToObject);
    }

    public void unitTestEstimateOtherMethod(Pose2d pos1, Pose2d pos2, double yaw1, double yaw2) {
        Translation2d robotToRobot = pos2.minus(pos1).getTranslation();

        Rotation2d greaterRot = pos1.getRotation().getDegrees() > pos2.getRotation().getDegrees() ? pos1.getRotation() : pos2.getRotation();
        Rotation2d lesserRot = pos1.getRotation().getDegrees() > pos2.getRotation().getDegrees() ? pos2.getRotation() : pos1.getRotation();
        //TODO: Need to make sure there are no weird singularities or smth happening here
        Angle deltaYaw = Degrees.of(greaterRot.minus(lesserRot).getDegrees());

        System.out.println(greaterRot);

        Translation2d firstCameraPos = ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION.toTranslation2d();
        //TODO: Need to check if this rotates the right way - should be to the left
        Translation2d secondCameraPos = firstCameraPos.rotateAround(Translation2d.kZero, Rotation2d.fromDegrees(deltaYaw.in(Degrees))).plus(robotToRobot);

        Angle cameraRot = ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION.getMeasureZ();
        Angle firstYaw = Degrees.of(-yaw1);
        Angle secondYaw = Degrees.of(-yaw2);

        double posX = -(Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians))*firstCameraPos.getX()+firstCameraPos.getY() - Math.tan(cameraRot.in(Radians) - secondYaw.in(Radians) - deltaYaw.in(Radians))*secondCameraPos.getX() - secondCameraPos.getY())/(-Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians)) + Math.tan(cameraRot.in(Radians) - secondYaw.in(Radians) - deltaYaw.in(Radians)));
        double posY = -Math.tan(cameraRot.in(Radians) - firstYaw.in(Radians))*posX+Math.tan(cameraRot.in(Radians)-firstYaw.in(Radians))*firstCameraPos.getX()+firstCameraPos.getY();

        Translation2d robotToObject = new Translation2d(posX, posY);

        System.out.println(robotToObject);

        Logger.recordOutput("PSSTUFF/estimated translation", robotToObject);
    }

    public void Periodic() {
        if (parallaxSamples != null) {
            Logger.recordOutput("PSSTUFF/Queue_Length", parallaxSamples.size());
            int index = 0;
            for (var x : parallaxSamples.stream().toArray()) {
                Logger.recordOutput("PSSTUFF/Pose" + index, ((GamePieceParallaxSample) x).robotPose);
                Logger.recordOutput("PSSTUFF/Yaw" + index, ((GamePieceParallaxSample) x).visionSample.yaw);
                index++;
            }
        }
        if (DRIVER_CONTROLLER.getBButtonPressed()) {
            updateEstimate();
        }
    }
}
