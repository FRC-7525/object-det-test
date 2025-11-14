package frc.robot.Subsystems.GamePieceFinder;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Subsystems.Drive.Drive;

public class GamePieceFinder {

    private static final AtomicReference<GamePieceFinder> instance = new AtomicReference<>();

    private final Queue<GamePieceParallaxSample> parallaxSamples = new LinkedList<>();
    private final List<Pose2d> confirmedPieces = new ArrayList<>();

    private final Time SAMPLE_EXPIRATION = Seconds.of(10.0); 

    private final double MIN_YAW_DIFFERENCE_DEG = 10.0;
    private final double AREA_DISTANCE_TOLERANCE = 0.3; // 30% 

    private final Translation2d CAMERA_OFFSET = new Translation2d(0.0, 0.0); 

    private final Rotation2d CAMERA_ROTATION_OFFSET = new Rotation2d(Units.degreesToRadians(27.8));

    private Pose2d latestEstimate;

    private record Ray2d(Translation2d origin, Translation2d dir, double length, double area, Angle measuredAngle, Pose2d robotPose) {}

    private class GamePieceParallaxSample {
        public final Pose2d robotPose;
        public final PhotonTrackedTarget visionSample;
        public final Time timestamp;

        public GamePieceParallaxSample(Pose2d robotPose, Time timestamp, PhotonTrackedTarget visionSample) {
            this.robotPose = robotPose;
            this.visionSample = visionSample;
            this.timestamp = timestamp;
        }

        public Ray2d toRay() { 
            double yawDeg = visionSample.getYaw();
            double distance = estimateDistanceFromArea(visionSample.getArea());

            Translation2d cameraPos = robotPose.transformBy(new Transform2d(CAMERA_OFFSET, CAMERA_ROTATION_OFFSET)).getTranslation();

            double globalAngle = robotPose.getRotation().getRadians() + Math.toRadians(yawDeg);
            Translation2d dir = new Translation2d(Math.cos(globalAngle), Math.sin(globalAngle));

            return new Ray2d(cameraPos, dir, distance, visionSample.getArea(), Degrees.of(yawDeg), robotPose);
        }
    }

    public static GamePieceFinder getInstance() {
        if (instance.get() == null) {
            instance.set(new GamePieceFinder());
        }
        return instance.get();
    }

    private GamePieceFinder() {}

    public void addVisionSample(PhotonTrackedTarget sample) {
        parallaxSamples.add(new GamePieceParallaxSample(Drive.getInstance().getPose(), Milliseconds.of(System.currentTimeMillis()), sample));

    }

    public List<Pose2d> getDetectedPiecesSortedByDistance(Pose2d robotPose) {
        return confirmedPieces.stream()
                .sorted(Comparator.comparingDouble(p ->
                        p.getTranslation().getDistance(robotPose.getTranslation())))
                .toList();
    }

    public Pose2d getLatestGamepieceEstimate() {
        return latestEstimate;
    }

    public void periodic() {
        cleanupOldSamples();
        if (parallaxSamples.size() > 1) updateEstimates();
    }

    public void clearPoseEstimates() {
        confirmedPieces.clear();
        latestEstimate = null;
    }


    private void updateEstimates() {
        List<GamePieceParallaxSample> samples = new ArrayList<>(parallaxSamples);
        Set<GamePieceParallaxSample> toRemove = new HashSet<>();

        for (int i = 0; i < samples.size(); i++) {
            for (int j = i + 1; j < samples.size(); j++) {
                //I need the "first" sample to have the smaller rotation so that I can set it as 0 and go off of that for calculations
                //TODO: Logic will mess up if rotation can be negative, so need to confirm that its not/deal with it if it is
                //TODO: Probably better way to implement this logic lol
                var s1 = samples.get(i).robotPose.getRotation().getDegrees() < samples.get(j).robotPose.getRotation().getDegrees() ? samples.get(i) : samples.get(j);
                var s2 = samples.get(i).robotPose.getRotation().getDegrees() > samples.get(j).robotPose.getRotation().getDegrees() ? samples.get(i) : samples.get(j);

                double yawDiff = Math.abs(s2.visionSample.getYaw() - s1.visionSample.getYaw());
                if (yawDiff < MIN_YAW_DIFFERENCE_DEG) continue;

                Ray2d r1 = s1.toRay();
                Ray2d r2 = s2.toRay();

                Translation2d objectPoint = getPoseThroughParallax(r1, r2);
                // Chat will it continue if the first condition isnt met and then leave my thingy that might get a null pointer alone
                if (areasAtIntersection(r1, r2, objectPoint)) {

                    Pose2d found = new Pose2d(objectPoint, new Rotation2d());
                    confirmedPieces.add(found);
                    latestEstimate = found;
                    toRemove.add(s1);
                    toRemove.add(s2);
                }
            }
        }
        parallaxSamples.removeAll(toRemove);
    }

    private void cleanupOldSamples() {
        double now = System.currentTimeMillis();
        parallaxSamples.removeIf(s ->
                now - s.timestamp.in(Milliseconds) > SAMPLE_EXPIRATION.in(Milliseconds));
    }

    // Yeah no idea what this relationship is gona be, linear, sqrt under, 6 or 7
    private double estimateDistanceFromArea(double area) {
        double k = 6.7; 
        return k / Math.sqrt(Math.max(area, 0.2));
    }

    private boolean areasAtIntersection(Ray2d r1, Ray2d r2, Translation2d intersection) {
        double distAlongR1 = intersection.minus(r1.origin()).getNorm();
        double distAlongR2 = intersection.minus(r2.origin()).getNorm();
    
        double predictedR1 = r1.length();
        double predictedR2 = r2.length();
        
        boolean ok1 = Math.abs(distAlongR1 - predictedR1) / predictedR1 < AREA_DISTANCE_TOLERANCE;
        boolean ok2 = Math.abs(distAlongR2 - predictedR2) / predictedR2 < AREA_DISTANCE_TOLERANCE;
    
        return ok1 && ok2;
    }

    // Silly vector intersection solution, idrk if this is optimal or works @william feel free to replace with your calc
    private Optional<Translation2d> intersectRays(Ray2d r1, Ray2d r2) {
        double x1 = r1.origin().getX(), y1 = r1.origin().getY();
        double x2 = x1 + r1.dir().getX(), y2 = y1 + r1.dir().getY();
        double x3 = r2.origin().getX(), y3 = r2.origin().getY();
        double x4 = x3 + r2.dir().getX(), y4 = y3 + r2.dir().getY();

        double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denominator) < 1e-2) return Optional.empty(); // if theyre parallel then throw ts out

        double px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denominator;
        double py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denominator;

        Translation2d p = new Translation2d(px, py);

        // Ignore balls that are fall away
        if (p.minus(r1.origin()).getNorm() > r1.length() * 1.5 ||
            p.minus(r2.origin()).getNorm() > r2.length() * 1.5)
            return Optional.empty();

        return Optional.of(p);
    }

    //TODO: Wait what if the bot has moved its center between samples? Is that gonna be a problem
    private Translation2d getPoseThroughParallax(Ray2d r1, Ray2d r2) {
        Angle deltaYaw = Degrees.of(r2.robotPose().getRotation().minus(r1.robotPose().getRotation()).getDegrees());

        Translation2d firstCameraPos = ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION.toTranslation2d();
        //TODO: Need to check if this rotates the right way - should be to the left
        Translation2d secondCameraPos = firstCameraPos.rotateAround(Translation2d.kZero, Rotation2d.fromDegrees(deltaYaw.in(Degrees)));

        Translation2d vecBtwnCameraPos = firstCameraPos.minus(secondCameraPos);
        
        Angle cameraRot = ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION.getMeasureZ();
        Angle angleOffset = Radians.of(Math.atan2(firstCameraPos.getY(), firstCameraPos.getX()));
        Angle firstYaw = r1.measuredAngle;
        Angle secondYaw = r2.measuredAngle;

        //TODO: There's some redudant math here, need to come back and simplify/clean it up later
        Angle alpha = Radians.of(Math.atan2(vecBtwnCameraPos.getY(), vecBtwnCameraPos.getX()));
        Angle beta = Degrees.of(90 - alpha.in(Degrees));
        Angle A = Degrees.of(cameraRot.in(Degrees) - firstYaw.in(Degrees) - alpha.in(Degrees));
        Angle B = Degrees.of(360 - (cameraRot.in(Degrees) - secondYaw.in(Degrees) + angleOffset.in(Degrees) + ((180 - deltaYaw.in(Degrees))/2 - beta.in(Degrees))));
        Angle C = Degrees.of(180 - A.in(Degrees) - B.in(Degrees));

        //TODO: Need to make sure coordinate system matches (im assuming right is positive and up is positive)
        Distance b = Meters.of((vecBtwnCameraPos.getDistance(Translation2d.kZero)/Math.sin(C.in(Radians)))*Math.sin(A.in(Radians)));
        Translation2d robotToObject = new Translation2d(
            -b.in(Meters)*Math.cos(A.in(Radians) + alpha.in(Radians)) + firstCameraPos.getMeasureX().in(Meters),
            b.in(Meters)*Math.sin(A.in(Radians) + alpha.in(Radians)) + firstCameraPos.getMeasureY().in(Meters)
        );
        
        //I did the above calculations assuming the robot was facing straight forward, so now I need to rotate the point to match the rotation of the robot
        //I could change it to work for any initial direction of the bot in my calculations but whatever
        //TODO: I assume it's gonna rotate in the right direction?
        robotToObject = robotToObject.rotateAround(Translation2d.kZero, r1.robotPose.getRotation());

        return r1.robotPose.getTranslation().plus(robotToObject);
    }
}
