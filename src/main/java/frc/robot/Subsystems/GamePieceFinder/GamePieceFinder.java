package frc.robot.Subsystems.GamePieceFinder;

import static edu.wpi.first.units.Units.*;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.Subsystems.Drive.Drive;

public class GamePieceFinder {

    private static final AtomicReference<GamePieceFinder> instance = new AtomicReference<>();

    private final Deque<GamePieceParallaxSample> parallaxSamples = new LinkedList<>();
    private final List<Pose2d> confirmedPieces = new ArrayList<>();
    private Pose2d testRobotPose = null;
    private final Time SAMPLE_EXPIRATION = Seconds.of(10.0); 

    private final double MIN_YAW_DIFFERENCE_DEG = 10.0;
    private final double AREA_DISTANCE_TOLERANCE = 0.3; // 30% 

    private final Translation2d CAMERA_OFFSET = new Translation2d(Units.inchesToMeters(11.809459), Units.inchesToMeters(-11.164206)); 

    private final Rotation2d CAMERA_ROTATION_OFFSET = new Rotation2d(Units.degreesToRadians(27.8));

    private Pose2d latestEstimate;

    public record Ray2d(Translation2d origin, Translation2d dir, double length, double area, Angle measuredAngle, Pose2d robotPose) {}

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
            
            Transform2d robotToCamera = new Transform2d(CAMERA_OFFSET, CAMERA_ROTATION_OFFSET);
            Translation2d cameraPos = robotPose.transformBy(robotToCamera).getTranslation();
            
            double globalAngleRad = robotPose.getRotation().getRadians() 
                                  + CAMERA_ROTATION_OFFSET.getRadians() 
                                  + Math.toRadians(yawDeg);
            
            Translation2d dir = new Translation2d(
                Math.cos(globalAngleRad), 
                Math.sin(globalAngleRad)
            );
            
            return new Ray2d(cameraPos, dir, distance, visionSample.getArea(), 
                             Degrees.of(yawDeg), robotPose);
        }
    }

    public static GamePieceFinder getInstance() {
        if (instance.get() == null) {
            instance.set(new GamePieceFinder());
        }
        return instance.get();
    }

    private GamePieceFinder() {}

    // TODO: Check env name and throw an exception if youre not in CI mode
    public void setTestRobotPose(Pose2d pose) {
        this.testRobotPose = pose;
    }

    public void addVisionSample(PhotonTrackedTarget sample) {
        // Order of if statements matters here
        if (parallaxSamples.size() > 0 && Math.abs(sample.getYaw() - parallaxSamples.peekLast().visionSample.getYaw())< MIN_YAW_DIFFERENCE_DEG) {
            return; 
        }

        // Pose2d robotPose = (testRobotPose != null) 
        //     ? testRobotPose 
        //     : Drive.getInstance().getPose();
        Pose2d robotPose = Drive.getInstance().getPose();

        parallaxSamples.add(new GamePieceParallaxSample(robotPose, Milliseconds.of(System.currentTimeMillis()), sample));

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
        Logger.recordOutput("GamePieceFinder/ConfirmedPieces", confirmedPieces.size());
        Logger.recordOutput("GamePieceFinder/Pose", getLatestGamepieceEstimate());
        Logger.recordOutput("GamePiceceFinder/VisionSamples", parallaxSamples.size());
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
                var s1 = samples.get(i);
                var s2 = samples.get(j);

                double yawDiff = Math.abs(s2.visionSample.getYaw() - s1.visionSample.getYaw());
                if (yawDiff < MIN_YAW_DIFFERENCE_DEG) continue;

                Ray2d r1 = s1.toRay();
                Ray2d r2 = s2.toRay();

                Translation2d objectPoint = getPoseThroughParallax(r1, r2);
                debugRays(r1, r2, objectPoint); 
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
        // TODO: Fx ts
        return true;
    }

    private Translation2d getPoseThroughParallax(Ray2d r1, Ray2d r2) {
        // Use simple 2D ray intersection instead of complex angle math
        Optional<Translation2d> intersection = intersectRays(r1, r2);
        
        if (intersection.isEmpty()) {
            // for parallel thngs
            return r1.origin().plus(r1.dir().times(r1.length()));
        }
        
        return intersection.get();
    }

    private Optional<Translation2d> intersectRays(Ray2d r1, Ray2d r2) {
        double x1 = r1.origin().getX();
        double y1 = r1.origin().getY();
        double dx1 = r1.dir().getX();
        double dy1 = r1.dir().getY();
        
        double x2 = r2.origin().getX();
        double y2 = r2.origin().getY();
        double dx2 = r2.dir().getX();
        double dy2 = r2.dir().getY();
        
        // Check if rays are parallel
        double denominator = dx1 * dy2 - dy1 * dx2;
        if (Math.abs(denominator) < 1e-6) {
            return Optional.empty();
        }
        
        // Calculate parameter t1 along ray1
        double t1 = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / denominator;
        
        // Calculate intersection point
        Translation2d intersection = new Translation2d(
            x1 + t1 * dx1,
            y1 + t1 * dy1
        );
        
        // Basic sanity checks that don't depend on distance estimation:
        
        // 1. Intersection must be in front of both cameras (t > 0)
        if (t1 < 0) {
            return Optional.empty();
        }
        
        // 2. Calculate t2 to verify it's also positive
        double t2 = ((x1 - x2) * dy1 - (y1 - y2) * dx1) / (-denominator);
        if (t2 < 0) {
            return Optional.empty();
        }
        
        // 3. Reasonable distance check - intersection should be within ~10m
        //    (prevents crazy far-away intersections from numerical issues)
        double dist1 = intersection.minus(r1.origin()).getNorm();
        double dist2 = intersection.minus(r2.origin()).getNorm();
        
        if (dist1 > 10.0 || dist2 > 10.0) {
            return Optional.empty();
        }
        
        // TODO: Once distance formula is calibrated, re-enable this check:
        // if (dist1 > r1.length() * 2.0 || dist2 > r2.length() * 2.0) {
        //     return Optional.empty();
        // }
        
        return Optional.of(intersection);
    }

    private void debugRays(Ray2d r1, Ray2d r2, Translation2d intersection) {
        // Log ray origins
        Logger.recordOutput("Parallax/Ray1Origin", new Pose2d(r1.origin(), r1.());
        Logger.recordOutput("Parallax/Ray2Origin", new Pose2d(r2.origin(), new Rotation2d()));
        
        // Log ray endpoints (origin + direction * length)
        Translation2d r1End = r1.origin().plus(r1.dir().times(r1.length()));
        Translation2d r2End = r2.origin().plus(r2.dir().times(r2.length()));
        Logger.recordOutput("Parallax/Ray1End", new Pose2d(r1End, new Rotation2d()));
        Logger.recordOutput("Parallax/Ray2End", new Pose2d(r2End, new Rotation2d()));
        
        // Log intersection
        Logger.recordOutput("Parallax/Intersection", new Pose2d(intersection, new Rotation2d()));
        
        // Log angles and distances
        Logger.recordOutput("Parallax/Ray1Length", r1.length());
        Logger.recordOutput("Parallax/Ray2Length", r2.length());
        Logger.recordOutput("Parallax/ActualDist1", intersection.minus(r1.origin()).getNorm());
        Logger.recordOutput("Parallax/ActualDist2", intersection.minus(r2.origin()).getNorm());
    }
}
