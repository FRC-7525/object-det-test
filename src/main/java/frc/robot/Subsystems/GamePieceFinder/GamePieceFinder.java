package frc.robot.Subsystems.GamePieceFinder;

import static edu.wpi.first.units.Units.*;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
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

    private Pose2d latestEstimate;

    private record Ray2d(Translation2d origin, Translation2d dir, double length, double area) {}

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

            Translation2d cameraPos = robotPose.transformBy(new Transform2d(CAMERA_OFFSET, new Rotation2d())).getTranslation();

            // Wait lwky is the getYaw function in deg
            double globalAngle = robotPose.getRotation().getRadians() + Math.toRadians(yawDeg);
            Translation2d dir = new Translation2d(Math.cos(globalAngle), Math.sin(globalAngle));

            return new Ray2d(cameraPos, dir, distance, visionSample.getArea());
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


    private void updateEstimates() {
        List<GamePieceParallaxSample> samples = new ArrayList<>(parallaxSamples);
        Set<GamePieceParallaxSample> toRemove = new HashSet<>();

        for (int i = 0; i < samples.size(); i++) {
            for (int j = i + 1; j < samples.size(); j++) {
                var s1 = samples.get(i);
                var s2 = samples.get(j);

                double yawDiff = Math.abs(s1.visionSample.getYaw() - s2.visionSample.getYaw());
                if (yawDiff < MIN_YAW_DIFFERENCE_DEG) continue;

                Ray2d r1 = s1.toRay();
                Ray2d r2 = s2.toRay();

                Optional<Translation2d> intersection = intersectRays(r1, r2);
                // Chat will it continue if the first condition isnt met and then leave my thingy that might get a null pointer alone
                if (intersection.isPresent() &&
                   areasAtIntersection(r1, r2, intersection.get())) {

                    Pose2d found = new Pose2d(intersection.get(), new Rotation2d());
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
}
