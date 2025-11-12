package frc.robot.Subsystems.GamePieceFinder;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;

public class GamePieceFinder {

    private static AtomicReference<GamePieceFinder> instance;
    
    private Queue<GamePieceParallaxSample> parllaxSamples;
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
        if (instance.get() == null) {
            instance.set(new GamePieceFinder());
        }
        return instance.get();
    }

    private GamePieceFinder() {

    }

    public void addVisionSample(PhotonTrackedTarget sample) {
        visionSamples.add(new GamePieceVisionSample(Milliseconds.of(System.currentTimeMillis()), sample));
    }

    public void addDrivePose(Pose2d pose, Time timestamp) {
        for (GamePieceVisionSample t : visionSamples) {
            if (Math.abs(t.timestamp.in(Milliseconds) - timestamp.in(Milliseconds)) < ALLOWED_TIMESTAMP_DEVIATION.in(Milliseconds)) {
                parllaxSamples.add(new GamePieceParallaxSample(pose, timestamp, t.visionSample));
                visionSamples.remove(t);
                continue;
            }
        }

    }

    public Pose2d getLatestGamepieceEstimate() {
        return estimate;
    }

    private void updateEstimate() {
        // Parallax Calc Here
    }

    public void Periodic() { 
        if (parllaxSamples.size() > 1) {
            updateEstimate();
        }
    }
}
