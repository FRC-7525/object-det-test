// org/team7525/CI/BallDetectionTest.java

package frc.robot.Lib;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import frc.robot.Subsystems.GamePieceFinder.GamePieceFinder;

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

public class BallDetectionTest extends IterativeRobotBase {
    
    private GamePieceFinder finder;
    private int testsPassed = 0;
    private int testsFailed = 0;
    private long startTime;

    // TODO: No magic numbers
    
    public BallDetectionTest() {
        super(0.02);
        HAL.initialize(500, 0);
    }
    
    @Override
    public void startCompetition() {
        try { Thread.sleep(200); } catch (InterruptedException e) {}
        System.out.println("********** Parallax Triangulation Tests **********\n");
        
        finder = GamePieceFinder.getInstance();
        startTime = System.nanoTime();
        
        // Test the actual ray intersection math
        testParallaxGeometry();
        
        printResults();
        System.exit(testsFailed > 0 ? 1 : 0);
    }
    
    private void testParallaxGeometry() {
        System.out.println("=== Testing Parallax Ray Intersection Math ===\n");
        
        // Test Case 1: Ball straight ahead with robot moving sideways
        testCase1_StraightAhead();
        
        // Test Case 2: Ball at an angle
        testCase2_AtAngle();
        
        // Test Case 3: Large baseline (robot moved far between samples)
        testCase3_LargeBaseline();
        
        // Test Case 4: Small angle difference (barely above threshold)
        testCase4_SmallAngle();
    }
    
    private void testCase1_StraightAhead() {
        System.out.println("Test 1: Ball straight ahead, robot moves sideways");
        finder.clearPoseEstimates();
        
        // Ball at (5, 0)
        Translation2d ballPos = new Translation2d(5.0, 0.0);
        
        // Robot at (0, -1) looking at ball
        Pose2d pose1 = new Pose2d(0.0, -1.0, new Rotation2d());
        double yaw1 = calculateYaw(pose1, ballPos);
        
        // Robot at (0, 1) looking at ball
        Pose2d pose2 = new Pose2d(0.0, 1.0, new Rotation2d());
        double yaw2 = calculateYaw(pose2, ballPos);
        
        System.out.printf("  Sample 1: Robot at (%.1f, %.1f), Yaw = %.1f°\n", 
            pose1.getX(), pose1.getY(), yaw1);
        System.out.printf("  Sample 2: Robot at (%.1f, %.1f), Yaw = %.1f°\n", 
            pose2.getX(), pose2.getY(), yaw2);
    
        // Area random bc uh i like dont have a formula for area that works and wouldnt sim that so yeah
        finder.setTestRobotPose(pose1);
        System.out.println("  Some doohickey with adding a pose .");
        finder.addVisionSample(createTarget(yaw1, 5.0));
        System.out.println("  Added first vision sample.");

        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        finder.setTestRobotPose(pose2);
        finder.addVisionSample(createTarget(yaw2, 5.0));
        System.out.println("  Added two vision samples.");

        finder.periodic();
        
        evaluateResult(ballPos, "Test 1");
    }
    
    private void testCase2_AtAngle() {
        System.out.println("\nTest 2: Ball at an angle");
        finder.clearPoseEstimates();
        
        // Ball at (4, 2)
        Translation2d ballPos = new Translation2d(4.0, 2.0);
        
        // Robot at (0, 0) rotated 30°
        Pose2d pose1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(30));
        double yaw1 = calculateYaw(pose1, ballPos);
        
        // Robot at (1, -1) rotated 60°
        Pose2d pose2 = new Pose2d(1.0, -1.0, Rotation2d.fromDegrees(60));
        double yaw2 = calculateYaw(pose2, ballPos);
        
        System.out.printf("  Sample 1: Robot at (%.1f, %.1f) @ %.0f°, Yaw = %.1f°\n", 
            pose1.getX(), pose1.getY(), pose1.getRotation().getDegrees(), yaw1);
        System.out.printf("  Sample 2: Robot at (%.1f, %.1f) @ %.0f°, Yaw = %.1f°\n", 
            pose2.getX(), pose2.getY(), pose2.getRotation().getDegrees(), yaw2);
        
        finder.setTestRobotPose(pose1);
        finder.addVisionSample(createTarget(yaw1, 5.0));
        
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        finder.setTestRobotPose(pose2);
        finder.addVisionSample(createTarget(yaw2, 5.0));
        
        finder.periodic();
        
        evaluateResult(ballPos, "Test 2");
    }
    
    private void testCase3_LargeBaseline() {
        System.out.println("\nTest 3: Large baseline (good triangulation)");
        finder.clearPoseEstimates();
        
        // Ball at (3, 3)
        Translation2d ballPos = new Translation2d(3.0, 3.0);
        
        // Robot far left
        Pose2d pose1 = new Pose2d(-2.0, 0.0, Rotation2d.fromDegrees(45));
        double yaw1 = calculateYaw(pose1, ballPos);
        
        // Robot far right
        Pose2d pose2 = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(135));
        double yaw2 = calculateYaw(pose2, ballPos);
        
        System.out.printf("  Sample 1: Robot at (%.1f, %.1f) @ %.0f°, Yaw = %.1f°\n", 
            pose1.getX(), pose1.getY(), pose1.getRotation().getDegrees(), yaw1);
        System.out.printf("  Sample 2: Robot at (%.1f, %.1f) @ %.0f°, Yaw = %.1f°\n", 
            pose2.getX(), pose2.getY(), pose2.getRotation().getDegrees(), yaw2);
        
        double baseline = pose1.getTranslation().getDistance(pose2.getTranslation());
        System.out.printf("  Baseline distance: %.2fm\n", baseline);
        
        finder.setTestRobotPose(pose1);
        finder.addVisionSample(createTarget(yaw1, 5.0));
        
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        finder.setTestRobotPose(pose2);
        finder.addVisionSample(createTarget(yaw2, 5.0));
        
        finder.periodic();
        
        evaluateResult(ballPos, "Test 3");
    }
    
    private void testCase4_SmallAngle() {
        System.out.println("\nTest 4: Small angle difference (edge case)");
        finder.clearPoseEstimates();
        
        // Ball at (6, 1)
        Translation2d ballPos = new Translation2d(6.0, 1.0);
        
        // Robot moves slightly - should be right at the MIN_YAW_DIFFERENCE threshold
        Pose2d pose1 = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        double yaw1 = calculateYaw(pose1, ballPos);
        
        Pose2d pose2 = new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0));
        double yaw2 = calculateYaw(pose2, ballPos);
        
        System.out.printf("  Sample 1: Yaw = %.1f°\n", yaw1);
        System.out.printf("  Sample 2: Yaw = %.1f°\n", yaw2);
        System.out.printf("  Yaw difference: %.1f° (threshold is 10°)\n", Math.abs(yaw2 - yaw1));
        
        finder.setTestRobotPose(pose1);
        finder.addVisionSample(createTarget(yaw1, 5.0));
        
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        finder.setTestRobotPose(pose2);
        finder.addVisionSample(createTarget(yaw2, 5.0));
        
        finder.periodic();
        
        evaluateResult(ballPos, "Test 4");
    }
    
    // Calculate yaw angle from robot to ball (relative to robot frame)
    private double calculateYaw(Pose2d robotPose, Translation2d ballPos) {
        // Vector from robot to ball in global frame
        Translation2d robotToBall = ballPos.minus(robotPose.getTranslation());
        
        // Global angle to ball
        double globalAngle = Math.atan2(robotToBall.getY(), robotToBall.getX());
        
        // Convert to robot-relative angle
        double robotAngle = robotPose.getRotation().getRadians();
        double yawRad = globalAngle - robotAngle;
        
        // Convert to degrees and normalize to [-180, 180]
        double yawDeg = Math.toDegrees(yawRad);
        while (yawDeg > 180) yawDeg -= 360;
        while (yawDeg < -180) yawDeg += 360;
        
        return yawDeg;
    }
    
    private PhotonTrackedTarget createTarget(double yaw, double area) {
        return new PhotonTrackedTarget(
            // just a ton of random stuff, yaw and area only useful
            yaw,    // yaw
            0,      // pitch (not used)
            area,   // area (dummy value)
            0,      // skew
            -1,     // fiducialId
            -1,     // i dont even know what this is
            1, 
            new Transform3d(),   // bestCameraToTarget
            new Transform3d(),   // altCameraToTarget
            0,      // poseAmbiguity
            new ArrayList<>(),   // minAreaRectCorners
            new ArrayList<>()    // detectedCorners
        );
    }
    
    private void evaluateResult(Translation2d expectedBall, String testName) {
        Pose2d estimate = finder.getLatestGamepieceEstimate();
        
        if (estimate == null) {
            System.out.printf(" %s FAILED: No estimate generated\n", testName);
            System.out.println("(Samples may not have passed MIN_YAW_DIFFERENCE threshold)\n");
            testsFailed++;
            return;
        }
        
        Translation2d estimatedPos = estimate.getTranslation();
        double error = estimatedPos.getDistance(expectedBall);
        
        System.out.printf("  Expected ball at: (%.2f, %.2f)\n", 
            expectedBall.getX(), expectedBall.getY());
        System.out.printf("  Estimated ball at: (%.2f, %.2f)\n", 
            estimatedPos.getX(), estimatedPos.getY());
        System.out.printf("  Error: %.3fm\n", error);
        
        // Very tight tolerance since we're testing pure geometry
        boolean pass = error < 0.1; // 10cm tolerance for pure math
        
        if (pass) {
            System.out.printf("%s PASSED\n\n", testName);
            testsPassed++;
        } else {
            System.out.printf("%s FAILED: Error too large (>0.1m)\n\n", testName);
            testsFailed++;
        }
    }
    
    private void printResults() {
        double elapsed = (System.nanoTime() - startTime) / 1e9;
        System.out.println("========================================");
        System.out.println("Parallax Triangulation Test Results");
        System.out.println("========================================");
        System.out.printf("Tests Passed: %d\n", testsPassed);
        System.out.printf("Tests Failed: %d\n", testsFailed);
        System.out.printf("Time Elapsed: %.2fs\n", elapsed);
        System.out.println("========================================");
        
        if (testsFailed == 0) {
            System.out.println("ALL TESTS PASSED - Parallax math is correct!");
        } else {
            System.out.println("SOME TESTS FAILED - Check ray intersection logic");
        }
    }
    
    @Override
    public void endCompetition() {}
}