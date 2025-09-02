/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.vision;

import java.util.Comparator;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frclib.drivebase.FrcRobotDrive;
import frclib.driverio.FrcDashboard;
import frclib.vision.FrcPhotonVision;
import teamcode.RobotParams;
import teamcode.subsystems.LEDIndicator;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcEvent;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVision extends FrcPhotonVision
{
    /**
     * This class contains the parameters of the front camera.
     */
    public static class HD3000CamParams extends FrcRobotDrive.VisionInfo
    {
        public HD3000CamParams()
        {
            camName = "HD-3000";
            camImageWidth = 1280;
            camImageHeight = 720;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0.0;                   // Inches forward from robot center
            camZOffset = 0.0;                   // Inches up from the floor
            camPitch = 0.0;                     // degrees up from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //HD3000CamParams
    }   //class HD3000CamParams

    public static class OV9782CamParams extends FrcRobotDrive.VisionInfo
    {
        public OV9782CamParams()
        {
            camName = "FrontOV9782";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = 0.0;                   // Inches to the right from robot center
            camYOffset = 0.0;                   // Inches forward from robot center
            camZOffset = 0.0;                   // Inches up from the floor
            camYaw = 0.0;                       // degrees clockwise from robot front
            camPitch = 0.0;                     // degrees up from horizontal
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //OV9782CamParams
    }   //class OV9782CamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class OV9281CamParams extends FrcRobotDrive.VisionInfo
    {
        public OV9281CamParams()
        {
            camName = "OV9281";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = -3.5;                  // Inches to the right from robot center
            camYOffset = -2.375;                // Inches forward from robot center
            camZOffset = 23.125;                // Inches up from the floor
            camPitch = 33.0;                    // degrees up from horizontal
            camYaw = 0.0;                       // degrees clockwise from robot front
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //OV9281CamParams
    }   //class OV9281CamParams

    private static final String DBKEY_PREFIX                = "Vision/";
    public static final double ONTARGET_THRESHOLD           = 0.5;
    public static final double GUIDANCE_ERROR_THRESHOLD     = 12.0;

    public enum PipelineType
    {
        APRILTAG(0),
        COLOR_BLOB(1);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

    }   //enum PipelineType

    private final FrcDashboard dashboard;
    private final String instanceName;
    private final Transform3d robotToCam;
    private final LEDIndicator ledIndicator;
    private PipelineType currPipeline = PipelineType.APRILTAG;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCam specifies the 3D transform location of the camera from robot center.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     */
    public PhotonVision(String cameraName, Transform3d robotToCam, LEDIndicator ledIndicator)
    {
        super(cameraName, robotToCam);
        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFIX + cameraName, "");
        this.instanceName = cameraName;
        this.robotToCam = robotToCam;
        this.ledIndicator = ledIndicator;
        setPipeline(currPipeline);
    }   //PhotonVision

    /**
     * This method returns the transform between two adjacent AprilTags.
     *
     * @param fromAprilTagId specifies the From AprilTag ID.
     * @param toAprilTagId specifies the To AprilTag ID.
     * @return transform between two adjacent AprilTags.
     */
    public Transform3d getMultiTagTransform(int fromAprilTagId, int toAprilTagId)
    {
        return FrcPhotonVision.getAprilTagFieldPose3d(toAprilTagId, null).minus(
               FrcPhotonVision.getAprilTagFieldPose3d(fromAprilTagId, null));
    }   //getMultiTagTransform

    /**
     * This method returns the robot's field position.
     *
     * @param aprilTagObj specifies the detected AprilTag object.
     * @param usePoseEstimator specifies true to use PhotonVision Lib pose estimator, false to use the AprilTag field
     *        pose to calculate it ourselves.
     * @return robot's field location.
     */
    public TrcPose2D getRobotFieldPose(DetectedObject aprilTagObj, boolean usePoseEstimator)
    {
        return usePoseEstimator? getRobotEstimatedPose(robotToCam):
                                 getRobotPoseFromAprilTagFieldPose(
                                    FrcPhotonVision.getAprilTagFieldPose3d(aprilTagObj.target.getFiducialId(), null),
                                    aprilTagObj.target.getBestCameraToTarget(),
                                    robotToCam);
    }   //getRobotFieldPose

    /**
     * This method returns the best detected object.
     *
     * @param comparator specifies comparator for sorting the detected objects, can be null if not provided.
     * @param detectionEvent specifies the event to signal when it detects the target.
     * @return best detected object.
     */
    public DetectedObject getBestDetectedObject(
        Comparator<? super PhotonTrackedTarget> comparator, TrcEvent detectionEvent)
    {
        DetectedObject bestDetectedObj = super.getBestDetectedObject(comparator);

        if (bestDetectedObj != null)
        {
            if (detectionEvent != null)
            {
                detectionEvent.signal();
            }

            if (ledIndicator != null)
            {
                ledIndicator.setPhotonDetectedObject(getPipeline(), bestDetectedObj.targetPose);
            }
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method returns the best detected object.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getBestDetectedObject(Comparator<? super PhotonTrackedTarget> comparator)
    {
        return getBestDetectedObject(comparator, null);
    }   //getBestDetectedObject

    /**
     * This method is called by the comparator to sort the detected object array in descending area of the target.
     *
     * @param t1 specifies the target 1 object.
     * @param t2 specifies the target 2 object.
     * @return positive value if target 2 area is greater than target 1, negative value if target 2 area is smaller
     *         than target 1, zero if areas are equal.
     */
    private int compareAreas(PhotonTrackedTarget t1, PhotonTrackedTarget t2)
    {
        return (int)((t2.getArea() - t1.getArea())*100);
    }   //compareArea

    /**
     * This method get the best detected AprilTag matching the specified AprilTag IDs array sorted by most preferred
     * ID at the top.
     *
     * @param detectionEvent specifies the event to signal when it detects the target.
     * @param aprilTagIds specifies the AprilTag IDs to look for, null to look for any.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(TrcEvent detectionEvent, int... aprilTagIds)
    {
        DetectedObject bestObj = null;

        if (currPipeline == PipelineType.APRILTAG)
        {
            bestObj = super.getDetectedAprilTag(this::compareAreas, aprilTagIds);
        }

        if (bestObj != null)
        {
            if (detectionEvent != null)
            {
                detectionEvent.signal();
            }

            if  (ledIndicator != null)
            {
                ledIndicator.setPhotonDetectedObject(currPipeline, bestObj.targetPose);
            }
        }

        return bestObj;
    }   //getBestDetectedAprilTag

    /**
     * This method get the best detected AprilTag matching the specified AprilTag IDs array sorted by most preferred
     * ID at the top.
     *
     * @param aprilTagIds specifies the AprilTag IDs to look for, null to look for any.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(int... aprilTagIds)
    {
        return getBestDetectedAprilTag((TrcEvent) null, aprilTagIds);
    }   //getBestDetectedAprilTag

    /**
     * This method sets the active pipeline type used in the LimeLight.
     *
     * @param pipelineType specifies the pipeline to activate in the LimeLight.
     */
    public void setPipeline(PipelineType pipelineType)
    {
        if (pipelineType != currPipeline)
        {
            currPipeline = pipelineType;
            super.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setPipeline

    /**
     * This method returns the active pipeline of the LimeLight.
     *
     * @return active pipeline.
     */
    public PipelineType getPipeline()
    {
        currPipeline = PipelineType.values()[getPipelineIndex()];
        return currPipeline;
    }   //getPipeline

    public static TrcPose2D getClosestAprilTagPose(TrcPose2D robotPose)
    {
        TrcPose2D closestAprilTagPose = null;
        double minDistance = Double.MAX_VALUE;

        for (TrcPose2D aprilTagPose: RobotParams.Game.APRILTAG_POSES)
        {
            double distance = robotPose.distanceTo(aprilTagPose);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestAprilTagPose = aprilTagPose;
            }
        }

        return closestAprilTagPose.clone();
    }   //getClosestAprilTagPose

    //
    // Implements FrcPhotonVision abstract methods.
    //

    /**
     * This method returns the ground offset of the detected target.
     *
     * @return target ground offset.
     */
    public double getTargetGroundOffset(PhotonTrackedTarget target)
    {
        double targetHeight = 0.0;
        PipelineType pipelineType = getPipeline();

        switch (pipelineType)
        {
            case APRILTAG:
                if (target != null)
                {
                    // Even though PhotonVision said detected target, FieldLayout may not give us AprilTagPose.
                    // Check it before access the AprilTag pose.
                    Pose3d aprilTagPose = getAprilTagFieldPose3d(target.getFiducialId(), null);
                    if (aprilTagPose != null)
                    {
                        targetHeight = aprilTagPose.getZ();
                    }
                }
                break;

            default:
                targetHeight = 0.0;
                break;
        }

        return targetHeight;
    }   //getTargetGroundOffset

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum)
    {
        PipelineType pipelineType;
        FrcPhotonVision.DetectedObject object = getBestDetectedObject(this::compareAreas);

        if (object != null)
        {
            pipelineType = getPipeline();
            if (pipelineType == PipelineType.APRILTAG)
            {
                dashboard.putString(
                    DBKEY_PREFIX + instanceName,
                    String.format(
                        "%s[%d]:targetPose=%s,robotPose=%s",
                        pipelineType, object.target.getFiducialId(), object.targetPose, object.robotPose));
            }
            else
            {
                dashboard.putString(
                    DBKEY_PREFIX + instanceName,
                    String.format(
                        "%s:targetPose=%s,robotPose=%s",
                        pipelineType, object.targetPose, object.robotPose));
            }
        }
        return lineNum;
    }   //updateStatus

}   //class PhotonVision
