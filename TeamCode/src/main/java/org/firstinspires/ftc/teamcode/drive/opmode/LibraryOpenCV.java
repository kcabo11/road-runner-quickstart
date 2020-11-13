package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    public class LibraryOpenCV {
        private OpenCvInternalCamera phoneCam;
        private UltimateGoalDeterminationPipeline pipeline;
        HardwareMap hardwareMap;
        SampleMecanumDrive robot;
        Telemetry telemetry;
        ElapsedTime timer;
        private boolean openCVIsActive = true;
        public static String currentConfig = "";

        public LibraryOpenCV(SampleMecanumDrive newHardwareMap, Telemetry newTelemetry, HardwareMap newHwMap) {

            robot = newHardwareMap;
            telemetry = newTelemetry;
            hardwareMap = newHwMap;
        }

        /**
         * This method starts up the phone light and reads the ring configuration.
         *
         * @return This return function sends back the ring configuration
         */
        public String findRingConfig() {
            long startTime = 0;
            String previousConfig = "";
            String currentPos = "";

            currentPos = currentRingPos();

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("currentConfig", currentPos);
            telemetry.update();

            return currentPos;
        }

        public void initOpenCV() {

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            // Specify the image processing you are using to receive feedback
            pipeline = new UltimateGoalDeterminationPipeline();
            phoneCam.setPipeline(pipeline);
            phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

                @Override
                public void onOpened() {
                    phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }
            });
        }

        public void shutDownOpenCV() {
            phoneCam.stopStreaming();
        }

        public static class UltimateGoalDeterminationPipeline extends OpenCvPipeline {
            /*
             * An enum to define the skystone position
             */
            public enum RingPosition {
                FOUR,
                ONE,
                NONE
            }

            /*
             * Some color constants
             */
            static final Scalar BLUE = new Scalar(0, 0, 255);
            static final Scalar GREEN = new Scalar(0, 255, 0);

            /*
             * The core values which define the location and size of the sample regions
             */
            static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 98);

            static final int REGION_WIDTH = 35;
            static final int REGION_HEIGHT = 25;

            final int FOUR_RING_THRESHOLD = 150;
            final int ONE_RING_THRESHOLD = 135;

            Point region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            Point region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            /*
             * Working variables
             */
            Mat region1_Cb;
            Mat YCrCb = new Mat();
            Mat Cb = new Mat();
            int avg1;

            // Volatile since accessed by OpMode thread w/o synchronization
            private volatile RingPosition position = RingPosition.FOUR;

            /*
             * This function takes the RGB frame, converts to YCrCb,
             * and extracts the Cb channel to the 'Cb' variable
             */
            void inputToCb(Mat input) {
                Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
                Core.extractChannel(YCrCb, Cb, 1);
            }

            public void init(Mat firstFrame) {
                inputToCb(firstFrame);

                region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            }

            public Mat processFrame(Mat input) {
                ElapsedTime timer = new ElapsedTime();
                timer.reset();
                inputToCb(input);

                avg1 = (int) Core.mean(region1_Cb).val[0];

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        2); // Thickness of the rectangle lines

                position = RingPosition.FOUR; // Record our analysis
                if (avg1 > FOUR_RING_THRESHOLD) {
                    position = RingPosition.FOUR;
                } else if (avg1 > ONE_RING_THRESHOLD) {
                    position = RingPosition.ONE;
                } else {
                    position = RingPosition.NONE;
                }

                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

                return input;
            }

            public int getAnalysis() {
                return avg1;
            }
        }

        public String currentRingPos() {
            String currentConfig = "";
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            if (pipeline.position == UltimateGoalDeterminationPipeline.RingPosition.ONE) {
                telemetry.addData("currentConfig = B, ONE", "");
                telemetry.update();
                currentConfig = "ONE";

            } else if (pipeline.position == UltimateGoalDeterminationPipeline.RingPosition.FOUR) {
                telemetry.addData("currentConfig = C, FOUR", "");
                telemetry.update();
                currentConfig = "FOUR";
            } else if (pipeline.position == UltimateGoalDeterminationPipeline.RingPosition.NONE) {
                telemetry.addData("currentConfig = A, NONE", "");
                telemetry.update();
                currentConfig = "NONE";
            }
            return currentConfig;
        }
    }

