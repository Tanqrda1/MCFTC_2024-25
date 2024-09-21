package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
public class ArnavOpMode extends LinearOpMode {

    //private DistanceSensor sensorColorRange;

    private DcMotor LadderLift;
    private DcMotor Hook;
    private DcMotor Frontright;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Backright;

    private Servo Test;
    private Servo Drone;
    private Servo Servoarm;

    private Limelight3A limelight;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    ElapsedTime timer = new ElapsedTime();

    public void init_motors() {
        Test = hardwareMap.get(Servo.class, "Test");
        Servoarm = hardwareMap.get(Servo.class, "Servoarm");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        Hook.setDirection(DcMotorSimple.Direction.REVERSE);
        Hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift = hardwareMap.get(DcMotor.class, "LadderLift");
        LadderLift.setDirection(DcMotorSimple.Direction.REVERSE);
        LadderLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", Frontleft.getDirection());
        Frontright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", Frontright.getDirection());
        Backright.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", Backright.getDirection());
        Backleft.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", Backleft.getDirection());
        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", Frontleft.getCurrentPosition());
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", Frontright.getCurrentPosition());
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", Backleft.getCurrentPosition());
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBR:", Backleft.getCurrentPosition());
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }

    public void Mecanumdrive() {
        Frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        Backright.setDirection(DcMotorSimple.Direction.REVERSE);
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        Frontleft.setPower(v1);
        Frontright.setPower(v2);
        Backleft.setPower(v3);
        Backright.setPower(v4);
    }

    public void Hook_Zero_Pwr_Behavior() {
        Hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ladder_run_to_position0() {
        int tgt0Position = 6;
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt0Position);
        LadderLift.setPower(0.1);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void ladder_run_to_position1() {
        int tgt1Position = 150;
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt1Position);
        LadderLift.setPower(0.5);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void ladder_run_to_position2() {
        int tgt2Position = 300;
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt2Position);
        LadderLift.setPower(0.5);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void ladder_run_to_position3() {
        int tgt3Position = 600;
        LadderLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LadderLift.setTargetPosition(tgt3Position);
        LadderLift.setPower(0.8);
        LadderLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LadderLift.isBusy()) {
            telemetry.addData("LadderPos:", LadderLift.getCurrentPosition());
            telemetry.update();
        }
    }

    public void hook_run_to_high_position() {
        int tgthPosition = 8400;
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hook.setTargetPosition(tgthPosition);
        Hook.setPower(0.6);
        Hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Hook.isBusy()) {
            telemetry.addData("Hook Pos:", Hook.getCurrentPosition());
            telemetry.update();
        }
    }

    public void hook_run_to_zero_position() {
        int tgthlPosition = 40;
        Hook.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hook.setTargetPosition(tgthlPosition);
        Hook.setPower(0.6);
        Hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Hook.isBusy()) {
            telemetry.addData("Hook Pos:", Hook.getCurrentPosition());
            telemetry.update();
        }
        Hook_Zero_Pwr_Behavior();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        init_motors();
        telemetry.addData("Status", "Actuators Initialized");
        telemetry.update();

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            //noinspection BusyWait
            Thread.sleep(50);
        }
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated.");
        telemetry.clear();
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        /*Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.*/
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addLine()
                    .addData("dx", formatRate(rates.xRotationRate))
                    .addData("dy", formatRate(rates.yRotationRate))
                    .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));

            telemetry.addLine()
                    .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("roll", formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("pitch", "%s deg", formatAngle(angles.angleUnit, angles.thirdAngle));
            telemetry.update();

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            Mecanumdrive();
            /* servo open and close and positions */ /* Hook Positions */
            if (gamepad1.dpad_left) {
                Test.setPosition(0);
            } else if (gamepad1.dpad_right) {
                Test.setPosition(1);
            } else if (gamepad1.left_bumper) {
                Servoarm.setPosition(0);
            } else if (gamepad1.right_bumper) {
                Servoarm.setPosition(1);
            } else if (gamepad1.dpad_up) {
                hook_run_to_high_position();
            } else if (gamepad1.dpad_down) {
                hook_run_to_zero_position();
            }

            /* LadderLift positions */ /* Drone Positions */
            if (gamepad2.a) {
                ladder_run_to_position0();
            } else if (gamepad2.x) {
                ladder_run_to_position1();
            } else if (gamepad2.b) {
                ladder_run_to_position2();
            } else if (gamepad2.y) {
                ladder_run_to_position3();
            } else if (gamepad2.left_bumper) {
                Drone.setPosition(0);
            } else if (gamepad2.right_bumper) {
                Drone.setPosition(1);
            }
            telemetry.update();
            idle();
        }
        limelight.stop();
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
          /*  if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }*/

