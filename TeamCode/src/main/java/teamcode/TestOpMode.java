package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOpMode extends LinearOpMode {

    //private Gyroscope imu;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;

    private DcMotor LadderLift;
    private DcMotor Hook;
    private DcMotor Frontright;
    private DcMotor Frontleft;
    private DcMotor Backleft;
    private DcMotor Backright;
    private IMU imu;

    private Servo Test;
    private Servo Drone;
    private Servo Servoarm;
    //static final double INCREMENT = 0.01;
    //static final int CYCLE_MS = 50;
    //static final double MAX_FWD = 1.0;
    //static final double MAX_REV = -1.0;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    int cp = 0;
    //double power = 0;
    double tgtPower = 0;
    //boolean rampUp = true;

    public void init_motors() {
        Test = hardwareMap.get(Servo.class, "Test");
        Servoarm = hardwareMap.get(Servo.class, "Servoarm");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Backleft = hardwareMap.get(DcMotor.class, "Backleft");
        Backright = hardwareMap.get(DcMotor.class, "Backright");
        Frontleft = hardwareMap.get(DcMotor.class, "Frontleft");
        Frontright = hardwareMap.get(DcMotor.class, "Frontright");
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        imu = hardwareMap.get(IMU.class, "imu");
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

   /* public void Mecanumdrive() {
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
    }*/

    public void AllMotorsFrwrd() {
        tgtPower = gamepad1.left_stick_y;
        Backleft.setPower(tgtPower);
        Backright.setPower(tgtPower);
        Frontleft.setPower(tgtPower);
        Frontright.setPower(tgtPower);
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Status", "Motors");
        telemetry.update();
    }

    public void run_to_position_FL() {
        int tgtPosition = 8000;
        cp = -1 * Frontleft.getCurrentPosition();
        Frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontleft.setTargetPosition(tgtPosition);
        Frontleft.setPower(1);
        Frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Frontleft.isBusy()) {

            telemetry.addData("DcMotor PositionFL:", Frontleft.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        }
        Frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontleft.setPower(0);
    }

    public void run_to_position_FR() {
        int tgtPosition = 8000;
        cp = -1 * Frontright.getCurrentPosition();
        Frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Frontright.setTargetPosition(tgtPosition);
        Frontright.setPower(1);
        Frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Frontright.isBusy()) {

            telemetry.addData("DcMotor PositionFR:", Frontright.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        }
        Frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Frontright.setPower(0);
    }

    public void run_to_position_BL() {
        int tgtPosition = 8000;
        cp = -1 * Backleft.getCurrentPosition();
        Backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backleft.setTargetPosition(tgtPosition);
        Backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backleft.setPower(1);
        while (Backleft.isBusy()) {

            telemetry.addData("DcMotor PositionBL:", Backleft.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        }
        Backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backleft.setPower(0);
    }

    public void run_to_position_BR() {
        int tgtPosition = 8000;
        cp = -1 * Backright.getCurrentPosition();
        Backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Backright.setTargetPosition(tgtPosition);
        Backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backright.setPower(1);
        while (Backright.isBusy()) {

            telemetry.addData("DcMotor PositionBR:", Backright.getCurrentPosition());
            telemetry.addData("cp", cp);
            telemetry.update();
        }
        Backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Backright.setPower(0);
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
    public void runOpMode() {

        /* imu = hardwareMap.get(Gyroscope.class, "imu");
         digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
         sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
         */

        init_motors();
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
        // Check to see if heading reset is requested
        if (gamepad1.y) {
            telemetry.addData("Yaw", "Resetting\n");
            imu.resetYaw();
        } else {
            telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
        }

        waitForStart();

        while (opModeIsActive()) {

           // Mecanumdrive();

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();

            /* servo open and close and positions */

            if (gamepad1.dpad_left) {
                Test.setPosition(0);
            } else if (gamepad1.dpad_right) {
                Test.setPosition(1);
            } else if (gamepad1.left_bumper) {
                Servoarm.setPosition(0);
            } else if (gamepad1.right_bumper) {
                Servoarm.setPosition(1);
            } else if (gamepad1.a) {
                run_to_position_FL();
            } else if (gamepad1.b) {
                run_to_position_FR();
            } else if (gamepad1.x) {
                run_to_position_BL();
            } else if (gamepad1.y) {
                run_to_position_BR();
            }

            /* LadderLift positions */

            if (gamepad2.a) {
                ladder_run_to_position0();
            } else if (gamepad2.x) {
                ladder_run_to_position1();
            } else if (gamepad2.b) {
                ladder_run_to_position2();
            } else if (gamepad2.y) {
                ladder_run_to_position3();
            }

            /* Drone Positions */

            if (gamepad2.left_bumper) {
                Drone.setPosition(0);
            } else if (gamepad2.right_bumper) {
                Drone.setPosition(1);
            }

            /* Hook Positions */

            if (gamepad1.dpad_up) {
                hook_run_to_high_position();
            } else if (gamepad1.dpad_down) {
                hook_run_to_zero_position();
            }

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
            }

            if (gamepad1.x) {
                Test.setPosition(0);
            } else if (gamepad1.y || gamepad1.b) {
                Test.setPosition(0.5);
            } else if (gamepad1.a) {
                Test.setPosition(1);
            }

            telemetry.addData("Motor Power", "%5.2f", power);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            tEst.setPower(power);
            sleep(CYCLE_MS);
            idle();
        }
            tEst.setPower(0);
            telemetry.addData(">", "Done");
            telemetry.update();
*/

}




