package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
@TeleOp(name = "RoadRunnerTeleOpCoords", group = "TeleOp")
public class RoadrunnerTeleOpCoordinates extends LinearOpMode {

    private MecanumDrive drive;
    private boolean trajectoryTriggered = false;

     RevBlinkinLedDriver lights;


    public void initlights() {
        //lights = hardwareMap.get(RevBlinkinLedDriver.class, "Blinkin");
        //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    //private Pose2d targetPose = new Pose2d(30, 30, Math.toRadians(45));
    // private Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));

  /*  public void colorTelemetry() {
        telemetry.addData("redValue","%.2f", redValue);
        telemetry.addData("greenValue","%.2f", greenValue);
        telemetry.addData("blueValue","%.2f", blueValue);
        telemetry.addData("alphaValue","%.2f", alphaValue);
        telemetry.update();

    }*/

    public class Lift {
        private DcMotorEx llift;
        private DcMotorEx rlift;
        ;

        public Lift(HardwareMap hardwareMap) {
            llift = hardwareMap.get(DcMotorEx.class, "LArm");
            rlift = hardwareMap.get(DcMotorEx.class, "RArm");
            llift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            llift.setDirection(DcMotorSimple.Direction.REVERSE);
            rlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rlift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rlift.setPower(0.75);
                    llift.setPower(0.75);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2900) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftInit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rlift.setPower(0.8);
                    llift.setPower(0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1450) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftInit() {
            return new LiftInit();
        }

        public class LiftMid implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.8);
                    rlift.setPower(-0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    sleep(300);
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftMid() {
            return new LiftMid();
        }

        public class LiftPark implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.8);
                    rlift.setPower(-0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1800) {
                    return true;
                } else {
                    //   sleep(300);
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftPark() {
            return new LiftPark();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.75);
                    rlift.setPower(-0.75);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 10) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class Slide {
        private DcMotorEx lslide;
        private DcMotorEx rslide;

        public Slide(HardwareMap hardwareMap) {
            lslide = hardwareMap.get(DcMotorEx.class, "LSlide");
            rslide = hardwareMap.get(DcMotorEx.class, "RSlide");
            lslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lslide.setDirection(DcMotorSimple.Direction.FORWARD);
            rslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class Up implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rslide.setPower(-0.8);
                    lslide.setPower(-0.8);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 2500) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }

        public Action slideUp() {
            return new Up();
        }

        public class Down implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(0.8);
                    rslide.setPower(0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode. STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos > 165) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }

        public Action slideDown() {
            return new Down();
        }

        public class Out implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(-0.8);
                    rslide.setPower(-0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }

        public Action Slideout() {
            return new Out();
        }

        public class In implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lslide.setPower(0.8);
                    rslide.setPower(0.8);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos > 215) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }

        public Action Slidein() {
            return new In();
        }
    }


    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.28);
                sleep(500);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.2);
                sleep(500);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

        public class DownWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.25);
                sleep(500);
                return false;
            }
        }

        public Action wristDown() {
            return new DownWrist();
        }

        public class BackWrist implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(1);
                sleep(200);
                return false;
            }
        }

        public Action wristBack() {
            return new BackWrist();
        }

        public class UpWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.6);
                return false;
            }
        }

        public Action wristUp() {
            return new UpWrist();
        }
    }

    public class Drive {
        public Action turn(double angle) {
            return new TodoAction();
        }
    }

    private DcMotor LFMotor;
    private DcMotor RFMotor;
    private DcMotor LBMotor;
    private DcMotor RBMotor;

    private DcMotor LSlide;
    private DcMotor RSlide;
    private DcMotor LArm;
    private DcMotor RArm;
    private Servo Wrist;
    private Servo Claw;


    boolean WristIsOpen = true;
    boolean ClawIsOpen = true;

    boolean lastYState = false;
    boolean currentYState = false;

    boolean lastXState = false;
    boolean currentXState = false;

    private RevColorSensorV3 colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; // Light Intensity
    private double redTarget = 200;
    private double greenTarget = 200;
    private double blueTarget = 200;

   /* public RoadrunnerTeleOp() {
        robot = new Robot(hardwareMap);
    }*/

    public void initHardware() {
        initColorSensor();
    }

    public void initColorSensor() {
        //colorSensor = hardwareMap.get(RevColorSensorV3.class, "ClawColor");
    }

    private RevColorSensorV3 ColorSensor;
    //private RevBlinkinLedDriver lights;

   /* public void getColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }*/


    @Override
    public void runOpMode() throws InterruptedException {
        //drive = new MecanumDrive(hardwareMap);
        // drive.setPoseEstimate(new Pose2d(0,0,0));
        //drive.updatePoseEstimate();

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, startPose);
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ClawColor");
        colorSensor.enableLed(true); // Turn on LED

        //lights = hardwareMap.get(RevBlinkinLedDriver.class, "Lights");


        //Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        //MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
        //.strafeToLinearHeading(new Vector2d(20,20), Math.toRadians(45));


        //Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        //MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LSlide = hardwareMap.get(DcMotor.class, "LSlide");
        RSlide = hardwareMap.get(DcMotor.class, "RSlide");
        LArm = hardwareMap.get(DcMotor.class, "LArm");
        RArm = hardwareMap.get(DcMotor.class, "RArm");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Claw = hardwareMap.get(Servo.class, "Claw");


        // ClassActions.Claw claw = (ClassActions.Claw) hardwareMap.get(Servo.class, "Claw");
        //ClassActions.Wrist wrist = (ClassActions.Wrist) hardwareMap.get(Servo.class, "Wrist");
        // ClassActions.Slide slide = (ClassActions.Slide) hardwareMap.get(DcMotor.class, "LSlide");


        //Encoders
       /* LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        //localizer = new TwoDeadWheelLocalizer(hardwareMap, lazyImu.get(), PARAMS.inPerTick);

        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);


        //   LArm.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Running");
        telemetry.update();

        LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LArm.setPower(0.04);
        RArm.setPower(-0.04);


        //Wrist.setDirection(Servo.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);

        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        //LArm.setDirection(DcMotor.Direction.REVERSE);


        currentYState = gamepad2.y;
        currentXState = gamepad2.x;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        imu.resetYaw();

        // vision here that outputs position
        int visionOutputPosition = 1;


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();

        }
        /*NormalizedRGBA colors = colorSensor.getNormalizedColors();
        final float[] hsvValues = new float[3];
        Color.colorToHSV(colors.toColor(), hsvValues);


        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);

        if (colors.red >1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        if (colors.red >1 && colors.green >1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        if (colors.blue >1) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }*/


        initHardware();
        while (!isStarted()) {

            //   colorTelemetry();
        }
        while (opModeIsActive()) {
            Pose2d currentPose = drive.getPoseEstimate();
            drive.updatePoseEstimate();

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Convert RGB to HSV (Hue is in degrees: 0-360)
            float[] hsv = new float[3];
            Color.RGBToHSV(red, green, blue, hsv);
            float hue = hsv[0];

            if ((hue >= 330 || hue < 30) && hsv[1] > 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);// Red (high saturation)
            } else if (hue >= 90 && hue < 150 && hsv[1] > 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);// Green
            } else if (hue >= 210 && hue < 270 && hsv[1] > 0.5) {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);// Blue
            } else {
                lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
            }

            double x = currentPose.position.x;
            double y = currentPose.position.y;
            double heading = currentPose.heading.real;

            telemetry.addData("Pose X", x);
            telemetry.addData("Pose Y", y);
            telemetry.addData("Pose Heading", Math.toDegrees(heading));
            telemetry.update();


            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();


            //colorTelemetry();

            int LArmPos = LArm.getCurrentPosition();
            int RArmPos = RArm.getCurrentPosition();
            int LSlidePos = LSlide.getCurrentPosition();
            int RSlidePos = RSlide.getCurrentPosition();
            double WristPos = Wrist.getPosition();

            TwoDeadWheelLocalizer.Params Position = new TwoDeadWheelLocalizer.Params();

              /*  PositionVelocityPair parPosVel = par.getPositionAndVelocity();

                lastParPos = parPosVel.position;
                lastPerpPos = perpPosVel.position;
                lastHeading = heading;

*/
            telemetry.addData("RArmPos", RArmPos);
            telemetry.addData("LArmPos", LArmPos);
            telemetry.addData("LSlidePos", LSlidePos);
            telemetry.addData("RSlidePos", RSlidePos);
            telemetry.addData("ClawPos", ClawIsOpen);
            telemetry.addData("Wrist", WristIsOpen);
            telemetry.update();
            currentXState = gamepad2.x;
               /* telemetry.addData("x", initialPose.position);
                telemetry.addData("Y", initialPose.position);
                telemetry.addData("heading", initialPose.heading);*/

            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading -= Math.toRadians(90);

            if (gamepad1.start) {
                imu.resetYaw();
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = px * Math.cos(-botHeading) - py * Math.sin(-botHeading);
            double rotY = px * Math.sin(-botHeading) + py * Math.cos(-botHeading);

            rotX = rotX * 1.5;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(pa), 1);

            double LFtgtPower = (rotY + rotX + pa) / (denominator / 2);
            double LBtgtPower = (-rotY + rotX - pa) / (denominator / 2);
            double RFtgtPower = (rotY - rotX - pa) / (denominator / 2);
            double RBtgtPower = (rotY + rotX - pa) / (denominator / 2);

            RFMotor.setPower(RFtgtPower);

            LFMotor.setPower(LFtgtPower);

            RBMotor.setPower(RBtgtPower);

            LBMotor.setPower(LBtgtPower);

            if (WristIsOpen && gamepad2.y) {
                Actions.runBlocking(
                        new ParallelAction(
                                wrist.wristBack()
                        )
                );
                WristIsOpen = false;
            }
            if (!WristIsOpen && gamepad2.y) {
                Actions.runBlocking(
                        new ParallelAction(
                                wrist.wristDown()
                        )
                );
                WristIsOpen = true;
            }

            if (gamepad1.b) {
                LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;

            if (ClawIsOpen) {
                Claw.setPosition(0.01);
                //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);


            } else {
                Claw.setPosition(0.27);
                // lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);

            }



            //Claw toggle
        /*    if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;
*/

               /* if (ClawIsOpen && gamepad2.x){
                Actions.runBlocking(
                        new ParallelAction(
                                claw.openClaw()
                        )

                );
                ClawIsOpen = false;
                    telemetry.addData("ClawPos", ClawIsOpen);
                    telemetry.update();

            }
            if(!ClawIsOpen && gamepad2.x) {
                Actions.runBlocking(
                        new ParallelAction(
                               claw.closeClaw()

                        )
                );
                ClawIsOpen = true;
                telemetry.addData("ClawPos", ClawIsOpen);
                telemetry.update();
            }*/

//Arm control
            if (gamepad2.right_bumper) {//down
                if ((RArmPos < 100)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                    LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    LArm.setPower(-0.85);
                    RArm.setPower(-0.85);
                }

            } else if (gamepad2.left_bumper) {//up
                if ((RArmPos > 2700)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                } else {
                    LArm.setPower(0.85);
                    RArm.setPower(0.85);
                }
            } else {
                LArm.setPower(0.08);
                RArm.setPower(0.08);
            }
//Slide control
            if (gamepad1.right_bumper) {//down
                LSlide.setPower(0.9);
                RSlide.setPower(0.9);
            } else if (gamepad1.left_bumper) {//up
                if ((LArmPos < 1300) && (RArmPos < 1300) && ((LSlidePos > 900))) {
                    LSlide.setPower(0);
                    RSlide.setPower(0);
                } else {
                    LSlide.setPower(-0.9);
                    RSlide.setPower(-0.9);
                }
            } else {
                LSlide.setPower(-0.04);
                RSlide.setPower(-0.04);
            }

           /* if (redValue> redTarget) {
                claw.closeClaw();
            }

            if (greenValue> greenTarget) {
                claw.closeClaw();
            }

            if (blueValue> blueTarget) {
                claw.closeClaw();
            }*/


            if (gamepad1.dpad_up) {
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        //        lift.liftUp(),
                                        slide.slideUp()
                                )
                        )
                );
            }

            if (gamepad1.dpad_down) {
                Actions.runBlocking(
                        new ParallelAction(
                                //  claw.closeClaw(),
                                slide.slideDown()
                                //  lift.liftDown()
                        )
                );
            }


            // Check if the "A" button is pressed to drive to a specified location
           /* if (gamepad1.x) {
                // Define the target pose (x, y, heading)
                Pose2d targetPose = new Pose2d(30, 30, Math.toRadians(90)); // Example target

                // Build a trajectory from the current pose to the target pose
                TrajectoryActionBuilder tab1 = drive.actionBuilder(currentPose)
                        .strafeToLinearHeading(new Vector2d(1, 1), Math.toRadians(45));

                // Follow the trajectory
                Actions.runBlocking(
                        tab1.build()

                );

            }*/

            // Display telemetry
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
        }
        //if (gamepad1.x) {
        // Trigger movement to the target position when the A button is pressed
        // Actions.runBlocking(
        // new SequentialAction(
        // tab1.build()

        // )
        // );
        //  }


        if (isStopRequested()) return;

    }
}
