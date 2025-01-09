package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "RoadRunnerTeleOp", group = "TeleOp")
public class RoadrunnerTeleOp extends LinearOpMode {
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
    boolean ClawIsOpen = false;

    boolean lastYState = false;
    boolean currentYState = false;

    boolean lastXState = false;
    boolean currentXState = false;

    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue; // Light Intensity
    private double redTarget = 200;
    private double greenTarget = 200;
    private double blueTarget = 200;

    public void initHardware( ){
        initColorSensor();
    }
    public void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class, "ClawColor");
    }
    public void getColor() {
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }

    public void colorTelemetry() {
        telemetry.addData("redValue","%.2f", redValue);
        telemetry.addData("greenValue","%.2f", greenValue);
        telemetry.addData("blueValue","%.2f", blueValue);
        telemetry.addData("alphaValue","%.2f", alphaValue);
        telemetry.update();

    }





    public class Lift {
        private DcMotorEx llift;
        private DcMotorEx rlift;

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
                    rlift.setPower(1);
                    llift.setPower(1);
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
                    llift.setPower(-0.6);
                    rlift.setPower(-0.6);
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
        public Action liftMid(){
            return new LiftMid();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    llift.setPower(-0.9);
                    rlift.setPower(-0.9);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0) {
                    return true;
                } else {
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
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
                    rslide.setPower(-1);
                    lslide.setPower(-1);
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
                    lslide.setPower(1);
                    rslide.setPower(1);
                    //lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos > 25) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action slideDown(){
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
        public Action Slideout(){
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
                if (pos > 200) {
                    return true;
                } else {
                    lslide.setPower(0);
                    rslide.setPower(0);
                    return false;
                }
            }
        }
        public Action Slidein(){
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
                claw.setPosition(0.57);
                sleep(300);
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
                wrist.setPosition(0.3);
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
                sleep(400);
                wrist.setPosition(1);
                return false;
            }
        }

        public  Action wristBack() {
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
    public class Drive{
        public Action turn(double angle){
            return new TodoAction();
        }
    }


    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
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





        //Encoders
       /* LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

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

        Wrist.setDirection(Servo.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        //LArm.setDirection(DcMotor.Direction.REVERSE);

        int  LArmPos = LArm.getCurrentPosition();
        int  RArmPos = RArm.getCurrentPosition();
        int  LSlidePos = LSlide.getCurrentPosition();
        int  RSlidePos = RSlide.getCurrentPosition();

        telemetry.addData("RArmPos", RArmPos);
        telemetry.addData("LArmPos", LArmPos);
        telemetry.addData("LSlidePos", LSlidePos);
        telemetry.addData("RSlidePos", RSlidePos);
        telemetry.update();

        currentYState = gamepad2.y;
        currentXState = gamepad2.x;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        imu.resetYaw();

        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //.lineToX(25);
                .strafeToLinearHeading(new Vector2d(5,5),Math.toRadians(-45));
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(25,40,Math.toRadians(0)));
               // .waitSeconds(0.3)
                //.lineToX(20)
                //.strafeToLinearHeading(new Vector2d(25,40),Math.toRadians(-45));
        //.splineTo(new Vector2d(20.5,39.5),Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(20.5,39.5,Math.toRadians(0)))
                .waitSeconds(0.3)
                //.lineToX(10)
                //.turn(Math.toRadians(-45));
                //.lineToYLinearHeading(45,Math.toRadians(-45));
                .strafeToLinearHeading(new Vector2d(13,44), Math.toRadians(-45));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(13,44,Math.toRadians(-45)))
                .waitSeconds(0.3)
                //.lineToXLinearHeading(20, Math.toRadians(0));
                .strafeToLinearHeading(new Vector2d(20.5,48.7), Math.toRadians(0));
        //.strafeToLinearHeading(new Vector2d(11,45),Math  .toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(20.5,48.7,Math.toRadians(0)))
                .waitSeconds(0.3)
                //.lineToX(15)
                //.lineToYLinearHeading(45,Math.toRadians(-45));

                .strafeToLinearHeading(new Vector2d(13,44), Math.toRadians(-45));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(13,44,Math.toRadians(-45)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(21.5,54), Math.toRadians(20));
        Action trajectoryActionClose1 = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();
        Action trajectoryActionClose2 = tab2.endTrajectory().fresh()
                .build();
        Action trajectoryActionClose3 = tab3.endTrajectory().fresh()
                .waitSeconds(.5)
                .build();
        Action trajectoryActionClose4 = tab4.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose5 = tab5.endTrajectory().fresh()
                .waitSeconds(.5)
                .build();
        Action trajectoryActionClose6 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        /*Actions.runBlocking(claw.closeClaw());
        //Actions.runBlocking(lift.liftInit());

        Actions.runBlocking(wrist.wristBack());*/


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }


        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();

        waitForStart();



            initHardware();
            while (!isStarted()) {
                getColor();
                colorTelemetry();
            }
            while (opModeIsActive()) {
                getColor();
                colorTelemetry();

            double py = -gamepad1.left_stick_y;
            double px = gamepad1.left_stick_x;
            double pa = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading -= Math.toRadians(0);

            if (gamepad1.start) {
                imu.resetYaw();
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = px * Math.cos(-botHeading) - py * Math.sin(-botHeading);
            double rotY = px * Math.sin(-botHeading) + py * Math.cos(-botHeading);

            rotX = rotX * 1.5;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(pa), 1);

            double LFtgtPower = (rotY + rotX + pa)/denominator;
            double LBtgtPower = (-rotY + rotX - pa) /denominator;
            double RFtgtPower = (rotY - rotX - pa)/denominator;
            double RBtgtPower = (rotY + rotX - pa)/denominator;

            RFMotor.setPower(RFtgtPower);

            LFMotor.setPower(LFtgtPower);

            RBMotor.setPower(RBtgtPower);

            LBMotor.setPower(LBtgtPower);



           /* if (gamepad1.a) {
                Actions.runBlocking(
                        new SequentialAction(
                                tab1.build()

                        )
                );
            }*/

          /*  if (gamepad2.x) {
                Actions.runBlocking(
                        new SequentialAction(
                               claw.closeClaw()
                        )
                );
            }*/
            //Wrist toggle
            if (currentYState && !lastYState) {
                WristIsOpen = !WristIsOpen;
            }

            lastYState = currentYState;

            if (WristIsOpen) {
                Wrist.setPosition(0.72);
            }
            else {
                Wrist.setPosition(0);
            }

            if (gamepad1.b){
                LArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                LSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                LSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                LArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            //Claw toggle
            if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;

            if (ClawIsOpen) {
                Claw.setPosition(0.2);

            } else {
                Claw.setPosition(0.40);
            }

//Arm control
            if (gamepad2.right_bumper) {
                LArm.setPower(-0.85);
                RArm.setPower(-0.85);
            }
            else if(gamepad2.left_bumper){
                LArm.setPower(0.85);
                RArm.setPower(0.85);
            }
            else{
                LArm.setPower(0.05);
                RArm.setPower(0.05);
            }
//Slide control
            if (gamepad1.right_bumper){//down
                LSlide.setPower(0.9);
                RSlide.setPower(0.9);
            }
            else if (gamepad1.left_bumper){//up
                if ((LArmPos < 1300) && (RArmPos < 1300) && ((LSlidePos > 1450))){
                    LSlide.setPower(0);
                    RSlide.setPower(0);
                }
                else{
                    LSlide.setPower(-0.9);
                    RSlide.setPower(-0.9);
                }
            }
            else{
                LSlide.setPower(-0.04);
                RSlide.setPower(-0.04);
            }

            if (redValue> redTarget) {
                claw.closeClaw();
            }

            if (greenValue> greenTarget) {
                claw.closeClaw();
            }

            if (blueValue> blueTarget) {
                claw.closeClaw();
            }


            if (gamepad2.a) {
                Actions.runBlocking(
                        new SequentialAction(
                                lift.liftUp(),
                                slide.slideUp(),
                                wrist.wristDown(),
                                wrist.wristBack()

                        )
                );
            }

            if (gamepad2.b) {
                Actions.runBlocking(
                        new SequentialAction(
                                claw.openClaw(),
                                wrist.wristDown(),
                                slide.slideDown(),
                                lift.liftDown()
                        )
                );
            }

        }

        if (isStopRequested()) return;




    }
}