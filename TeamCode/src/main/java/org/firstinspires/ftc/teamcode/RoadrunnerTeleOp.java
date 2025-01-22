package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.MecanumDrive.DriveLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.ClassActions;

@Config
@TeleOp(name = "RoadRunnerTeleOp", group = "TeleOp")
public class RoadrunnerTeleOp extends LinearOpMode {


    //TwoDeadWheelLocalizer myLocalizer = new TwoDeadWheelLocalizer(hardwareMap);
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

    private ColorSensor colorSensor;
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

  /*  public void colorTelemetry() {
        telemetry.addData("redValue","%.2f", redValue);
        telemetry.addData("greenValue","%.2f", greenValue);
        telemetry.addData("blueValue","%.2f", blueValue);
        telemetry.addData("alphaValue","%.2f", alphaValue);
        telemetry.update();

    }*/

    @Override
    public void runOpMode() throws InterruptedException {



        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

       /* ClassActions.Claw claw = new claw(hardwareMap);
        ClassActions.Lift lift = new lift(hardwareMap);
        ClassActions.Slide slide = new slide(hardwareMap);
        ClassActions.Wrist wrist = new wrist(hardwareMap);*/

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

        /*TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
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

            initHardware();
            while (!isStarted()) {
                getColor();
             //   colorTelemetry();
            }
            while (opModeIsActive()) {
                getColor();
                //colorTelemetry();

                int  LArmPos = LArm.getCurrentPosition();
                int  RArmPos = RArm.getCurrentPosition();
                int  LSlidePos = LSlide.getCurrentPosition();
                int  RSlidePos = RSlide.getCurrentPosition();
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
                telemetry.addData("x", initialPose.position);
                telemetry.addData("Y", initialPose.position);
                telemetry.addData("heading", initialPose.heading);

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

            double LFtgtPower = (rotY + rotX + pa)/denominator;
            double LBtgtPower = (-rotY + rotX - pa) /denominator;
            double RFtgtPower = (rotY - rotX - pa)/denominator;
            double RBtgtPower = (rotY + rotX - pa)/denominator;

            RFMotor.setPower(RFtgtPower);

            LFMotor.setPower(LFtgtPower);

            RBMotor.setPower(RBtgtPower);

            LBMotor.setPower(LBtgtPower);


                ClassActions.Wrist wrist = (ClassActions.Wrist);
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
        /*    if (currentXState && !lastXState) {
                ClawIsOpen = !ClawIsOpen;
            }

            lastXState = currentXState;
*/
                ClassActions.Claw claw = (ClassActions.Claw) ;
                if (ClawIsOpen && gamepad2.x){
                Actions.runBlocking(
                        new ParallelAction(
                                claw.openClaw()
                        )

                );
                ClawIsOpen = false;

            }
            if(!ClawIsOpen && gamepad2.x) {
                Actions.runBlocking(
                        new ParallelAction(
                               claw.closeClaw()

                        )
                );
                ClawIsOpen = true;
            }

//Arm control
            if (gamepad2.right_bumper) {//down
                if ((RArmPos <400)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                }
                else {
                    LArm.setPower(0.85);
                    RArm.setPower(-0.85);
                }

            }
            else if(gamepad2.left_bumper){//up
                if ((RArmPos> 2400)) {
                    LArm.setPower(0);
                    RArm.setPower(0);
                }
                else {
                    LArm.setPower(-0.85);
                    RArm.setPower(0.85);
                }
            }
            else{
                LArm.setPower(-0.08);
                RArm.setPower(0.08);
            }
//Slide control
            if (gamepad1.right_bumper){//down
                LSlide.setPower(0.9);
                RSlide.setPower(0.9);
            }
            else if (gamepad1.left_bumper){//up
                if ((LArmPos < 1300) && (RArmPos < 1300) && ((LSlidePos > 900))){
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

           /* if (redValue> redTarget) {
                claw.closeClaw();
            }

            if (greenValue> greenTarget) {
                claw.closeClaw();
            }

            if (blueValue> blueTarget) {
                claw.closeClaw();
            }*/


                ClassActions.Slide slide = null;
                if (gamepad1.dpad_up) {
                Actions.runBlocking(
                        new ParallelAction(
                        new SequentialAction(
                        //        lift.liftUp(),
                                slide.slideUp(),
                                wrist.wristBack()
                        )
                        )
                );
            }

            if (gamepad1.dpad_down) {
                Actions.runBlocking(
                        new ParallelAction(
                        new SequentialAction(
                                claw.closeClaw(),
                                wrist.wristDown(),
                                slide.slideDown()
                              //  lift.liftDown()
                        )
                        )
                );
            }

        }
        if (isStopRequested()) return;




    }
}
