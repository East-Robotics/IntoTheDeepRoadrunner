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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Vector;

@Config
@Autonomous(name = "4+0?", group = "Autonomous")
public class fourSpeciemenAuto extends LinearOpMode {
    public class Lift {
        public DcMotorEx llift;
        public DcMotorEx rlift;
;
        public Lift(HardwareMap hardwareMap) {
            llift = hardwareMap.get(DcMotorEx.class, "LArm");
            rlift = hardwareMap.get(DcMotorEx.class, "RArm");
            llift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            llift.setDirection(DcMotorSimple.Direction.REVERSE);
            rlift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rlift.setDirection(DcMotorSimple.Direction.FORWARD);
            //Encoders
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
        public Action liftMid(){
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
                if (pos > 1900) {
                    return true;
                } else {
                    //   sleep(300);
                    llift.setPower(0);
                    rlift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftPark(){
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
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Slide {
        public DcMotorEx lslide;
        public DcMotorEx rslide;

        public Slide(HardwareMap hardwareMap) {
            lslide = hardwareMap.get(DcMotorEx.class, "LSlide");
            rslide = hardwareMap.get(DcMotorEx.class, "RSlide");
            lslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lslide.setDirection(DcMotorSimple.Direction.FORWARD);
            rslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rslide.setDirection(DcMotorSimple.Direction.FORWARD);
            //Encoders
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
                if (pos < 1700) {
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
                if (pos > 20) {
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
                claw.setPosition(0.27);
                sleep(500);
                return false;
            }
        }
        public Action closeClaw() {
            return new fourSpeciemenAuto.Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.01);
                sleep(500);
                return false;
            }
        }
        public Action openClaw() {
            return new fourSpeciemenAuto.Claw.OpenClaw();}
    }
    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

        public class DownWrist implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0.2);
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


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(29.5);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(29.5,0,0))
                .waitSeconds(.5)
                .lineToX(21.5)
                .strafeToLinearHeading(new Vector2d(21.5,-28),Math.toRadians(0));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(21,-28,Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToX(55)
                .strafeToConstantHeading(new Vector2d(55,-39))
                .strafeToConstantHeading(new Vector2d(5,-39));
                //.strafeToLinearHeading(new Vector2d(-25,))
                //.turn(Math.toRadians(-45));
                //.lineToYLinearHeading(45,Math.toRadians(-45));
                //.strafeToLinearHeading(new Vector2d(18,41), Math.toRadians(-45));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(5,-48,Math.toRadians(0)))
                .waitSeconds(0.5)
                //.lineToXLinearHeading(20, Math.toRadians(0));
                .strafeToConstantHeading(new Vector2d(29.5,1));
                //.strafeToLinearHeading(new Vector2d(11,45),Math  .toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(22,49,Math.toRadians(0)))
                .waitSeconds(0.5)
                //.lineToX(15)
                //.lineToYLinearHeading(45,Math.toRadians(-45));
                .strafeToConstantHeading(new Vector2d(21,-39))
                .strafeToConstantHeading(new Vector2d(55,-40))
                .strafeToConstantHeading(new Vector2d(55,-45))
                .strafeToConstantHeading(new Vector2d(10,-45));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(18,41,Math.toRadians(-45)))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(5,-45));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(21,52,Math.toRadians(22)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(18,41), Math.toRadians(-45));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(18,41,Math.toRadians(-45)))
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(75,1.5), Math.toRadians(-90));
        Action trajectoryActionClose1 = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();
        Action trajectoryActionClose2 = tab2.endTrajectory().fresh()
                .build();
        Action trajectoryActionClose3 = tab3.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose4 = tab4.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose5 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();
        Action trajectoryActionClose6 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();

        Action trajectoryActionClose7 = tab7.endTrajectory().fresh()
                .waitSeconds(.2)
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.liftInit());

      //  Actions.runBlocking(wrist.wristBack());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();

        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(
                        //specimen
                        wrist.wristBack(),
                        tab1.build(),
                        trajectoryActionClose1,
                        wrist.wristDown(),
                        lift.liftMid(),
                        claw.openClaw(),
                        new ParallelAction(
                                wrist.wristBack(),
                                tab2.build()
                        ),
                        //Push back 1st
                        new ParallelAction(
                                tab3.build(),
                                lift.liftDown(),
                                wrist.wristBack()
                        ),
                        lift.liftUp(),
                        claw.closeClaw(),
                        new ParallelAction(
                                lift.liftPark(),
                                tab4.build(),
                                wrist.wristDown()
                        ),
                        slide.Slideout(),
                        wrist.wristBack(),
                        claw.openClaw(),
                        new ParallelAction(
                                tab5.build(),
                                lift.liftDown(),
                                slide.Slidein(),
                                wrist.wristBack()
                        ),
                        lift.liftUp(),
                        claw.closeClaw(),
                        new ParallelAction(
                                tab4.build(),
                                lift.liftPark(),
                                wrist.wristDown()
                        ),
                        slide.Slideout(),
                        wrist.wristBack(),
                        claw.openClaw(),
                        new ParallelAction(
                                tab6.build(),
                                lift.liftUp()
                        )
                )


        );
    }
}