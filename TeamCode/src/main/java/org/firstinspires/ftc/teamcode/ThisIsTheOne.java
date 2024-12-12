package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleops.CenterstageTestCode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "ThisIsTheOne", group = "Autonomous")
public class ThisIsTheOne extends LinearOpMode {
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
                    rlift.setPower(0.8);
                    llift.setPower(0.8);
                    initialized = true;
                }

                double pos = llift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2300) {
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
                if (pos < 1300) {
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
                    llift.setPower(-0.8);
                    rlift.setPower(-0.8);
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
                    rslide.setPower(-0.8);
                    lslide.setPower(-0.8);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 1300) {
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
                    lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    initialized = true;
                }

                double pos = lslide.getCurrentPosition();
                packet.put("SlidePos", pos);
                if (pos < 400) {
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
    }



    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
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
                return false;
            }
        }
        public Action wristDown() {
            return new DownWrist();
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
                .lineToX(25.3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(21.5,39.5));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(new Vector2d(0, 43), 0))
                .waitSeconds(0.5)
                .lineToXLinearHeading(21.5,-45);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(21.5, 49.5), Math.toRadians(0));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(18, 49.5), Math.toRadians(-45));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(15.5, 45.5), Math.toRadians(-45));

        Action trajectoryActionClose1 = tab1.endTrajectory().fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();
        Action trajectoryActionClose2 = tab2.endTrajectory().fresh()
                .build();
        Action trajectoryActionClose3 = tab3.endTrajectory().fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(lift.liftInit());

       // Actions.runBlocking(wrist.wristUp());


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
                        wrist.wristUp(),
                        tab1.build(),
                        trajectoryActionClose1,
                        wrist.wristDown(),
                        lift.liftMid(),
                        claw.openClaw(),
                        wrist.wristUp(),
                        tab2.build(),
                        wrist.wristDown(),
                        lift.liftDown(),
                        claw.closeClaw(),
                        trajectoryActionClose2,
                        lift.liftUp(),
                        tab3.build(),
                        wrist.wristUp(),
                        slide.slideUp(),
                        trajectoryActionClose3,
                        claw.openClaw(),
                        wrist.wristDown(),
                        slide.slideDown(),
                        lift.liftDown()




                )
        );
    }
}