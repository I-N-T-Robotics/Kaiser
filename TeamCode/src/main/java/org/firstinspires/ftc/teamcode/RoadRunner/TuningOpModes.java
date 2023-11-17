package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.AngularRampLogger;
import org.firstinspires.ftc.teamcode.DriveType;
import org.firstinspires.ftc.teamcode.DriveViewer;
import org.firstinspires.ftc.teamcode.DriveViewerFactory;
import org.firstinspires.ftc.teamcode.ForwardPushTest;
import org.firstinspires.ftc.teamcode.ForwardRampLogger;
import org.firstinspires.ftc.teamcode.LateralPushTest;
import org.firstinspires.ftc.teamcode.LateralRampLogger;
import org.firstinspires.ftc.teamcode.ManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.MecanumMotorDirectionDebugger;
import org.firstinspires.ftc.teamcode.Testing.LocalizationTest;
import org.firstinspires.ftc.teamcode.Testing.ManualFeedbackTuner;
import org.firstinspires.ftc.teamcode.Testing.SplineTest;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = DriveTrain.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewerFactory dvf;
        if (DRIVE_CLASS.equals(DriveTrain.class)) {
            dvf = hardwareMap -> {
                DriveTrain dt = new DriveTrain(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (dt.localizer instanceof DriveTrain.DriveLocalizer) {
                    DriveTrain.DriveLocalizer dl = (DriveTrain.DriveLocalizer) dt.localizer;
                    leftEncs.add(dl.leftFront);
                    leftEncs.add(dl.leftRear);
                    rightEncs.add(dl.rightFront);
                    rightEncs.add(dl.rightRear);
                } else if (dt.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) dt.localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else {
                    throw new IllegalArgumentException("unknown localizer: " + dt.localizer.getClass().getName());
                }

                return new DriveViewer(
                    DriveType.MECANUM,
                        DriveTrain.PARAMS.inPerTick,
                        DriveTrain.PARAMS.maxWheelVel,
                        DriveTrain.PARAMS.minProfileAccel,
                        DriveTrain.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(
                                dt.leftFront,
                                dt.leftBack
                        ),
                        Arrays.asList(
                                dt.rightFront,
                                dt.rightBack
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        dt.imu,
                        dt.angIMU,
                        dt.voltageSensor,
                        dt.feedforward
                );
            };
        } else {
            throw new AssertionError();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);
    }
}

