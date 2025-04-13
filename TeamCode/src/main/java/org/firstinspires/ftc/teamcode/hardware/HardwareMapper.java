package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.ArrayList;

/**
 * Annotation processor for hardware map things.
 */
public abstract class HardwareMapper {
    static final class BadAnnotationSetException extends RuntimeException {
        private static <A extends Annotation, B extends Annotation> String reason(
                Class<A> existing, Class<B> incoming, @Nullable String context
        ) {
            String message;
            if (context != null) message = String.format(
                    "Annotation @%s is incompatible with @%s\n    %s",
                    incoming.getSimpleName(), existing.getSimpleName(),
                    context
            );
            else message = String.format(
                    "Annotation @%s is incompatible with @%s",
                    incoming.getSimpleName(), existing.getSimpleName()
            );
            return message;
        }

        public <A extends Annotation, B extends Annotation> BadAnnotationSetException(
                Class<A> existing, Class<B> incoming, @Nullable String context
        ) {
            super(reason(existing, incoming, context));
        }
    }

    /**
     * Represents an annotation processor for hardware devices.
     * Internals are a bit of a mess right now.
     * <p> </p>
     * HardwareName is hardcoded because it provides the value of the field.
     */
    static final class DeviceAnnotation<AnnotationT extends Annotation, TargetT> {
        private final Class<AnnotationT> annotationClass;
        private final Class<TargetT> targetClass;
        private final Action<AnnotationT, TargetT> action;

        @FunctionalInterface
        interface Action<AnnotationT extends Annotation, TargetType> {
            void use(AnnotationT annotation, TargetType target);
        }

        DeviceAnnotation(Class<AnnotationT> annotationClass, Class<TargetT> targetClass, Action<AnnotationT, TargetT> action) {
            this.annotationClass = annotationClass;
            this.targetClass = targetClass;
            this.action = action;
        }

        private boolean check(Field t, Object value) {
            if (!t.isAnnotationPresent(annotationClass)) return false;
            AnnotationT annotation = t.getAnnotation(annotationClass);
            if (annotation == null)
                throw new NullPointerException("Annotation is null! (isAnnotationPresent is true but getAnnotation returned null)");
            if (!targetClass.isAssignableFrom(value.getClass()))
                throw new ClassCastException("Annotation " + annotationClass.getName() + " can only be used on " + targetClass.getName() + " (or subclasses)");
            return true;
        }

        void use(Field t, Object value) {
            if (!check(t, value)) return;
            AnnotationT annotation = t.getAnnotation(annotationClass);
            if (annotation == null)
                throw new NullPointerException("Annotation disappeared between check and use");
            action.use(annotation, targetClass.cast(value));
        }
    }

    static final DeviceAnnotation<Reversed, DcMotorSimple> reversed =
            new DeviceAnnotation<>(
                    Reversed.class,
                    DcMotorSimple.class,
                    (annotation, target) -> target.setDirection(DcMotorSimple.Direction.REVERSE)
            );

    static final DeviceAnnotation<Reversed, Encoder> reversedEncoder =
            new DeviceAnnotation<>(
                    Reversed.class,
                    Encoder.class,
                    (annotation, target) -> target.setFlipped(true)
            );

    static final DeviceAnnotation<ZeroPower, DcMotor> zeroPower =
            new DeviceAnnotation<>(
                    ZeroPower.class,
                    DcMotor.class,
                    (annotation, target) -> target.setZeroPowerBehavior(annotation.value())
            );

    static final DeviceAnnotation<AutoClearEncoder, DcMotor> autoClearEncoder =
            new DeviceAnnotation<>(
                    AutoClearEncoder.class,
                    DcMotor.class,
                    (annotation, target) -> {
                        DcMotor.RunMode current = target.getMode();
                        target.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        target.setMode(current);
                    }
            );

    static final DeviceAnnotation<GoBildaExtendedServo, ServoImplEx> goBildaExtendedServo =
            new DeviceAnnotation<>(
                    GoBildaExtendedServo.class,
                    ServoImplEx.class,
                    (annotation, target) -> {
                        // https://github.com/goBILDA-Official/FTC-Servo-Helper-Examples/blob/main/ServoExExample.java
                        target.setPwmRange(new PwmControl.PwmRange(500, 2500));
                    }
            );

    private final HardwareMap thisMap;

    private <A extends Annotation, B extends Annotation> void assertNotAnnotated(
            @NotNull Field field, @NotNull Class<A> invalid, @Nullable Class<B> incompatibility
    ) {
        if (field.getAnnotation(invalid) != null) {
            if (incompatibility != null)
                throw new BadAnnotationSetException(incompatibility, invalid, String.format(
                        "preparing field '%s'", field.getName()
                ));
            throw new IllegalArgumentException(String.format(
                    "@%s is invalid on field '%s'",
                    invalid.getSimpleName(), field.getName()
            ));
        }
    }

    private final boolean isEntireClassOptional;
    private final ArrayList<Exception> initErrors = new ArrayList<>();

    private void fieldAccessFail(@NotNull Field field, Object result) throws RuntimeException {
        if (result == null)
            throw new RuntimeException("Field " + field.getName() + " assign failed: null to " + field.getType().getName());
        throw new RuntimeException("Field " + field.getName() + " assign failed: " + result.getClass().getName() + " to " + field.getType().getName());
    }

    private void matchHardwareName(@NotNull Field field, @NotNull Class<?> targetType, @NotNull HardwareName annotation, boolean optional) {
        Object result = thisMap.tryGet(targetType, annotation.value());

        if (result == null) {
            if (!optional) {
                initErrors.add(
                        new RuntimeException("Hardware: '" + annotation.value() + "' not found, expected type " + targetType.getName() + " for field " + field.getName() + " in " + this.getClass().getSimpleName())
                );
            }
            try {
                field.set(this, null);
            } catch (IllegalAccessException | IllegalArgumentException e) {
                fieldAccessFail(field, null);
            }
            return;
        }
        try {
            field.set(this, result);
        } catch (IllegalAccessException | IllegalArgumentException e) {
            fieldAccessFail(field, result);
        }

        reversed.use(field, result);
        zeroPower.use(field, result);
        autoClearEncoder.use(field, result);
        goBildaExtendedServo.use(field, result);
    }

    private void matchEncoderFor(@NotNull Field field, @NotNull Class<?> targetType, @NotNull EncoderFor annotation, boolean optional) {
        DcMotor result = thisMap.tryGet(DcMotor.class, annotation.value());
        if (!targetType.isAssignableFrom(Encoder.class)) throw new ClassCastException(String.format(
                "Hardware: cannot assign Encoder to field '%s' with type %s, in class %s",
                field.getName(), targetType.getSimpleName(), this.getClass().getSimpleName()
        ));

        if (result == null) {
            if (!optional) {
                initErrors.add(
                        new RuntimeException("Hardware: '" + annotation.value() + "' not found, expecting a DcMotor to drive a Encoder for field " + field.getName() + " in " + this.getClass().getSimpleName())
                );
                return;
            }
            try {
                field.set(this, null);
            } catch (IllegalAccessException | IllegalArgumentException e) {
                fieldAccessFail(field, null);
            }
            return;
        }

        Encoder wrapper = new MotorEncoder(result);
        try {
            field.set(this, wrapper);
        } catch (IllegalAccessException | IllegalArgumentException e) {
            fieldAccessFail(field, result);
        }

        reversedEncoder.use(field, wrapper);
        assertNotAnnotated(field, ZeroPower.class, EncoderFor.class);
        autoClearEncoder.use(field, result);
        assertNotAnnotated(field, GoBildaExtendedServo.class, EncoderFor.class);
    }

    @SuppressLint("DefaultLocale")
    public HardwareMapper(HardwareMap map) {
        isEntireClassOptional = this.getClass().getAnnotation(HWOptional.class) != null;
        thisMap = map;
        Field[] fields = this.getClass().getDeclaredFields();
        for (Field field : fields) {
            boolean accessible = field.isAccessible();
            if (!accessible) field.setAccessible(true);
            try {
                Class<?> targetType = field.getType();
                boolean isOptional = isEntireClassOptional || field.getAnnotation(HWOptional.class) != null;
                HardwareName hardwareNameAnn = field.getAnnotation(HardwareName.class);
                if (hardwareNameAnn != null) {
                    assertNotAnnotated(field, EncoderFor.class, HardwareName.class);
                    matchHardwareName(field, targetType, hardwareNameAnn, isOptional);
                    continue;
                }
                EncoderFor encoderForAnn = field.getAnnotation(EncoderFor.class);
                if (encoderForAnn != null) {
                    assertNotAnnotated(field, HardwareName.class, EncoderFor.class);
                    matchEncoderFor(field, targetType, encoderForAnn, isOptional);
                    continue;
                }

            } catch (IllegalArgumentException e) {
                throw new RuntimeException("Field " + field.getName() + " typecast failed");
            } finally {
                // lock the field back if it was locked
                field.setAccessible(accessible);
            }
        }
        if (!initErrors.isEmpty()) {
            StringBuilder aggregate = new StringBuilder();
            if (initErrors.size() > 1) {
                aggregate.append(String.format("%d hardware errors:\n", initErrors.size()));
            }
            boolean first = true;
            for (Exception exc : initErrors) {
                if (first) first = false;
                else aggregate.append('\n');
                aggregate.append(exc.getLocalizedMessage());
            }
            throw new RuntimeException(aggregate.toString());
        }
        for (Field field : fields) {
            boolean accessible = field.isAccessible();
            if (!accessible) field.setAccessible(true);
            try {
                field.get(this);
            } catch (IllegalAccessException ignored) {
            } finally {
                field.setAccessible(accessible);
            }
        }
    }
}
