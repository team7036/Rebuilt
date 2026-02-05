package frc.robot.util.sendablesv2;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface DashboardProperty {
    String value(); //Dashboard Key
    Class<? extends Transformer<?>> transformer() default Transformer.BlankTransformer.class;
}
