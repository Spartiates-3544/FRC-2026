package frc.mentor.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.BiConsumer;

import dev.doglog.DogLog;
import edu.wpi.first.units.Measure;
public class ExtendedLogger extends DogLog {
    private static ArrayList<Object> registeredInstances = new ArrayList<Object>();
    private static final Map<Class<?>, BiConsumer<String, Object>> handlers = Map.ofEntries(
        Map.entry(boolean.class, (path, object) -> log(path, (boolean)object)),
        Map.entry(boolean[].class, (path, object) -> log(path, (boolean[])object)),
        Map.entry(double.class, (path, object) -> log(path, (double)object)),
        Map.entry(double[].class, (path, object) -> log(path, (double[])object)),
        Map.entry(Double.class, (path, object) -> log(path, (Double)object)),
        Map.entry(Double[].class, (path, object) -> log(path, (double[])object)),
        Map.entry(Measure.class, (path, object) -> log(path, (Measure<?>)object)),
        Map.entry(float.class, (path, object) -> log(path, (float)object)),
        Map.entry(float[].class, (path, object) -> log(path, (float[])object)),
        Map.entry(int.class, (path, object) -> log(path, (int)object)),
        Map.entry(int[].class, (path, object) -> log(path, (int[])object)),
        Map.entry(long.class, (path, object) -> log(path, (long)object)),
        Map.entry(long[].class, (path, object) -> log(path, (long[])object)),
        Map.entry(String.class, (path, object) -> log(path, (String)object)),
        Map.entry(String[].class, (path, object) -> log(path, (String[])object)),
        Map.entry(Record.class, (path, object) -> log(path, (Record)object))
    );

    public static void registerInstance(Object registeredInstance) {
        registeredInstances.add(registeredInstance);
    }

    public static void logAll() {
        for (Object registeredInstance : registeredInstances) {
            for (Field field : registeredInstance.getClass().getDeclaredFields()) {
                if (field.isAnnotationPresent(LoggableField.class)) {
                    LoggableField annotation = field.getAnnotation(LoggableField.class);
                    // String path = registeredInstance.getClass().getSimpleName() + "/" + field.getName();
                    
                    try {
                        field.setAccessible(true);
                        Object fieldValue = field.get(registeredInstance);
                        logGeneric(annotation.path(), fieldValue);
                    } catch (IllegalAccessException e) {
                        logFault("L'accès à l'attribut " + field.getName() + " est restraint par le système de sécurité Java! (L'attribut est-il privé?)");
                    } catch (IllegalArgumentException e) {
                        // En pratique, ne devrait jamais apparaître.
                        logFault(e.getMessage());
                    }
                }
            }
        }
    }

    private static void logGeneric(String path, Object value) {
        BiConsumer<String, Object> handler = handlers.get(value.getClass());
        
        if (handler != null) {
            handler.accept(path, value);
        } else {
            logFault("Impossible de logger la valeur de type " + value.getClass().getSimpleName() + " avec @LoggableField ou logGeneric()!");
        }
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface LoggableField {
        String path();
    }

}
